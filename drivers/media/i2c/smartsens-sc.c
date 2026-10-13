// SPDX-License-Identifier: GPL-2.0
/*
 * SmartSens SC1300CS / SC820CS camera sensor driver.
 *
 * Register tables and timings come from the vendor CamX sensor module
 * binaries (com.qti.sensormodule.icelandmain/icelandfront.bin) of the
 * OnePlus Pad 4; the exposure/gain register layout follows the vendor
 * sensor HAL (com.qti.sensor.iceland*.so).
 *
 * Both parts use a 16-bit register address / 8-bit data I2C interface, run
 * from a 19.2 MHz MCLK and stream RAW10 over a 4-lane MIPI D-PHY link.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/oplus-camres.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define SC_REG_SW_RESET		CCI_REG8(0x0103)
#define SC_REG_STREAM		CCI_REG8(0x0100)
#define SC_REG_CHIP_ID		CCI_REG16(0x3107)	/* 0x3107 high, 0x3108 low */
#define SC_REG_HTS		CCI_REG16(0x320c)	/* line length, sensor clocks */
#define SC_REG_VTS		CCI_REG16(0x320e)	/* frame length lines */
/*
 * Exposure: 20 bits spread over 0x3e00[3:0], 0x3e01[7:0], 0x3e02[7:4],
 * in units of half lines (so the register value is lines << 5).
 */
#define SC_REG_EXP_H		CCI_REG8(0x3e00)
#define SC_REG_EXP_M		CCI_REG8(0x3e01)
#define SC_REG_EXP_L		CCI_REG8(0x3e02)
/* fine gain (1/128 steps, 0x80..0xff), shared by both chips */
#define SC_REG_GAIN_FINE	CCI_REG8(0x3e07)
#define SC_REG_TEST_PATTERN	CCI_REG8(0x4501)
#define SC_TEST_PATTERN_BAR	BIT(3)

#define SC_MCLK_FREQ		19200000

#define SC_EXPOSURE_MIN		1
#define SC_EXPOSURE_DEFAULT	1000
/* VTS is a 16-bit register; keep exposure (lines << 5) inside 20 bits */
#define SC_VTS_MAX		0x7fff

/* analogue gain in 1/128 units: 1x .. 16x */
#define SC_GAIN_STEP		128
#define SC_GAIN_MIN		(1 * SC_GAIN_STEP)
#define SC_GAIN_MAX		(16 * SC_GAIN_STEP)
#define SC_GAIN_DEFAULT		SC_GAIN_MIN

#define SC_MAX_SUPPLIES		4

struct sc_mode {
	u32 width;
	u32 height;
	u32 vts;		/* vendor frame length lines */
	u32 hblank;		/* pixels, at the chip pixel rate */
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct sc_chip_info {
	const char *name;
	u16 chip_id;
	u32 code;
	u8 lanes;
	u64 link_freq;
	u64 pixel_rate;
	u32 exp_vofs;		/* max exposure = vts - exp_vofs */
	u32 gain_coarse_reg;
	const struct cci_reg_sequence *init_regs;
	unsigned int num_init_regs;
	const struct sc_mode *modes;
	unsigned int num_modes;
	const char * const *supplies;
	unsigned int num_supplies;
};

struct sc_sensor {
	struct device *dev;
	const struct sc_chip_info *chip;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[SC_MAX_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	bool id_ok;
};

static inline struct sc_sensor *to_sc(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc_sensor, sd);
}

/*
 * Analogue coarse gain codes for 1x, 2x, 4x, 8x, 16x (vendor HAL table);
 * the fraction inside each octave goes to the fine gain register.
 */
static const u8 sc_gain_coarse[] = { 0x00, 0x08, 0x09, 0x0b, 0x0f };

static const char * const sc_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
};

static const s64 sc820cs_link_freq[] = { 330000000 };
static const s64 sc1300cs_link_freq[] = { 662400000 };

/* SC820CS init (vendor CamX sensormodule, 130 regs after the 0x0103 soft reset) */
static const struct cci_reg_sequence sc820cs_init_regs[] = {
	{ CCI_REG8(0x36e9), 0x80 }, { CCI_REG8(0x37f9), 0x80 },
	{ CCI_REG8(0x36ea), 0x0b }, { CCI_REG8(0x36eb), 0x0c },
	{ CCI_REG8(0x36ec), 0x5a }, { CCI_REG8(0x36ed), 0x28 },
	{ CCI_REG8(0x37fa), 0x0b }, { CCI_REG8(0x37fb), 0x71 },
	{ CCI_REG8(0x37fc), 0x10 }, { CCI_REG8(0x37fd), 0x28 },
	{ CCI_REG8(0x36e9), 0x22 }, { CCI_REG8(0x37f9), 0x24 },
	{ CCI_REG8(0x301f), 0x44 }, { CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3211), 0x04 }, { CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x3215), 0x11 }, { CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x3221), 0x66 }, { CCI_REG8(0x3270), 0x00 },
	{ CCI_REG8(0x3271), 0x00 }, { CCI_REG8(0x3272), 0x00 },
	{ CCI_REG8(0x3273), 0x03 }, { CCI_REG8(0x3301), 0x08 },
	{ CCI_REG8(0x3303), 0x10 }, { CCI_REG8(0x3306), 0x84 },
	{ CCI_REG8(0x3307), 0x04 }, { CCI_REG8(0x3309), 0x88 },
	{ CCI_REG8(0x330a), 0x01 }, { CCI_REG8(0x330b), 0x0c },
	{ CCI_REG8(0x330c), 0x10 }, { CCI_REG8(0x330d), 0x18 },
	{ CCI_REG8(0x330e), 0x30 }, { CCI_REG8(0x330f), 0x04 },
	{ CCI_REG8(0x3310), 0x02 }, { CCI_REG8(0x3314), 0x15 },
	{ CCI_REG8(0x3317), 0x04 }, { CCI_REG8(0x331f), 0x79 },
	{ CCI_REG8(0x3326), 0x0e }, { CCI_REG8(0x3327), 0x0a },
	{ CCI_REG8(0x3329), 0x0b }, { CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x3334), 0x40 }, { CCI_REG8(0x3347), 0x0f },
	{ CCI_REG8(0x335d), 0x60 }, { CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x336c), 0xce }, { CCI_REG8(0x3390), 0x08 },
	{ CCI_REG8(0x3391), 0x09 }, { CCI_REG8(0x3392), 0x0f },
	{ CCI_REG8(0x3393), 0x10 }, { CCI_REG8(0x3394), 0x20 },
	{ CCI_REG8(0x3395), 0x28 }, { CCI_REG8(0x33ad), 0x3c },
	{ CCI_REG8(0x33af), 0x70 }, { CCI_REG8(0x33b2), 0x70 },
	{ CCI_REG8(0x33b3), 0x40 }, { CCI_REG8(0x349f), 0x1e },
	{ CCI_REG8(0x34a6), 0x09 }, { CCI_REG8(0x34a7), 0x0f },
	{ CCI_REG8(0x34a8), 0x30 }, { CCI_REG8(0x34a9), 0x20 },
	{ CCI_REG8(0x34f8), 0x1f }, { CCI_REG8(0x34f9), 0x08 },
	{ CCI_REG8(0x3637), 0x43 }, { CCI_REG8(0x363c), 0x8d },
	{ CCI_REG8(0x3670), 0x4a }, { CCI_REG8(0x3674), 0xf6 },
	{ CCI_REG8(0x3675), 0xdc }, { CCI_REG8(0x3676), 0xcc },
	{ CCI_REG8(0x367c), 0x09 }, { CCI_REG8(0x367d), 0x0f },
	{ CCI_REG8(0x3690), 0x34 }, { CCI_REG8(0x3691), 0x44 },
	{ CCI_REG8(0x3692), 0x55 }, { CCI_REG8(0x3698), 0x86 },
	{ CCI_REG8(0x3699), 0x8d }, { CCI_REG8(0x369a), 0x99 },
	{ CCI_REG8(0x369b), 0xaf }, { CCI_REG8(0x369c), 0x0f },
	{ CCI_REG8(0x369d), 0x1f }, { CCI_REG8(0x36a2), 0x09 },
	{ CCI_REG8(0x36a3), 0x0b }, { CCI_REG8(0x36a4), 0x0f },
	{ CCI_REG8(0x36b0), 0x48 }, { CCI_REG8(0x36b1), 0x38 },
	{ CCI_REG8(0x36b2), 0x41 }, { CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0xc1 }, { CCI_REG8(0x3771), 0x07 },
	{ CCI_REG8(0x3772), 0x03 }, { CCI_REG8(0x3773), 0x63 },
	{ CCI_REG8(0x377a), 0x08 }, { CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3901), 0x04 }, { CCI_REG8(0x3903), 0xa0 },
	{ CCI_REG8(0x3905), 0x8d }, { CCI_REG8(0x391d), 0x01 },
	{ CCI_REG8(0x3926), 0x23 }, { CCI_REG8(0x393f), 0x80 },
	{ CCI_REG8(0x3940), 0x00 }, { CCI_REG8(0x3941), 0x00 },
	{ CCI_REG8(0x3942), 0x00 }, { CCI_REG8(0x3943), 0x63 },
	{ CCI_REG8(0x3944), 0x5f }, { CCI_REG8(0x3c04), 0x01 },
	{ CCI_REG8(0x3e00), 0x01 }, { CCI_REG8(0x3e01), 0x38 },
	{ CCI_REG8(0x3e02), 0x00 }, { CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 }, { CCI_REG8(0x4403), 0x0e },
	{ CCI_REG8(0x4404), 0x28 }, { CCI_REG8(0x4405), 0x34 },
	{ CCI_REG8(0x4407), 0x0e }, { CCI_REG8(0x440c), 0x42 },
	{ CCI_REG8(0x440d), 0x42 }, { CCI_REG8(0x440e), 0x32 },
	{ CCI_REG8(0x440f), 0x53 }, { CCI_REG8(0x4412), 0x01 },
	{ CCI_REG8(0x4424), 0x01 }, { CCI_REG8(0x442d), 0x00 },
	{ CCI_REG8(0x442e), 0x00 }, { CCI_REG8(0x4509), 0x28 },
	{ CCI_REG8(0x450d), 0x18 }, { CCI_REG8(0x451d), 0xc8 },
	{ CCI_REG8(0x4526), 0x09 }, { CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x5900), 0x01 }, { CCI_REG8(0x5901), 0x00 },
};

/* SC820CS 3264x2448 full, 30 fps */
static const struct cci_reg_sequence sc820cs_full_regs[] = {
	{ CCI_REG8(0x3200), 0x00 }, { CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 }, { CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x0c }, { CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3206), 0x09 }, { CCI_REG8(0x3207), 0x9f },
	{ CCI_REG8(0x3208), 0x0c }, { CCI_REG8(0x3209), 0xc0 },
	{ CCI_REG8(0x320a), 0x09 }, { CCI_REG8(0x320b), 0x90 },
	{ CCI_REG8(0x3210), 0x00 }, { CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3212), 0x00 }, { CCI_REG8(0x3213), 0x08 },
	{ CCI_REG8(0x3215), 0x11 }, { CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x3221), 0x66 }, { CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x5900), 0x01 }, { CCI_REG8(0x5901), 0x00 },
};

/* SC820CS 1632x1224 2x2 binned, 30 fps */
static const struct cci_reg_sequence sc820cs_bin_regs[] = {
	{ CCI_REG8(0x3200), 0x00 }, { CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 }, { CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x0c }, { CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3206), 0x09 }, { CCI_REG8(0x3207), 0x9f },
	{ CCI_REG8(0x3208), 0x06 }, { CCI_REG8(0x3209), 0x60 },
	{ CCI_REG8(0x320a), 0x04 }, { CCI_REG8(0x320b), 0xc8 },
	{ CCI_REG8(0x3210), 0x00 }, { CCI_REG8(0x3211), 0x02 },
	{ CCI_REG8(0x3212), 0x00 }, { CCI_REG8(0x3213), 0x04 },
	{ CCI_REG8(0x3215), 0x31 }, { CCI_REG8(0x3220), 0x01 },
	{ CCI_REG8(0x3221), 0x66 }, { CCI_REG8(0x5000), 0x4e },
	{ CCI_REG8(0x5900), 0xf1 }, { CCI_REG8(0x5901), 0x04 },
};

/* SC1300CS init (vendor CamX sensormodule, 164 regs after the 0x0103 soft reset) */
static const struct cci_reg_sequence sc1300cs_init_regs[] = {
	{ CCI_REG8(0x0100), 0x00 }, { CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x37f9), 0x80 }, { CCI_REG8(0x36ea), 0x17 },
	{ CCI_REG8(0x36eb), 0x0c }, { CCI_REG8(0x36ec), 0x4b },
	{ CCI_REG8(0x36ed), 0x18 }, { CCI_REG8(0x37fa), 0x19 },
	{ CCI_REG8(0x37fb), 0xb2 }, { CCI_REG8(0x37fc), 0x10 },
	{ CCI_REG8(0x37fd), 0x3a }, { CCI_REG8(0x301f), 0x60 },
	{ CCI_REG8(0x320c), 0x04 }, { CCI_REG8(0x320d), 0xe2 },
	{ CCI_REG8(0x320e), 0x0c }, { CCI_REG8(0x320f), 0x80 },
	{ CCI_REG8(0x3279), 0x10 }, { CCI_REG8(0x3301), 0x07 },
	{ CCI_REG8(0x3302), 0x18 }, { CCI_REG8(0x3304), 0x38 },
	{ CCI_REG8(0x3306), 0x58 }, { CCI_REG8(0x3308), 0x0c },
	{ CCI_REG8(0x3309), 0x80 }, { CCI_REG8(0x330b), 0xb8 },
	{ CCI_REG8(0x330d), 0x10 }, { CCI_REG8(0x330e), 0x2b },
	{ CCI_REG8(0x3314), 0x15 }, { CCI_REG8(0x331c), 0x04 },
	{ CCI_REG8(0x331e), 0x29 }, { CCI_REG8(0x331f), 0x71 },
	{ CCI_REG8(0x3320), 0x08 }, { CCI_REG8(0x3323), 0x04 },
	{ CCI_REG8(0x3333), 0x10 }, { CCI_REG8(0x3334), 0x40 },
	{ CCI_REG8(0x334a), 0x82 }, { CCI_REG8(0x3351), 0x06 },
	{ CCI_REG8(0x335d), 0x60 }, { CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x337f), 0x13 }, { CCI_REG8(0x3390), 0x09 },
	{ CCI_REG8(0x3391), 0x0f }, { CCI_REG8(0x3392), 0x1f },
	{ CCI_REG8(0x3393), 0x10 }, { CCI_REG8(0x3394), 0x1c },
	{ CCI_REG8(0x3395), 0x20 }, { CCI_REG8(0x33ac), 0x0a },
	{ CCI_REG8(0x33ad), 0x14 }, { CCI_REG8(0x33ae), 0x14 },
	{ CCI_REG8(0x33af), 0x60 }, { CCI_REG8(0x33b0), 0x0f },
	{ CCI_REG8(0x33b2), 0x50 }, { CCI_REG8(0x33b3), 0x28 },
	{ CCI_REG8(0x349f), 0x1e }, { CCI_REG8(0x34a6), 0x08 },
	{ CCI_REG8(0x34a7), 0x09 }, { CCI_REG8(0x34a8), 0x18 },
	{ CCI_REG8(0x34a9), 0x18 }, { CCI_REG8(0x34f8), 0x0f },
	{ CCI_REG8(0x34f9), 0x18 }, { CCI_REG8(0x3630), 0xc6 },
	{ CCI_REG8(0x3637), 0x47 }, { CCI_REG8(0x363c), 0x8d },
	{ CCI_REG8(0x3650), 0x31 }, { CCI_REG8(0x3651), 0x9d },
	{ CCI_REG8(0x3652), 0x32 }, { CCI_REG8(0x3653), 0x23 },
	{ CCI_REG8(0x3654), 0x31 }, { CCI_REG8(0x3670), 0x0c },
	{ CCI_REG8(0x367b), 0x56 }, { CCI_REG8(0x367c), 0x67 },
	{ CCI_REG8(0x367d), 0x67 }, { CCI_REG8(0x367e), 0x08 },
	{ CCI_REG8(0x367f), 0x0f }, { CCI_REG8(0x3690), 0x84 },
	{ CCI_REG8(0x3691), 0x87 }, { CCI_REG8(0x3692), 0x8e },
	{ CCI_REG8(0x3693), 0xb3 }, { CCI_REG8(0x3694), 0x09 },
	{ CCI_REG8(0x3695), 0x0b }, { CCI_REG8(0x3696), 0x0f },
	{ CCI_REG8(0x3697), 0x8b }, { CCI_REG8(0x3698), 0x8d },
	{ CCI_REG8(0x3699), 0x8d }, { CCI_REG8(0x369a), 0x08 },
	{ CCI_REG8(0x369b), 0x0f }, { CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0x41 }, { CCI_REG8(0x3771), 0x03 },
	{ CCI_REG8(0x3772), 0x03 }, { CCI_REG8(0x3773), 0x63 },
	{ CCI_REG8(0x377a), 0x08 }, { CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3900), 0x0d }, { CCI_REG8(0x3903), 0x20 },
	{ CCI_REG8(0x3905), 0x0c }, { CCI_REG8(0x3908), 0x40 },
	{ CCI_REG8(0x391a), 0x30 }, { CCI_REG8(0x391b), 0x21 },
	{ CCI_REG8(0x391c), 0x13 }, { CCI_REG8(0x391d), 0x00 },
	{ CCI_REG8(0x391f), 0x41 }, { CCI_REG8(0x3926), 0xe0 },
	{ CCI_REG8(0x393f), 0x80 }, { CCI_REG8(0x3940), 0x00 },
	{ CCI_REG8(0x3941), 0x00 }, { CCI_REG8(0x3942), 0x00 },
	{ CCI_REG8(0x3943), 0x7c }, { CCI_REG8(0x3944), 0x79 },
	{ CCI_REG8(0x3945), 0x7d }, { CCI_REG8(0x3946), 0x7d },
	{ CCI_REG8(0x39dd), 0x00 }, { CCI_REG8(0x39de), 0x04 },
	{ CCI_REG8(0x39e7), 0x04 }, { CCI_REG8(0x39e8), 0x03 },
	{ CCI_REG8(0x39e9), 0x80 }, { CCI_REG8(0x3e00), 0x00 },
	{ CCI_REG8(0x3e01), 0x0f }, { CCI_REG8(0x3e02), 0x90 },
	{ CCI_REG8(0x3f08), 0x0a }, { CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 }, { CCI_REG8(0x4403), 0x0c },
	{ CCI_REG8(0x4404), 0x24 }, { CCI_REG8(0x4405), 0x2f },
	{ CCI_REG8(0x440c), 0x3c }, { CCI_REG8(0x440d), 0x3c },
	{ CCI_REG8(0x440e), 0x2d }, { CCI_REG8(0x440f), 0x4b },
	{ CCI_REG8(0x4413), 0x01 }, { CCI_REG8(0x441b), 0x18 },
	{ CCI_REG8(0x4509), 0x18 }, { CCI_REG8(0x450d), 0x13 },
	{ CCI_REG8(0x4800), 0x24 }, { CCI_REG8(0x4819), 0x71 },
	{ CCI_REG8(0x4837), 0x18 }, { CCI_REG8(0x5000), 0x1e },
	{ CCI_REG8(0x5002), 0x00 }, { CCI_REG8(0x5015), 0x41 },
	{ CCI_REG8(0x550e), 0x00 }, { CCI_REG8(0x550f), 0x7a },
	{ CCI_REG8(0x5780), 0x76 }, { CCI_REG8(0x5784), 0x08 },
	{ CCI_REG8(0x5787), 0x0a }, { CCI_REG8(0x5788), 0x0a },
	{ CCI_REG8(0x5789), 0x0a }, { CCI_REG8(0x578a), 0x0a },
	{ CCI_REG8(0x578b), 0x0a }, { CCI_REG8(0x578c), 0x0a },
	{ CCI_REG8(0x578d), 0x40 }, { CCI_REG8(0x5790), 0x08 },
	{ CCI_REG8(0x5791), 0x04 }, { CCI_REG8(0x5792), 0x04 },
	{ CCI_REG8(0x5793), 0x08 }, { CCI_REG8(0x5794), 0x04 },
	{ CCI_REG8(0x5795), 0x04 }, { CCI_REG8(0x5799), 0x44 },
	{ CCI_REG8(0x57a6), 0x1c }, { CCI_REG8(0x57aa), 0xeb },
	{ CCI_REG8(0x57ab), 0x7f }, { CCI_REG8(0x57ac), 0x00 },
	{ CCI_REG8(0x57ad), 0x00 }, { CCI_REG8(0x58e0), 0xb4 },
	{ CCI_REG8(0x36e9), 0x25 }, { CCI_REG8(0x37f9), 0x24 },
};

/* SC1300CS 4160x3120 full, 30 fps */
static const struct cci_reg_sequence sc1300cs_full_regs[] = {
	{ CCI_REG8(0x3200), 0x00 }, { CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 }, { CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x10 }, { CCI_REG8(0x3205), 0x7f },
	{ CCI_REG8(0x3206), 0x0c }, { CCI_REG8(0x3207), 0x3f },
	{ CCI_REG8(0x3208), 0x10 }, { CCI_REG8(0x3209), 0x40 },
	{ CCI_REG8(0x320a), 0x0c }, { CCI_REG8(0x320b), 0x30 },
	{ CCI_REG8(0x3210), 0x00 }, { CCI_REG8(0x3211), 0x20 },
	{ CCI_REG8(0x3212), 0x00 }, { CCI_REG8(0x3213), 0x08 },
	{ CCI_REG8(0x3215), 0x11 }, { CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x3221), 0x66 }, { CCI_REG8(0x5000), 0x1e },
	{ CCI_REG8(0x5900), 0x01 }, { CCI_REG8(0x5901), 0x00 },
};

/* SC1300CS 2104x1560 2x2 binned, 30 fps */
static const struct cci_reg_sequence sc1300cs_bin_regs[] = {
	{ CCI_REG8(0x3200), 0x00 }, { CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 }, { CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x10 }, { CCI_REG8(0x3205), 0x7f },
	{ CCI_REG8(0x3206), 0x0c }, { CCI_REG8(0x3207), 0x3f },
	{ CCI_REG8(0x3208), 0x08 }, { CCI_REG8(0x3209), 0x38 },
	{ CCI_REG8(0x320a), 0x06 }, { CCI_REG8(0x320b), 0x18 },
	{ CCI_REG8(0x3210), 0x00 }, { CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3212), 0x00 }, { CCI_REG8(0x3213), 0x04 },
	{ CCI_REG8(0x3215), 0x31 }, { CCI_REG8(0x3220), 0x01 },
	{ CCI_REG8(0x3221), 0x66 }, { CCI_REG8(0x5000), 0x5e },
	{ CCI_REG8(0x5900), 0x01 }, { CCI_REG8(0x5901), 0x04 },
};

/*
 * SC820CS: HTS 1760 clocks = 3520 pixels at 264 MHz, VTS 2500 -> 30 fps.
 * 660 Mbit/s per lane over 4 lanes.
 */
static const struct sc_mode sc820cs_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.vts = 2500,
		.hblank = 3520 - 3264,
		.regs = sc820cs_full_regs,
		.num_regs = ARRAY_SIZE(sc820cs_full_regs),
	}, {
		.width = 1632,
		.height = 1224,
		.vts = 2500,
		.hblank = 3520 - 1632,
		.regs = sc820cs_bin_regs,
		.num_regs = ARRAY_SIZE(sc820cs_bin_regs),
	},
};

/*
 * SC1300CS: HTS 1250 clocks = 5000 pixels at 480 MHz, VTS 3200 -> 30 fps.
 * 1324.8 Mbit/s per lane over 4 lanes (vendor output pixel clock 529.92 MHz).
 */
static const struct sc_mode sc1300cs_modes[] = {
	{
		.width = 4160,
		.height = 3120,
		.vts = 3200,
		.hblank = 5000 - 4160,
		.regs = sc1300cs_full_regs,
		.num_regs = ARRAY_SIZE(sc1300cs_full_regs),
	}, {
		.width = 2104,
		.height = 1560,
		.vts = 3200,
		.hblank = 5000 - 2104,
		.regs = sc1300cs_bin_regs,
		.num_regs = ARRAY_SIZE(sc1300cs_bin_regs),
	},
};

/* Power-up order: IO rail, analog, digital (then AF for the rear module) */
static const char * const sc1300cs_supplies[] = {
	"dovdd", "avdd", "dvdd", "afvdd",
};

static const char * const sc820cs_supplies[] = {
	"dovdd", "avdd", "dvdd",
};

/*
 * Both vendor tables set mirror + flip (0x3221 = 0x66); the resulting
 * readout order was verified on captured frames to be BGGR.
 */
static const struct sc_chip_info sc1300cs_info = {
	.name = "sc1300cs",
	.chip_id = 0xc658,
	.code = MEDIA_BUS_FMT_SBGGR10_1X10,
	.lanes = 4,
	.link_freq = 662400000,
	.pixel_rate = 480000000,
	.exp_vofs = 5,
	.gain_coarse_reg = CCI_REG8(0x3e09),
	.init_regs = sc1300cs_init_regs,
	.num_init_regs = ARRAY_SIZE(sc1300cs_init_regs),
	.modes = sc1300cs_modes,
	.num_modes = ARRAY_SIZE(sc1300cs_modes),
	.supplies = sc1300cs_supplies,
	.num_supplies = ARRAY_SIZE(sc1300cs_supplies),
};

static const struct sc_chip_info sc820cs_info = {
	.name = "sc820cs",
	.chip_id = 0xd154,
	.code = MEDIA_BUS_FMT_SBGGR10_1X10,
	.lanes = 4,
	.link_freq = 330000000,
	.pixel_rate = 264000000,
	.exp_vofs = 4,
	.gain_coarse_reg = CCI_REG8(0x3e08),
	.init_regs = sc820cs_init_regs,
	.num_init_regs = ARRAY_SIZE(sc820cs_init_regs),
	.modes = sc820cs_modes,
	.num_modes = ARRAY_SIZE(sc820cs_modes),
	.supplies = sc820cs_supplies,
	.num_supplies = ARRAY_SIZE(sc820cs_supplies),
};

static int sc_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc_sensor *sc = to_sc(sd);
	int ret;

	/* the bus is SEE-owned by default: take it before any I2C */
	ret = oplus_camres_acquire(dev);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(sc->chip->num_supplies, sc->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		goto err_bus;
	}

	/* let the rails settle before clocking the sensor */
	fsleep(2000);

	ret = clk_prepare_enable(sc->mclk);
	if (ret) {
		dev_err(dev, "failed to enable mclk: %d\n", ret);
		goto err_reg;
	}

	fsleep(1000);

	/* release reset (physical level high, pin is GPIO_ACTIVE_LOW) */
	gpiod_set_value_cansleep(sc->reset_gpio, 0);

	/* SmartSens parts need a few ms before the first I2C access */
	fsleep(10 * USEC_PER_MSEC);

	return 0;

err_reg:
	regulator_bulk_disable(sc->chip->num_supplies, sc->supplies);
err_bus:
	oplus_camres_release(dev);
	return ret;
}

static int sc_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc_sensor *sc = to_sc(sd);

	gpiod_set_value_cansleep(sc->reset_gpio, 1);
	fsleep(1000);
	clk_disable_unprepare(sc->mclk);
	regulator_bulk_disable(sc->chip->num_supplies, sc->supplies);

	/* last I2C is done: hand the bus back to SEE */
	oplus_camres_release(dev);

	return 0;
}

static void sc_identify(struct sc_sensor *sc)
{
	u64 val = 0;
	int ret;

	ret = cci_read(sc->regmap, SC_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_warn(sc->dev, "%s: chip id read failed: %d (no I2C ack?)\n",
			 sc->chip->name, ret);
		return;
	}

	if (val == sc->chip->chip_id) {
		sc->id_ok = true;
		dev_info(sc->dev, "%s: chip id 0x%04llx OK\n",
			 sc->chip->name, val);
	} else {
		dev_warn(sc->dev, "%s: chip id 0x%04llx, expected 0x%04x\n",
			 sc->chip->name, val, sc->chip->chip_id);
	}
}

static const struct sc_mode *sc_find_mode(const struct sc_chip_info *chip,
					  u32 width, u32 height)
{
	return v4l2_find_nearest_size(chip->modes, chip->num_modes,
				      width, height, width, height);
}

static const struct sc_mode *sc_state_mode(struct sc_sensor *sc,
					   struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, 0);

	return sc_find_mode(sc->chip, fmt->width, fmt->height);
}

static void sc_fill_fmt(const struct sc_chip_info *chip,
			const struct sc_mode *mode,
			struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = chip->code;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

/* Called with the control handler lock held */
static void sc_update_mode_ctrls(struct sc_sensor *sc, const struct sc_mode *mode)
{
	u32 exp_max;

	__v4l2_ctrl_modify_range(sc->hblank, mode->hblank, mode->hblank, 1,
				 mode->hblank);
	__v4l2_ctrl_modify_range(sc->vblank, mode->vts - mode->height,
				 SC_VTS_MAX - mode->height, 1,
				 mode->vts - mode->height);
	__v4l2_ctrl_s_ctrl(sc->vblank, mode->vts - mode->height);

	exp_max = mode->vts - sc->chip->exp_vofs;
	__v4l2_ctrl_modify_range(sc->exposure, SC_EXPOSURE_MIN, exp_max, 1,
				 min_t(u32, SC_EXPOSURE_DEFAULT, exp_max));
}

static int sc_enum_mbus_code(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc_sensor *sc = to_sc(sd);

	if (code->index)
		return -EINVAL;

	code->code = sc->chip->code;

	return 0;
}

static int sc_enum_frame_size(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc_sensor *sc = to_sc(sd);
	const struct sc_mode *mode;

	if (fse->index >= sc->chip->num_modes || fse->code != sc->chip->code)
		return -EINVAL;

	mode = &sc->chip->modes[fse->index];
	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

static int sc_set_fmt(struct v4l2_subdev *sd,
		      struct v4l2_subdev_state *state,
		      struct v4l2_subdev_format *fmt)
{
	struct sc_sensor *sc = to_sc(sd);
	const struct sc_mode *mode;

	mode = sc_find_mode(sc->chip, fmt->format.width, fmt->format.height);
	sc_fill_fmt(sc->chip, mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;

	/* the state lock is the control handler lock */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		sc_update_mode_ctrls(sc, mode);

	return 0;
}

static int sc_write_exposure(struct sc_sensor *sc, u32 lines)
{
	u32 val = lines << 5;	/* half lines, left aligned in 0x3e02[7:4] */
	int ret = 0;

	cci_write(sc->regmap, SC_REG_EXP_H, (val >> 16) & 0x0f, &ret);
	cci_write(sc->regmap, SC_REG_EXP_M, (val >> 8) & 0xff, &ret);
	cci_write(sc->regmap, SC_REG_EXP_L, val & 0xf0, &ret);

	return ret;
}

static int sc_write_gain(struct sc_sensor *sc, u32 gain)
{
	unsigned int octave = fls(gain / SC_GAIN_STEP) - 1;
	int ret = 0;

	octave = min_t(unsigned int, octave, ARRAY_SIZE(sc_gain_coarse) - 1);

	cci_write(sc->regmap, sc->chip->gain_coarse_reg, sc_gain_coarse[octave],
		  &ret);
	/* remaining fraction of the octave, 0x80..0xff */
	cci_write(sc->regmap, SC_REG_GAIN_FINE,
		  min_t(u32, gain >> octave, 0xff), &ret);

	return ret;
}

static int sc_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc_sensor *sc = container_of(ctrl->handler, struct sc_sensor,
					    ctrls);
	struct v4l2_subdev_state *state;
	const struct sc_mode *mode;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&sc->sd);
	mode = sc_state_mode(sc, state);

	/* keep the exposure limits in sync with the frame length */
	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 exp_max = mode->height + ctrl->val - sc->chip->exp_vofs;

		__v4l2_ctrl_modify_range(sc->exposure, SC_EXPOSURE_MIN, exp_max,
					 1, min_t(u32, sc->exposure->default_value,
						  exp_max));
	}

	if (!pm_runtime_get_if_in_use(sc->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = sc_write_exposure(sc, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sc_write_gain(sc, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(sc->regmap, SC_REG_VTS, mode->height + ctrl->val,
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_update_bits(sc->regmap, SC_REG_TEST_PATTERN,
				      SC_TEST_PATTERN_BAR,
				      ctrl->val ? SC_TEST_PATTERN_BAR : 0, NULL);
		break;
	default:
		break;
	}

	pm_runtime_mark_last_busy(sc->dev);
	pm_runtime_put_autosuspend(sc->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc_ctrl_ops = {
	.s_ctrl = sc_set_ctrl,
};

static int sc_enable_streams(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     u32 pad, u64 streams_mask)
{
	struct sc_sensor *sc = to_sc(sd);
	const struct sc_chip_info *chip = sc->chip;
	const struct sc_mode *mode = sc_state_mode(sc, state);
	int ret;

	ret = pm_runtime_resume_and_get(sc->dev);
	if (ret < 0)
		return ret;

	ret = cci_write(sc->regmap, SC_REG_SW_RESET, 0x01, NULL);
	if (ret)
		goto err_pm;
	fsleep(2000);

	ret = cci_multi_reg_write(sc->regmap, chip->init_regs,
				  chip->num_init_regs, NULL);
	if (ret) {
		dev_err(sc->dev, "failed to write init table: %d\n", ret);
		goto err_pm;
	}

	ret = cci_multi_reg_write(sc->regmap, mode->regs, mode->num_regs, NULL);
	if (ret) {
		dev_err(sc->dev, "failed to write %ux%u mode table: %d\n",
			mode->width, mode->height, ret);
		goto err_pm;
	}

	/* exposure, gain, frame length, test pattern */
	ret = __v4l2_ctrl_handler_setup(&sc->ctrls);
	if (ret)
		goto err_pm;

	ret = cci_write(sc->regmap, SC_REG_STREAM, 0x01, NULL);
	if (ret)
		goto err_pm;

	dev_dbg(sc->dev, "%s: streaming %ux%u\n", chip->name, mode->width,
		mode->height);

	return 0;

err_pm:
	pm_runtime_mark_last_busy(sc->dev);
	pm_runtime_put_autosuspend(sc->dev);
	return ret;
}

static int sc_disable_streams(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      u32 pad, u64 streams_mask)
{
	struct sc_sensor *sc = to_sc(sd);

	cci_write(sc->regmap, SC_REG_STREAM, 0x00, NULL);

	pm_runtime_mark_last_busy(sc->dev);
	pm_runtime_put_autosuspend(sc->dev);

	return 0;
}

static int sc_init_state(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state)
{
	struct sc_sensor *sc = to_sc(sd);

	sc_fill_fmt(sc->chip, &sc->chip->modes[0],
		    v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops sc_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sc_pad_ops = {
	.enum_mbus_code = sc_enum_mbus_code,
	.enum_frame_size = sc_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sc_set_fmt,
	.enable_streams = sc_enable_streams,
	.disable_streams = sc_disable_streams,
};

static const struct v4l2_subdev_ops sc_subdev_ops = {
	.video = &sc_video_ops,
	.pad = &sc_pad_ops,
};

static const struct v4l2_subdev_internal_ops sc_internal_ops = {
	.init_state = sc_init_state,
};

static int sc_init_controls(struct sc_sensor *sc)
{
	const struct sc_chip_info *chip = sc->chip;
	const struct sc_mode *mode = &chip->modes[0];
	struct v4l2_ctrl_handler *hdl = &sc->ctrls;
	const s64 *link_freq = chip == &sc1300cs_info ? sc1300cs_link_freq :
							sc820cs_link_freq;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 9);
	if (ret)
		return ret;

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
				 chip->pixel_rate, chip->pixel_rate, 1,
				 chip->pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sc->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
				       mode->hblank, mode->hblank, 1,
				       mode->hblank);
	if (sc->hblank)
		sc->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sc->vblank = v4l2_ctrl_new_std(hdl, &sc_ctrl_ops, V4L2_CID_VBLANK,
				       mode->vts - mode->height,
				       SC_VTS_MAX - mode->height, 1,
				       mode->vts - mode->height);

	sc->exposure = v4l2_ctrl_new_std(hdl, &sc_ctrl_ops, V4L2_CID_EXPOSURE,
					 SC_EXPOSURE_MIN,
					 mode->vts - chip->exp_vofs, 1,
					 SC_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, &sc_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  SC_GAIN_MIN, SC_GAIN_MAX, 1, SC_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(hdl, &sc_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(sc_test_pattern_menu) - 1, 0, 0,
				     sc_test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	/* orientation/rotation from DT: libcamera derives Location from these */
	ret = v4l2_fwnode_device_parse(sc->dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &sc_ctrl_ops, &props);
	if (ret)
		goto err_free;

	sc->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int sc_parse_endpoint(struct sc_sensor *sc)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(sc->dev), 0, 0, 0);
	if (!ep)
		return dev_err_probe(sc->dev, -EINVAL, "no endpoint node\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(sc->dev, ret, "could not parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != sc->chip->lanes) {
		ret = dev_err_probe(sc->dev, -EINVAL,
				    "%u data lanes in DT, %s needs %u\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes,
				    sc->chip->name, sc->chip->lanes);
		goto out;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == sc->chip->link_freq)
			break;
	if (i == bus_cfg.nr_of_link_frequencies)
		ret = dev_err_probe(sc->dev, -EINVAL,
				    "link frequency %llu Hz not listed in DT\n",
				    sc->chip->link_freq);

out:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int sc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc_sensor *sc;
	unsigned long rate;
	unsigned int i;
	int ret;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = dev;
	sc->chip = device_get_match_data(dev);
	if (!sc->chip)
		return -ENODEV;

	ret = sc_parse_endpoint(sc);
	if (ret)
		return ret;

	sc->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sc->mclk))
		return dev_err_probe(dev, PTR_ERR(sc->mclk),
				     "failed to get mclk\n");

	rate = clk_get_rate(sc->mclk);
	if (rate != SC_MCLK_FREQ)
		dev_warn(dev, "mclk rate %lu Hz, expected %u Hz\n",
			 rate, SC_MCLK_FREQ);

	/* reset asserted (physical low) until power-on */
	sc->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sc->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sc->reset_gpio),
				     "failed to get reset gpio\n");

	for (i = 0; i < sc->chip->num_supplies; i++)
		sc->supplies[i].supply = sc->chip->supplies[i];
	ret = devm_regulator_bulk_get(dev, sc->chip->num_supplies,
				      sc->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	sc->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sc->regmap))
		return dev_err_probe(dev, PTR_ERR(sc->regmap),
				     "failed to init CCI regmap\n");

	v4l2_i2c_subdev_init(&sc->sd, client, &sc_subdev_ops);
	snprintf(sc->sd.name, sizeof(sc->sd.name), "%s %d-%04x",
		 sc->chip->name, i2c_adapter_id(client->adapter), client->addr);

	/* camres reprobes us once SEE hands out the shared bus */
	if (!oplus_camres_is_ready())
		return -EPROBE_DEFER;

	ret = sc_power_on(dev);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	/* bring-up: report the ID but keep the device registered either way */
	sc_identify(sc);

	ret = sc_init_controls(sc);
	if (ret)
		goto err_power_off;

	sc->sd.internal_ops = &sc_internal_ops;
	sc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sc->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sc->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sc->sd.entity, 1, &sc->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto err_ctrls;
	}

	sc->sd.state_lock = sc->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sc->sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto err_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&sc->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_subdev;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_subdev:
	v4l2_subdev_cleanup(&sc->sd);
err_entity:
	media_entity_cleanup(&sc->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sc->ctrls);
err_power_off:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	sc_power_off(dev);

	return ret;
}

static void sc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc_sensor *sc = to_sc(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sc->ctrls);

	pm_runtime_disable(sc->dev);
	if (!pm_runtime_status_suspended(sc->dev)) {
		sc_power_off(sc->dev);
		pm_runtime_set_suspended(sc->dev);
	}
}

static const struct of_device_id sc_of_match[] = {
	{ .compatible = "smartsens,sc1300cs", .data = &sc1300cs_info },
	{ .compatible = "smartsens,sc820cs", .data = &sc820cs_info },
	{ }
};
MODULE_DEVICE_TABLE(of, sc_of_match);

static DEFINE_RUNTIME_DEV_PM_OPS(sc_pm_ops, sc_power_off, sc_power_on, NULL);

static struct i2c_driver sc_i2c_driver = {
	.driver = {
		.name = "smartsens-sc",
		.of_match_table = sc_of_match,
		.pm = pm_ptr(&sc_pm_ops),
	},
	.probe = sc_probe,
	.remove = sc_remove,
};
module_i2c_driver(sc_i2c_driver);

MODULE_DESCRIPTION("SmartSens SC1300CS/SC820CS camera sensor driver");
MODULE_LICENSE("GPL");
