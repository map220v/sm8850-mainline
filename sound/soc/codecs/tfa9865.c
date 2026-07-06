// SPDX-License-Identifier: GPL-2.0-only
/*
 * Goodix TFA9865 Smart AMP driver
 *
 * Copyright (c) 2026 map220v <map220v300@gmail.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>

#define TFA9865_DEVICE_CONTROL		0x00
#define TFA9865_POWERDOWN_MASK		0x0001
#define TFA9865_POWERDOWN_SHIFT		0
#define TFA9865_RESET_MASK		0x0002
#define TFA9865_ENBLAMP_SHIFT		3

#define TFA9865_I2S_FS_8000		0
#define TFA9865_I2S_FS_11025		1
#define TFA9865_I2S_FS_12000		2
#define TFA9865_I2S_FS_16000		3
#define TFA9865_I2S_FS_22050		4
#define TFA9865_I2S_FS_24000		5
#define TFA9865_I2S_FS_32000		6
#define TFA9865_I2S_FS_44100		7
#define TFA9865_I2S_FS_48000		8
#define TFA9865_I2S_FS_64000		9
#define TFA9865_I2S_FS_88200		10
#define TFA9865_I2S_FS_96000		11
#define TFA9865_I2S_FS_176400		12
#define TFA9865_I2S_FS_192000		13

#define TFA9865_FS_CONTROL		0x02
#define TFA9865_FS_MASK			0x000f
#define TFA9865_FS_SHIFT		0

#define TFA9865_PRODUCT			0x03

#define TFA9865_CLK_CONTROL		0x04
#define TFA9865_MANAOOSC_MASK		0x0040

#define TFA9865_HIDDEN_CODE		0x0f
#define TFA9865_HIDDEN_CODE_VAL		0x5a6b

#define TFA9865_STATUS2			0x12
#define TFA9865_MANSTATE_MASK		0x000f

#define TFA9865_TDM_CONTROL0		0x20
#define TFA9865_TDM_SLOTLEN_MASK	0x0300
#define TFA9865_TDM_SLOTLEN_SHIFT	8
#define TFA9865_TDM_SAMPLSZ_MASK	0x0c00
#define TFA9865_TDM_SAMPLSZ_SHIFT	10

#define TFA9865_TDM_16			0
#define TFA9865_TDM_24			2
#define TFA9865_TDM_32			3

#define TFA9865_INT_CLEAR		0x44
#define TFA9865_INT_ENABLE		0x48

#define TFA9865_MISC_CONTROL		0x58
#define TFA9865_H_MUTE_MASK		0x0002
#define TFA9865_H_MUTE_SHIFT		1

#define TFA9865_KEY1			0xa0
#define TFA9865_KEY2			0xa1
#define TFA9865_KEY_LOCK		0x0000
#define TFA9865_KEY_UNLOCK		0x005a

#define TFA9865_NOLPM_MASK		0x0200

#define TFA9865_TYPESEL_MASK		0x1800

#define TFA9865_LIMITDCDC_MASK		0x8000

#define TFA9865_EFUSE_B			0xfb

struct tfa9865_priv {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
};

struct tfa9865_reg_data {
	u8 reg;
	u16 val;
};

struct tfa9865_device_config {
	unsigned int reg_count;
	const struct tfa9865_reg_data *reg_data;
};

struct tfa9865_static_config {
	unsigned int dev_count;
	const struct tfa9865_device_config *dev_config;
};

static const struct tfa9865_reg_data tfa9865_iceland_regs_0__34_36[] = {
	{ .reg = 0x20, .val = 0x0021 },
	{ .reg = 0x21, .val = 0x100f },
	{ .reg = 0x02, .val = 0x0c28 },
	{ .reg = 0x87, .val = 0x0000 },
	{ .reg = 0x67, .val = 0x0621 },
	{ .reg = 0x7c, .val = 0x50f2 },
	{ .reg = 0x63, .val = 0x90d4 },
	{ .reg = 0x62, .val = 0x05c6 },
	{ .reg = 0x5a, .val = 0x2d8e },
	{ .reg = 0x74, .val = 0x6120 },
	{ .reg = 0x08, .val = 0x0042 },
	{ .reg = 0x5b, .val = 0x6095 },
	{ .reg = 0x5c, .val = 0x330b },
	{ .reg = 0x01, .val = 0x0084 },
	{ .reg = 0x00, .val = 0xf251 },
};

static const struct tfa9865_reg_data tfa9865_iceland_regs_0__37[] = {
	{ .reg = 0x20, .val = 0x0021 },
	{ .reg = 0x21, .val = 0x100f },
	{ .reg = 0x02, .val = 0x0c28 },
	{ .reg = 0x87, .val = 0x0000 },
	{ .reg = 0x67, .val = 0x0616 },
	{ .reg = 0x7c, .val = 0x50f2 },
	{ .reg = 0x63, .val = 0x90d4 },
	{ .reg = 0x62, .val = 0x05c6 },
	{ .reg = 0x5a, .val = 0x2d8e },
	{ .reg = 0x74, .val = 0x6120 },
	{ .reg = 0x08, .val = 0x0042 },
	{ .reg = 0x5b, .val = 0x6095 },
	{ .reg = 0x5c, .val = 0x330b },
	{ .reg = 0x01, .val = 0x0084 },
	{ .reg = 0x76, .val = 0xd700 },
	{ .reg = 0x5f, .val = 0x00e0 },
	{ .reg = 0x00, .val = 0xf251 },
};

static const struct tfa9865_reg_data tfa9865_iceland_regs_22__34_36[] = {
	{ .reg = 0x20, .val = 0x0001 },
	{ .reg = 0x21, .val = 0x100f },
	{ .reg = 0x02, .val = 0x0c28 },
	{ .reg = 0x87, .val = 0x0000 },
	{ .reg = 0x67, .val = 0x0621 },
	{ .reg = 0x7c, .val = 0x50f2 },
	{ .reg = 0x63, .val = 0x90d4 },
	{ .reg = 0x62, .val = 0x05c6 },
	{ .reg = 0x5a, .val = 0x2d8e },
	{ .reg = 0x74, .val = 0x6120 },
	{ .reg = 0x08, .val = 0x0042 },
	{ .reg = 0x5b, .val = 0x6095 },
	{ .reg = 0x5c, .val = 0x330b },
	{ .reg = 0x01, .val = 0x0084 },
	{ .reg = 0x00, .val = 0xf251 },
};

static const struct tfa9865_reg_data tfa9865_iceland_regs_22__37[] = {
	{ .reg = 0x20, .val = 0x0001 },
	{ .reg = 0x21, .val = 0x100f },
	{ .reg = 0x02, .val = 0x0c28 },
	{ .reg = 0x87, .val = 0x0000 },
	{ .reg = 0x67, .val = 0x0616 },
	{ .reg = 0x7c, .val = 0x50f2 },
	{ .reg = 0x63, .val = 0x90d4 },
	{ .reg = 0x62, .val = 0x05c6 },
	{ .reg = 0x5a, .val = 0x2d8e },
	{ .reg = 0x74, .val = 0x6120 },
	{ .reg = 0x08, .val = 0x0042 },
	{ .reg = 0x5b, .val = 0x6095 },
	{ .reg = 0x5c, .val = 0x330b },
	{ .reg = 0x01, .val = 0x0084 },
	{ .reg = 0x76, .val = 0xd700 },
	{ .reg = 0x5f, .val = 0x00e0 },
	{ .reg = 0x00, .val = 0xf251 },
};

static const struct tfa9865_device_config tfa9865_iceland_devices[] = {
	{
		.reg_count = ARRAY_SIZE(tfa9865_iceland_regs_0__34_36),
		.reg_data = tfa9865_iceland_regs_0__34_36,
	},
	{
		.reg_count = ARRAY_SIZE(tfa9865_iceland_regs_0__34_36),
		.reg_data = tfa9865_iceland_regs_0__34_36,
	},
	{
		.reg_count = ARRAY_SIZE(tfa9865_iceland_regs_0__37),
		.reg_data = tfa9865_iceland_regs_0__37,
	},
	{
		.reg_count = ARRAY_SIZE(tfa9865_iceland_regs_22__34_36),
		.reg_data = tfa9865_iceland_regs_22__34_36,
	},
	{
		.reg_count = ARRAY_SIZE(tfa9865_iceland_regs_22__34_36),
		.reg_data = tfa9865_iceland_regs_22__34_36,
	},
	{
		.reg_count = ARRAY_SIZE(tfa9865_iceland_regs_22__37),
		.reg_data = tfa9865_iceland_regs_22__37,
	},
};

static const struct tfa9865_static_config tfa9865_iceland_config = {
	.dev_count = ARRAY_SIZE(tfa9865_iceland_devices),
	.dev_config = tfa9865_iceland_devices,
};

static int tfa9865_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	int fs = 48000;
	int tdm_set = 0;

	switch (params_rate(params)) {
	case 8000:
		fs = TFA9865_I2S_FS_8000;
		break;
	case 11025:
		fs = TFA9865_I2S_FS_11025;
		break;
	case 12000:
		fs = TFA9865_I2S_FS_12000;
		break;
	case 16000:
		fs = TFA9865_I2S_FS_16000;
		break;
	case 22050:
		fs = TFA9865_I2S_FS_22050;
		break;
	case 24000:
		fs = TFA9865_I2S_FS_24000;
		break;
	case 32000:
		fs = TFA9865_I2S_FS_32000;
		break;
	case 44100:
		fs = TFA9865_I2S_FS_44100;
		break;
	case 48000:
		fs = TFA9865_I2S_FS_48000;
		break;
	case 64000:
		fs = TFA9865_I2S_FS_64000;
		break;
	case 88200:
		fs = TFA9865_I2S_FS_88200;
		break;
	case 96000:
		fs = TFA9865_I2S_FS_96000;
		break;
	case 176400:
		fs = TFA9865_I2S_FS_176400;
		break;
	case 192000:
		fs = TFA9865_I2S_FS_192000;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		tdm_set = TFA9865_TDM_16;
		break;
	case 24:
		tdm_set = TFA9865_TDM_24;
		break;
	case 32:
		tdm_set = TFA9865_TDM_32;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, TFA9865_TDM_CONTROL0,
				      TFA9865_TDM_SLOTLEN_MASK,
				      tdm_set << TFA9865_TDM_SLOTLEN_SHIFT);
	snd_soc_component_update_bits(component, TFA9865_TDM_CONTROL0,
				      TFA9865_TDM_SAMPLSZ_MASK,
				      tdm_set << TFA9865_TDM_SAMPLSZ_SHIFT);

	snd_soc_component_update_bits(component, TFA9865_FS_CONTROL,
				      TFA9865_FS_MASK,
				      fs << TFA9865_FS_SHIFT);
	return 0;
}

static int tfa9865_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;

	snd_soc_component_update_bits(component, TFA9865_MISC_CONTROL,
				      TFA9865_H_MUTE_MASK,
				      !!mute << TFA9865_H_MUTE_SHIFT);

	return 0;
}

static const struct snd_soc_dapm_widget tfa9865_dapm_widgets[] = {
SND_SOC_DAPM_AIF_IN("AIFINL", "Playback", 0, SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_AIF_IN("AIFINR", "Playback", 1, SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_OUT_DRV("AMP", TFA9865_DEVICE_CONTROL, TFA9865_ENBLAMP_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("LINEOUT"),
SND_SOC_DAPM_SUPPLY("POWER", TFA9865_DEVICE_CONTROL, TFA9865_POWERDOWN_SHIFT, 1,
		    NULL, 0),
};

static const struct snd_soc_dapm_route tfa9865_dapm_routes[] = {
	{ "AMP", NULL, "AIFINL" },
	{ "AMP", NULL, "AIFINR" },

	{ "LINEOUT", NULL, "AMP" },

	{ "AMP", NULL, "POWER" },
};

static const struct snd_soc_component_driver tfa9865_component = {
	//.controls		= tfa9865_controls,
	//.num_controls		= ARRAY_SIZE(tfa9865_controls),
	.dapm_widgets		= tfa9865_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tfa9865_dapm_widgets),
	.dapm_routes		= tfa9865_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tfa9865_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct regmap_config tfa9865_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static const struct snd_soc_dai_ops tfa9865_dai_ops = {
	.hw_params = tfa9865_hw_params,
	.mute_stream = tfa9865_mute_stream,
	.no_capture_mute = 1,
};

#define TFA9865_RATES SNDRV_PCM_RATE_8000_192000

#define TFA9865_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S24_LE | \
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver tfa9865_dai = {
	.name = "tfa9865-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = TFA9865_RATES,
		.formats = TFA9865_FORMATS, },
	.ops = &tfa9865_dai_ops,
};

static int tfa9865_init(struct tfa9865_priv *tfa9865)
{
	u32 val, version, product = 0;
	u32 variant = 0;
	u16 key = 0;
	int ret;

	enum variant {
		N = 0,
		MN = 1
	};

	regmap_write(tfa9865->regmap, TFA9865_DEVICE_CONTROL, TFA9865_RESET_MASK);

	ret = regmap_read(tfa9865->regmap, TFA9865_PRODUCT, &product);
	if (ret || product != 0x9866)
		return dev_err_probe(tfa9865->dev, -EINVAL, "Failed to get product id expected=%x got=%x\n",
				     0x9866, product);

	regmap_update_bits(tfa9865->regmap, TFA9865_DEVICE_CONTROL, TFA9865_POWERDOWN_MASK, 0);
	regmap_update_bits(tfa9865->regmap, TFA9865_CLK_CONTROL, TFA9865_MANAOOSC_MASK, 0);
	regmap_read_poll_timeout(tfa9865->regmap, TFA9865_STATUS2, val,
				 val & TFA9865_MANSTATE_MASK, 1, 50);

	regmap_read(tfa9865->regmap, 0xf3, &val);
	if (val & TFA9865_TYPESEL_MASK) {
		product--; //TFA9865
		regmap_read(tfa9865->regmap, 0xf4, &val);
		if (val & TFA9865_LIMITDCDC_MASK)
			product--; //TFA9864
	}

	regmap_read(tfa9865->regmap, 0xf0, &val);
	if (val & TFA9865_NOLPM_MASK)
		variant = N;
	else
		variant = MN;

	regmap_read(tfa9865->regmap, 0x06, &val);
	version = val >> 8;

	regmap_write(tfa9865->regmap, TFA9865_HIDDEN_CODE, TFA9865_HIDDEN_CODE_VAL);
	regmap_read(tfa9865->regmap, TFA9865_EFUSE_B, &val);
	key = val & 0xffff;
	key = key ^ TFA9865_KEY_UNLOCK;
	regmap_write(tfa9865->regmap, TFA9865_KEY1, key);
	regmap_write(tfa9865->regmap, TFA9865_KEY2, TFA9865_KEY_UNLOCK);

	if (product == 0x9864 && version == 0x30 && variant == N) {
		dev_info(tfa9865->dev, "Found TFA9864 N1\n");
		regmap_write(tfa9865->regmap, 0x00, 0xf241);
		regmap_write(tfa9865->regmap, 0x02, 0x0628);
		regmap_write(tfa9865->regmap, 0x50, 0xc000);
		regmap_write(tfa9865->regmap, 0x5a, 0x5f4c);
		regmap_write(tfa9865->regmap, 0x5b, 0x74e2);
		regmap_write(tfa9865->regmap, 0x5c, 0x302b);
		regmap_write(tfa9865->regmap, 0x5f, 0x00a0);
		regmap_write(tfa9865->regmap, 0x62, 0x05c6);
		regmap_write(tfa9865->regmap, 0x63, 0x80d4);
		regmap_write(tfa9865->regmap, 0x67, 0x0066);
		regmap_write(tfa9865->regmap, 0x68, 0x0820);
		regmap_write(tfa9865->regmap, 0x74, 0x60f0);
		regmap_write(tfa9865->regmap, 0x75, 0x0e00);
		regmap_write(tfa9865->regmap, 0x76, 0xbf00);
		regmap_write(tfa9865->regmap, 0x78, 0x0001);
		regmap_write(tfa9865->regmap, 0x7c, 0x10f2);
		regmap_write(tfa9865->regmap, 0xcf, 0x2cd7);
		regmap_write(tfa9865->regmap, 0xd7, 0x1000);
		regmap_write(tfa9865->regmap, 0xdd, 0x0036);
	} else if (product == 0x9864 && version == 0x30 && variant == MN) {
		dev_info(tfa9865->dev, "Found TFA9864 MN1\n");
		regmap_write(tfa9865->regmap, 0x00, 0xf241);
		regmap_write(tfa9865->regmap, 0x02, 0x0628);
		regmap_write(tfa9865->regmap, 0x50, 0xc000);
		regmap_write(tfa9865->regmap, 0x5a, 0x5f4c);
		regmap_write(tfa9865->regmap, 0x5b, 0x74e2);
		regmap_write(tfa9865->regmap, 0x5c, 0x302b);
		regmap_write(tfa9865->regmap, 0x5f, 0x00a0);
		regmap_write(tfa9865->regmap, 0x62, 0x05c6);
		regmap_write(tfa9865->regmap, 0x63, 0x80d4);
		regmap_write(tfa9865->regmap, 0x67, 0x0066);
		regmap_write(tfa9865->regmap, 0x68, 0x0820);
		regmap_write(tfa9865->regmap, 0x74, 0x60f0);
		regmap_write(tfa9865->regmap, 0x75, 0x0e00);
		regmap_write(tfa9865->regmap, 0x76, 0xbf00);
		regmap_write(tfa9865->regmap, 0x78, 0x0001);
		regmap_write(tfa9865->regmap, 0x7c, 0x10f2);
		regmap_write(tfa9865->regmap, 0xcf, 0x2cd7);
		regmap_write(tfa9865->regmap, 0xd7, 0x1000);
		regmap_write(tfa9865->regmap, 0xdd, 0x0036);
	} else if (product == 0x9865 && version == 0x30 && variant == MN) {
		dev_info(tfa9865->dev, "Found TFA9865 MN1\n");
		regmap_write(tfa9865->regmap, 0x00, 0xf241);
		regmap_write(tfa9865->regmap, 0x02, 0x0628);
		regmap_write(tfa9865->regmap, 0x50, 0xc000);
		regmap_write(tfa9865->regmap, 0x5a, 0x5f4c);
		regmap_write(tfa9865->regmap, 0x5b, 0x74e2);
		regmap_write(tfa9865->regmap, 0x5c, 0x302b);
		regmap_write(tfa9865->regmap, 0x5f, 0x00a0);
		regmap_write(tfa9865->regmap, 0x62, 0x05c6);
		regmap_write(tfa9865->regmap, 0x63, 0x80d4);
		regmap_write(tfa9865->regmap, 0x67, 0x0066);
		regmap_write(tfa9865->regmap, 0x68, 0x0820);
		regmap_write(tfa9865->regmap, 0x74, 0x60f0);
		regmap_write(tfa9865->regmap, 0x75, 0x0e00);
		regmap_write(tfa9865->regmap, 0x78, 0x0001);
		regmap_write(tfa9865->regmap, 0x7c, 0x10f2);
		regmap_write(tfa9865->regmap, 0xcf, 0x2cd7);
		regmap_write(tfa9865->regmap, 0xd7, 0x1000);
		regmap_write(tfa9865->regmap, 0xdd, 0x0036);
	} else {
		dev_warn(tfa9865->dev, "Unknown TFA986x version, no optimal settings applied\n");
	}


	regmap_update_bits(tfa9865->regmap, TFA9865_DEVICE_CONTROL, TFA9865_POWERDOWN_MASK,
			   TFA9865_POWERDOWN_MASK);
	regmap_read_poll_timeout(tfa9865->regmap, TFA9865_STATUS2, val,
				 !(val & TFA9865_MANSTATE_MASK), 1, 50);
	regmap_update_bits(tfa9865->regmap, TFA9865_CLK_CONTROL, TFA9865_MANAOOSC_MASK,
			   TFA9865_MANAOOSC_MASK);

	regmap_write(tfa9865->regmap, TFA9865_INT_ENABLE, 0x0000);
	regmap_write(tfa9865->regmap, TFA9865_INT_CLEAR, 0xffff);
	regmap_write(tfa9865->regmap, TFA9865_INT_ENABLE, 0x071e);

	return 0;
}

static int tfa9865_i2c_probe(struct i2c_client *i2c)
{
	const struct tfa9865_static_config *config;
	struct tfa9865_priv *tfa9865;
	u32 devindex = 0;
	int ret, i;

	tfa9865 = devm_kzalloc(&i2c->dev, sizeof(*tfa9865), GFP_KERNEL);
	if (!tfa9865)
		return -ENOMEM;

	i2c_set_clientdata(i2c, tfa9865);

	tfa9865->regmap = devm_regmap_init_i2c(i2c, &tfa9865_regmap);
	if (IS_ERR(tfa9865->regmap))
		return PTR_ERR(tfa9865->regmap);

	tfa9865->dev = &i2c->dev;

	ret = devm_regulator_get_enable(tfa9865->dev, "vddio");
	if (ret)
		return dev_err_probe(tfa9865->dev, ret, "Failed to get regulator\n");

	tfa9865->reset_gpio = devm_gpiod_get(tfa9865->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(tfa9865->reset_gpio))
		return dev_err_probe(tfa9865->dev, PTR_ERR(tfa9865->reset_gpio),
				     "Failed to get reset GPIO\n");

	gpiod_set_value_cansleep(tfa9865->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(tfa9865->reset_gpio, 0);
	usleep_range(1000, 2000);

	ret = tfa9865_init(tfa9865);
	if (ret)
		return dev_err_probe(tfa9865->dev, ret, "Failed to init TFA9865\n");

	config = device_get_match_data(tfa9865->dev);
	if (config) {
		of_property_read_u32(tfa9865->dev->of_node, "tfa,devindex", &devindex);
		if (devindex >= config->dev_count) {
			dev_err(tfa9865->dev,
				"devindex %u out of range (max %u)\n",
				devindex, config->dev_count - 1);
			return -EINVAL;
		}

		for (i = 0; i < config->dev_config[devindex].reg_count; i++) {
			const struct tfa9865_reg_data *reg_data =
				&config->dev_config[devindex].reg_data[i];
			ret = regmap_write(tfa9865->regmap, reg_data->reg, reg_data->val);
			if (ret) {
				dev_err(tfa9865->dev,
					"failed to write reg 0x%x val 0x%x ret=%d\n",
					reg_data->reg, reg_data->val, ret);
				return ret;
			}
		}
	}

	return devm_snd_soc_register_component(tfa9865->dev, &tfa9865_component,
					       &tfa9865_dai, 1);

	return 0;
}

static const struct i2c_device_id tfa9865_i2c_id[] = {
	{ "tfa9865" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa9865_i2c_id);

static const struct of_device_id tfa9865_of_match[] = {
	{
		.compatible = "goodix,tfa9865-iceland",
		.data = &tfa9865_iceland_config,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, tfa9865_of_match);

static struct i2c_driver tfa9865_i2c_driver = {
	.driver = {
		.name = "tfa9865",
		.of_match_table = tfa9865_of_match,
	},
	.probe = tfa9865_i2c_probe,
	.id_table = tfa9865_i2c_id,
};

module_i2c_driver(tfa9865_i2c_driver);

MODULE_DESCRIPTION("ASoC Goodix TFA9865 driver");
MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_LICENSE("GPL");
