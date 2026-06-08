// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for the Kinetic KTZ8869 in OnePlus Pad 4
 *
 * Copyright (c) 2026 map220v <map220v300@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define DEFAULT_BRIGHTNESS 2047
#define MAX_BRIGHTNESS 4095
#define REG_MAX 0x15

/* reg */
#define DEVICE_ID 0x00
#define BACKLIGHT_ENABLE 0x01
#define BL_CFG1 0x02
#define BL_CFG2 0x03
#define BL_BRT_LSB 0x04
#define BL_BRT_MSB 0x05
#define BL_EN 0x08
#define LCD_BIAS_CFG1 0x09
#define LCD_BIAS_CFG2 0x0A
#define LCD_BIAS_CFG3 0x0B
#define LCD_BOOST_CFG 0x0C
#define OUTP_CFG 0x0D
#define OUTN_CFG 0x0E
#define FLAG 0x0F
#define BL_OPTION1 0x10
#define BL_OPTION2 0x11
#define PWM2DIG_LSBs 0x12
#define PWM2DIG_MSBs 0x13
#define BL_DIMMING 0x14
#define PWM_RAMP_TIME 0x15

#define KTZ_MASTER 0x1
#define KTZ_SLAVE 0x2
#define KTZ_BOTH 0x3

struct ktz8869 {
	struct i2c_client *client;
	struct regmap *regmap;
	bool led_on;
	struct gpio_desc *enable_gpio;
};

static const struct regmap_config ktz8869_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

struct ktz8869 *ktz_b;

static int ktz8869_write(struct ktz8869 *ktz, unsigned int reg,
			 unsigned int val, u8 ic_mask)
{
	if (ic_mask & KTZ_MASTER)
		regmap_write(ktz->regmap, reg, val);

	if (ktz_b && (ic_mask & KTZ_SLAVE))
		regmap_write(ktz_b->regmap, reg, val);

	return 0;
}

static int ktz8869_backlight_update_status(struct backlight_device *backlight_dev)
{
	struct ktz8869 *ktz = bl_get_data(backlight_dev);
	unsigned int brightness = backlight_get_brightness(backlight_dev);

	/* Set brightness */
	ktz8869_write(ktz, BL_BRT_LSB, brightness & 0xF, KTZ_BOTH);
	ktz8869_write(ktz, BL_BRT_MSB, (brightness >> 4) & 0xFF, KTZ_BOTH);

	if (!ktz->led_on && brightness > 0) {
		ktz8869_write(ktz, BACKLIGHT_ENABLE, 0x01, KTZ_BOTH);
		ktz->led_on = true;
	} else if (brightness == 0) {
		ktz8869_write(ktz, BACKLIGHT_ENABLE, 0x00, KTZ_BOTH);
		ktz->led_on = false;
	}

	return 0;
}

static const struct backlight_ops ktz8869_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ktz8869_backlight_update_status,
};

static void ktz8869_init(struct ktz8869 *ktz)
{
	/* LCD Bias config */
	ktz8869_write(ktz, LCD_BOOST_CFG, 0x28, KTZ_MASTER);
	ktz8869_write(ktz, OUTP_CFG, 0x1E, KTZ_MASTER);
	ktz8869_write(ktz, OUTN_CFG, 0x1E, KTZ_MASTER);
	ktz8869_write(ktz, LCD_BIAS_CFG1, 0x9C, KTZ_MASTER);
	mdelay(5);
	ktz8869_write(ktz, LCD_BIAS_CFG1, 0x9E, KTZ_MASTER);

	/* Backlight config */
	ktz8869_write(ktz, BL_DIMMING, 0x48, KTZ_BOTH);
	ktz8869_write(ktz, BL_CFG1, 0xB3, KTZ_BOTH);
	ktz8869_write(ktz, BL_CFG2, 0xEB, KTZ_BOTH);
	ktz8869_write(ktz, BL_OPTION2, 0x76, KTZ_BOTH);
	ktz8869_write(ktz, PWM_RAMP_TIME, 0xF8, KTZ_BOTH);
	ktz8869_write(ktz, BL_EN, 0x77, KTZ_BOTH);
}

static int ktz8869_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	struct backlight_device *backlight_dev;
	struct backlight_properties props;
	struct ktz8869 *ktz;
	int ret = 0;

	if (id->driver_data == 1 && !ktz_b)
		return -EPROBE_DEFER;

	ktz = devm_kzalloc(&client->dev, sizeof(*ktz), GFP_KERNEL);
	if (!ktz)
		return -ENOMEM;

	ktz->client = client;
	ktz->regmap = devm_regmap_init_i2c(client, &ktz8869_regmap_config);
	if (IS_ERR(ktz->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(ktz->regmap), "failed to init regmap\n");

	ret = devm_regulator_get_enable(&client->dev, "vin");
	if (ret)
		return dev_err_probe(&client->dev, ret, "get regulator vin failed\n");
	ret = devm_regulator_get_enable(&client->dev, "vddio");
	if (ret)
		return dev_err_probe(&client->dev, ret, "get regulator vddio failed\n");

	if (id->driver_data == 2) {
		ktz_b = ktz;
		goto end;
	}

	ktz->enable_gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ktz->enable_gpio))
		return PTR_ERR(ktz->enable_gpio);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;
	props.brightness = DEFAULT_BRIGHTNESS;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	backlight_dev = devm_backlight_device_register(&client->dev, "ktz8869-backlight",
					&client->dev, ktz, &ktz8869_backlight_ops, &props);
	if (IS_ERR(backlight_dev))
		return dev_err_probe(&client->dev, PTR_ERR(backlight_dev),
				"failed to register backlight device\n");

	ktz8869_init(ktz);

	i2c_set_clientdata(client, backlight_dev);
	backlight_update_status(backlight_dev);

end:
	return 0;
}

static void ktz8869_remove(struct i2c_client *client)
{
	struct backlight_device *backlight_dev = i2c_get_clientdata(client);
	backlight_dev->props.brightness = 0;
	backlight_update_status(backlight_dev);
}

static const struct i2c_device_id ktz8869_ids[] = {
	{ "ktz8869", 0},
	{ "ktz8869a", 1 },
	{ "ktz8869b", 2 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ktz8869_ids);

static const struct of_device_id ktz8869_match_table[] = {
	{ .compatible = "kinetic,ktz8869", },
	{ .compatible = "kinetic,ktz8869a", },
	{ .compatible = "kinetic,ktz8869b", },
	{},
};
MODULE_DEVICE_TABLE(of, ktz8869_match_table);

static struct i2c_driver ktz8869_driver = {
	.driver = {
		.name = "ktz8869",
		.of_match_table = ktz8869_match_table,
	},
	.probe = ktz8869_probe,
	.remove = ktz8869_remove,
	.id_table = ktz8869_ids,
};

module_i2c_driver(ktz8869_driver);

MODULE_DESCRIPTION("Kinetic KTZ8869 Backlight Driver");
MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_LICENSE("GPL");
