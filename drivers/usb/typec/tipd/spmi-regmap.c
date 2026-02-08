// SPDX-License-Identifier: GPL-2.0
/*
 * Regmap for Apple SPMI bridge to TPS6598x
 *
 * Based on regmap-spmi.c:
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Based on regmap-i2c.c:
 * Copyright 2011 Wolfson Microelectronics plc
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 */

#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/module.h>
#include <linux/init.h>

#include "spmi-regmap.h"

static int regmap_spmi_tps6598x_select_reg(void *context, u8 reg)
{
	int err;
	u8 val;

	err = spmi_register_zero_write(context, reg);
	if (err)
		return err;

	while (1) {
		err = spmi_register_read(context, 0, &val);
		if (err)
			return err;
		if (val == (reg | 0x80))
			continue;
		if (val == reg)
			break;
		return -EIO;
	}

	return 0;
}

static int regmap_spmi_tps6598x_read(void *context,
				 const void *reg, size_t reg_size,
				 void *val, size_t val_size)
{
	u8 addr = *(u8 *)reg;
	int err;
	unsigned offset = 0x20;
	size_t len;

	BUG_ON(reg_size != 1);
	BUG_ON(val_size > 0x40);

	err = regmap_spmi_tps6598x_select_reg(context, addr);
	if (err)
		return err;

	while (val_size) {
		len = min_t(size_t, val_size, 16);
		err = spmi_ext_register_read(context, offset, val, val_size);
		if (err)
			return err;
		offset += len;
		val += len;
		val_size -= len;
	}

	return 0;
}

static int regmap_spmi_tps6598x_write(void *context, const void *data,
				  size_t count)
{
	BUG_ON(count < 1);

	u8 addr = *(u8 *)data;
	int err = 0;
	unsigned offset = 0xa0;
	size_t len;

	data += 1;
	count -= 1;

	BUG_ON(count > 0x40);

	err = regmap_spmi_tps6598x_select_reg(context, addr);
	if (err)
		return err;

	while (count) {
		len = min_t(size_t, count, 16);
		err = spmi_ext_register_write(context, offset, data, count);
		if (err)
			return err;
		offset += len;
		data += len;
		count -= len;
	}

	return err;
}

static const struct regmap_bus regmap_spmi_tps6598x = {
	.read				= regmap_spmi_tps6598x_read,
	.write				= regmap_spmi_tps6598x_write,
	.reg_format_endian_default	= REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default	= REGMAP_ENDIAN_NATIVE,
};

struct regmap *__devm_regmap_init_spmi_tps6598x(struct spmi_device *sdev,
					    const struct regmap_config *config,
					    struct lock_class_key *lock_key,
					    const char *lock_name)
{
	return __devm_regmap_init(&sdev->dev, &regmap_spmi_tps6598x, sdev, config,
				  lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__devm_regmap_init_spmi_tps6598x);

MODULE_DESCRIPTION("Regmap for Apple SPMI bridge to TPS6598x");
MODULE_LICENSE("GPL");
