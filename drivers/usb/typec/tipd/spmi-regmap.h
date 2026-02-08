// SPDX-License-Identifier: GPL-2.0
/*
 * Regmap for Apple SPMI bridge to TPS6598x
 */

#include <linux/regmap.h>
#include <linux/spmi.h>

struct regmap *__devm_regmap_init_spmi_tps6598x(struct spmi_device *dev,
					    const struct regmap_config *config,
					    struct lock_class_key *lock_key,
					    const char *lock_name);
#define devm_regmap_init_spmi_tps6598x(dev, config)				\
	__regmap_lockdep_wrapper(__devm_regmap_init_spmi_tps6598x, #config,	\
				dev, config)
