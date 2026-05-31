#include <linux/spmi.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#include "core.h"
#include "spmi-regmap.h"

static const struct of_device_id tps6598x_spmi_of_match[] = {
	{ .compatible = "apple,sn201202x", &cd321x_data},
	{}
};

static int tps6598x_probe_spmi(struct spmi_device *device)
{
       const struct of_device_id *match;
       const struct tipd_data *data;
       struct tps6598x *tps;
       int ret;

       match = of_match_device(tps6598x_spmi_of_match, &device->dev);
       if (!match)
               return -EINVAL;
       data = match->data;

       tps = devm_kzalloc(&device->dev, data->tps_struct_size, GFP_KERNEL);
       if (!tps)
               return -ENOMEM;

       mutex_init(&tps->lock);
       tps->dev = &device->dev;
       tps->data = data;

       tps->irq = of_irq_get(device->dev.of_node, 0);
       if (tps->irq < 0)
               return tps->irq;

       tps->regmap = devm_regmap_init_spmi_tps6598x(device, &tps6598x_regmap_config);
       if (IS_ERR(tps->regmap))
               return PTR_ERR(tps->regmap);

       spmi_command_wakeup(device);
       mdelay(10); /* TODO: polling loop? */

       ret = tps6598x_probe(tps);

       if (ret == 0)
               spmi_device_set_drvdata(device, tps);

       return ret;
}

static void tps6598x_remove_spmi(struct spmi_device *device)
{
       struct tps6598x *tps = spmi_device_get_drvdata(device);
       tps6598x_remove(tps);
}

MODULE_DEVICE_TABLE(of, tps6598x_spmi_of_match);

static struct spmi_driver tps6598x_spmi_driver = {
        .driver = {
                .name = "tps6598x-spmi",
                .pm = &tps6598x_pm_ops,
                .of_match_table = tps6598x_spmi_of_match,
        },
        .probe = tps6598x_probe_spmi,
        .remove = tps6598x_remove_spmi,
};
module_spmi_driver(tps6598x_spmi_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI TPS6598x USB Power Delivery Controller Driver");
