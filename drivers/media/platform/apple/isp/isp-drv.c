// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Image Signal Processor driver
 *
 * Copyright (C) 2023 The Asahi Linux Contributors
 */

#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>

#include "isp-cam.h"
#include "isp-fw.h"
#include "isp-iommu.h"
#include "isp-v4l2.h"

static void apple_isp_detach_genpd(struct apple_isp *isp)
{
	if (isp->pd_count <= 1)
		return;

	for (int i = isp->pd_count - 1; i >= 0; i--) {
		if (isp->pd_link[i])
			device_link_del(isp->pd_link[i]);
		if (!IS_ERR_OR_NULL(isp->pd_dev[i]))
			dev_pm_domain_detach(isp->pd_dev[i], true);
	}
}

static int apple_isp_attach_genpd(struct apple_isp *isp)
{
	struct device *dev = isp->dev;

	isp->pd_count = of_count_phandle_with_args(
		dev->of_node, "power-domains", "#power-domain-cells");
	if (isp->pd_count <= 1)
		return 0;

	isp->pd_dev = devm_kcalloc(dev, isp->pd_count, sizeof(*isp->pd_dev),
				   GFP_KERNEL);
	if (!isp->pd_dev)
		return -ENOMEM;

	isp->pd_link = devm_kcalloc(dev, isp->pd_count, sizeof(*isp->pd_link),
				    GFP_KERNEL);
	if (!isp->pd_link)
		return -ENOMEM;

	for (int i = 0; i < isp->pd_count; i++) {
		int flags = DL_FLAG_STATELESS;

		/* Primary power domain uses RPM integration */
		if (i == 0)
			flags |= DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE;

		isp->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(isp->pd_dev[i])) {
			apple_isp_detach_genpd(isp);
			return PTR_ERR(isp->pd_dev[i]);
		}

		isp->pd_link[i] =
			device_link_add(dev, isp->pd_dev[i], flags);

		if (!isp->pd_link[i]) {
			apple_isp_detach_genpd(isp);
			return -EINVAL;
		}
	}

	return 0;
}

static int apple_isp_init_iommu(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	dma_addr_t heap_base = 0;
	size_t heap_size;
	LIST_HEAD(resv_regions);
	struct iommu_resv_region *region;

	isp->domain = iommu_get_domain_for_dev(isp->dev);
	if (!isp->domain)
		return -ENODEV;
	isp->shift = __ffs(isp->domain->pgsize_bitmap);

	iommu_get_resv_regions(dev, &resv_regions);

	list_for_each_entry(region, &resv_regions, list) {
		if (region->dva > heap_base) {
			heap_base = region->dva;
			heap_size = region->length;
		}
	}
	iommu_put_resv_regions(dev, &resv_regions);

	isp->fw.heap_top = heap_base + heap_size;

	return 0;
}

static struct isp_preset isp_presets_imx248[] = {
	{0, {1296, 736}, {1280, 720}, {8, 8}, {1280, 720}},
	{0, {1296, 736}, {960, 720}, {168, 8}, {960, 720}},
	{0, {1296, 736}, {960, 540}, {8, 8}, {1280, 720}},
	{0, {1296, 736}, {640, 480}, {168, 8}, {960, 720}},
	{0, {1296, 736}, {640, 360}, {8, 8}, {1280, 720}},
	{0, {1296, 736}, {320, 180}, {8, 8}, {1280, 720}},
};

static struct isp_preset isp_presets_imx364[] = {
	{0, {1920, 1080}, {1920, 1080}, {0, 0}, {1920, 1080}},
	{0, {1920, 1080}, {1440, 1080}, {240, 0}, {1440, 1080}},
	{0, {1920, 1080}, {1280, 720}, {0, 0}, {1920, 1080}},
	{0, {1920, 1080}, {960, 720}, {240, 0}, {1440, 1080}},
	{0, {1920, 1080}, {960, 540}, {0, 0}, {1920, 1080}},
	{0, {1920, 1080}, {640, 480}, {240, 0}, {1440, 1080}},
	{0, {1920, 1080}, {640, 360}, {0, 0}, {1920, 1080}},
	{0, {1920, 1080}, {320, 180}, {0, 0}, {1920, 1080}},
};

static struct isp_preset isp_presets_imx558[] = {
	{1, {1920, 1080}, {1920, 1080}, {0, 0}, {1920, 1080}},
	{2, {1080, 1920}, {1080, 1920}, {0, 0}, {1080, 1920}},
	{3, {1760, 1328}, {1760, 1328}, {0, 0}, {1760, 1328}},
	{4, {1328, 1760}, {1328, 1760}, {0, 0}, {1328, 1760}},
	{5, {1152, 1152}, {1152, 1152}, {0, 0}, {1152, 1152}},
	{1, {1920, 1080}, {1280, 720}, {0, 0}, {1920, 1080}},
	{2, {1080, 1920}, {720, 1280}, {0, 0}, {1080, 1920}},
	{3, {1760, 1328}, {1280, 960}, {0, 4}, {1760, 1320}},
	{4, {1328, 1760}, {960, 1280}, {4, 0}, {1320, 1760}},
	{3, {1760, 1328}, {640, 480}, {0, 4}, {1760, 1320}},
	{4, {1328, 1760}, {480, 640}, {4, 0}, {1320, 1760}},
};

static struct isp_preset isp_presets_imx558_cfg0[] = {
	{0, {1920, 1920}, {1920, 1080}, {0, 420}, {1920, 1080}},
	{0, {1920, 1920}, {1080, 1920}, {420, 0}, {1080, 1920}},
	{0, {1920, 1920}, {1920, 1440}, {0, 240}, {1920, 1440}},
	{0, {1920, 1920}, {1440, 1920}, {240, 0}, {1440, 1920}},
	{0, {1920, 1920}, {1280, 720}, {0, 420}, {1920, 1080}},
	{0, {1920, 1920}, {720, 1280}, {420, 0}, {1080, 1920}},
	{0, {1920, 1920}, {1280, 960}, {0, 240}, {1920, 1440}},
	{0, {1920, 1920}, {960, 1280}, {240, 0}, {1440, 1920}},
	{0, {1920, 1920}, {640, 480}, {0, 240}, {1920, 1440}},
	{0, {1920, 1920}, {480, 640}, {240, 0}, {1440, 1920}},
};

static int apple_isp_init_presets(struct apple_isp *isp)
{
	struct isp_format *fmt = isp_get_format(isp, isp->current_ch);

	switch (fmt->version) {
	case 0x248:
		isp->num_presets = ARRAY_SIZE(isp_presets_imx248);
		isp->presets = isp_presets_imx248;
		break;
	case 0x364:
		isp->num_presets = ARRAY_SIZE(isp_presets_imx364);
		isp->presets = isp_presets_imx364;
		break;
	case 0x558:
		if (isp->hw->mlvnr) {
			isp->num_presets = ARRAY_SIZE(isp_presets_imx558_cfg0);
			isp->presets = isp_presets_imx558_cfg0;
		} else {
			isp->num_presets = ARRAY_SIZE(isp_presets_imx558);
			isp->presets = isp_presets_imx558;
		}
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

static const char *isp_fw2str(enum isp_firmware_version version)
{
	switch (version) {
	case ISP_FIRMWARE_V_12_3:
		return "12.3";
	case ISP_FIRMWARE_V_12_4:
		return "12.4";
	case ISP_FIRMWARE_V_13_5:
		return "13.5";
	default:
		return "unknown";
	}
}

#define ISP_FW_VERSION_MIN_LEN	3
#define ISP_FW_VERSION_MAX_LEN	5

static enum isp_firmware_version isp_read_fw_version(struct device *dev,
						     const char *name)
{
	u32 ver[ISP_FW_VERSION_MAX_LEN];
	int len = of_property_read_variable_u32_array(dev->of_node, name, ver,
						      ISP_FW_VERSION_MIN_LEN,
						      ISP_FW_VERSION_MAX_LEN);

	switch (len) {
	case 3:
		if (ver[0] == 12 && ver[1] == 3 && ver[2] <= 1)
			return ISP_FIRMWARE_V_12_3;
		else if (ver[0] == 12 && ver[1] == 4 && ver[2] == 0)
			return ISP_FIRMWARE_V_12_4;
		else if (ver[0] == 13 && ver[1] == 5 && ver[2] == 0)
			return ISP_FIRMWARE_V_13_5;

		dev_warn(dev, "unknown %s: %d.%d.%d\n", name, ver[0], ver[1], ver[2]);
		break;
	case 4:
		dev_warn(dev, "unknown %s: %d.%d.%d.%d\n", name, ver[0], ver[1],
			 ver[2], ver[3]);
		break;
	case 5:
		dev_warn(dev, "unknown %s: %d.%d.%d.%d.%d\n", name, ver[0],
			 ver[1], ver[2], ver[3], ver[4]);
		break;
	default:
		dev_warn(dev, "could not parse %s: %d\n", name, len);
		break;
	}

	return ISP_FIRMWARE_V_UNKNOWN;
}

static enum isp_firmware_version isp_check_firmware_version(struct device *dev)
{
	enum isp_firmware_version version, compat;

	/* firmware version is just informative */
	version = isp_read_fw_version(dev, "apple,firmware-version");
	compat = isp_read_fw_version(dev, "apple,firmware-compat");

	dev_info(dev, "ISP firmware-compat: %s (FW: %s)\n", isp_fw2str(compat),
		 isp_fw2str(version));

	return compat;
}

static int apple_isp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_isp *isp;
	int err;

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (err)
		return err;

	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;
	isp->hw = of_device_get_match_data(dev);
	platform_set_drvdata(pdev, isp);
	dev_set_drvdata(dev, isp);

	/* Differences between firmware versions are rather minor so try to work
	 * with unknown firmware.
	 */
	isp->fw_compat = isp_check_firmware_version(dev);

	err = of_property_read_u32(dev->of_node, "apple,platform-id",
				   &isp->platform_id);
	if (err) {
		dev_err(dev, "failed to get 'apple,platform-id' property: %d\n",
			err);
		return err;
	}

	err = of_property_read_u32(dev->of_node, "apple,temporal-filter",
				   &isp->temporal_filter);
	if (err)
		isp->temporal_filter = 0;

	err = apple_isp_attach_genpd(isp);
	if (err) {
		dev_err(dev, "failed to attach power domains\n");
		return err;
	}

	isp->coproc = devm_platform_ioremap_resource_byname(pdev, "coproc");
	if (IS_ERR(isp->coproc)) {
		err = PTR_ERR(isp->coproc);
		goto detach_genpd;
	}

	isp->mbox = devm_platform_ioremap_resource_byname(pdev, "mbox");
	if (IS_ERR(isp->mbox)) {
		err = PTR_ERR(isp->mbox);
		goto detach_genpd;
	}

	isp->gpio = devm_platform_ioremap_resource_byname(pdev, "gpio");
	if (IS_ERR(isp->gpio)) {
		err = PTR_ERR(isp->gpio);
		goto detach_genpd;
	}

	isp->mbox2 = devm_platform_ioremap_resource_byname(pdev, "mbox2");
	if (IS_ERR(isp->mbox2)) {
		err = PTR_ERR(isp->mbox2);
		goto detach_genpd;
	}

	isp->irq = platform_get_irq(pdev, 0);
	if (isp->irq < 0) {
		err = isp->irq;
		goto detach_genpd;
	}
	if (!isp->irq) {
		err = -ENODEV;
		goto detach_genpd;
	}

	mutex_init(&isp->video_lock);
	spin_lock_init(&isp->buf_lock);
	init_waitqueue_head(&isp->wait);
	INIT_LIST_HEAD(&isp->gc);
	INIT_LIST_HEAD(&isp->bufs_pending);
	INIT_LIST_HEAD(&isp->bufs_submitted);
	isp->wq = alloc_workqueue("apple-isp-wq", WQ_UNBOUND, 0);
	if (!isp->wq) {
		dev_err(dev, "failed to create workqueue\n");
		err = -ENOMEM;
		goto detach_genpd;
	}

	err = apple_isp_init_iommu(isp);
	if (err) {
		dev_err(dev, "failed to init iommu: %d\n", err);
		goto destroy_wq;
	}

	err = apple_isp_alloc_firmware_surface(isp);
	if (err) {
		dev_err(dev, "failed to alloc firmware surface: %d\n", err);
		goto destroy_wq;
	}

	pm_runtime_enable(dev);

	err = apple_isp_detect_camera(isp);
	if (err) {
		dev_err(dev, "failed to detect camera: %d\n", err);
		goto free_surface;
	}

	err = apple_isp_init_presets(isp);
	if (err) {
		dev_err(dev, "failed to initialize presets\n");
		return err;
	}

	err = apple_isp_setup_video(isp);
	if (err) {
		dev_err(dev, "failed to register video device: %d\n", err);
		goto free_surface;
	}

	return 0;

free_surface:
	pm_runtime_disable(dev);
	apple_isp_free_firmware_surface(isp);
destroy_wq:
	destroy_workqueue(isp->wq);
detach_genpd:
	apple_isp_detach_genpd(isp);
	return err;
}

static void apple_isp_remove(struct platform_device *pdev)
{
	struct apple_isp *isp = platform_get_drvdata(pdev);

	apple_isp_remove_video(isp);
	pm_runtime_disable(isp->dev);
	apple_isp_free_firmware_surface(isp);
	destroy_workqueue(isp->wq);
	apple_isp_detach_genpd(isp);
}

static const struct apple_isp_hw apple_isp_hw_t8103 = {
	.gen = ISP_GEN_T8103,
	.pmu_base = 0x23b704000,

	.dsid_count = 4,
	.dsid_clr_base0 = 0x200014000,
	.dsid_clr_base1 = 0x200054000,
	.dsid_clr_base2 = 0x200094000,
	.dsid_clr_base3 = 0x2000d4000,
	.dsid_clr_range0 = 0x1000,
	.dsid_clr_range1 = 0x1000,
	.dsid_clr_range2 = 0x1000,
	.dsid_clr_range3 = 0x1000,

	.clock_scratch = 0x23b738010,
	.clock_base = 0x23bc3c000,
	.clock_bit = 0x1,
	.clock_size = 0x4,
	.bandwidth_scratch = 0x23b73800c,
	.bandwidth_base = 0x23bc3c000,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x4,

	.scl1 = false,
	.lpdp = false,
	.mlvnr = false,
	.meta_size = ISP_META_SIZE_T8103,
};

static const struct apple_isp_hw apple_isp_hw_t6000 = {
	.gen = ISP_GEN_T8103,
	.pmu_base = 0x28e584000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200014000,
	.dsid_clr_base1 = 0x200054000,
	.dsid_clr_base2 = 0x200094000,
	.dsid_clr_base3 = 0x2000d4000,
	.dsid_clr_range0 = 0x1000,
	.dsid_clr_range1 = 0x1000,
	.dsid_clr_range2 = 0x1000,
	.dsid_clr_range3 = 0x1000,

	.clock_scratch = 0x28e3d0868,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x28e3d0980,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,

	.scl1 = false,
	.lpdp = false,
	.mlvnr = false,
	.meta_size = ISP_META_SIZE_T8103,
};

static const struct apple_isp_hw apple_isp_hw_t8112 = {
	.gen = ISP_GEN_T8112,
	.pmu_base = 0x23b704000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x23b3d0560,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x23b3d05d0,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,

	.scl1 = false,
	.lpdp = false,
	.mlvnr = true,
	.meta_size = ISP_META_SIZE_T8112,
};

static const struct apple_isp_hw apple_isp_hw_t6020 = {
	.gen = ISP_GEN_T8112,
	.pmu_base = 0x290284000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x28e3d10a8,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x28e3d1200,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,

	.scl1 = true,
	.lpdp = true,
	.mlvnr = true,
	.meta_size = ISP_META_SIZE_T8112,
};

static const struct of_device_id apple_isp_of_match[] = {
	{ .compatible = "apple,t8103-isp", .data = &apple_isp_hw_t8103 },
	{ .compatible = "apple,t8112-isp", .data = &apple_isp_hw_t8112 },
	{ .compatible = "apple,t6000-isp", .data = &apple_isp_hw_t6000 },
	{ .compatible = "apple,t6020-isp", .data = &apple_isp_hw_t6020 },
	{},
};
MODULE_DEVICE_TABLE(of, apple_isp_of_match);

static __maybe_unused int apple_isp_runtime_suspend(struct device *dev)
{
	/* RPM sleep is called when the V4L2 file handle is closed */
	return 0;
}

static __maybe_unused int apple_isp_runtime_resume(struct device *dev)
{
	return 0;
}

static __maybe_unused int apple_isp_suspend(struct device *dev)
{
	struct apple_isp *isp = dev_get_drvdata(dev);

	/* We must restore V4L2 context on system resume. If we were streaming
	 * before, we (essentially) stop streaming and start streaming again.
	 */
	apple_isp_video_suspend(isp);

	return 0;
}

static __maybe_unused int apple_isp_resume(struct device *dev)
{
	struct apple_isp *isp = dev_get_drvdata(dev);

	apple_isp_video_resume(isp);

	return 0;
}

static const struct dev_pm_ops apple_isp_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(apple_isp_suspend, apple_isp_resume)
	RUNTIME_PM_OPS(apple_isp_runtime_suspend, apple_isp_runtime_resume, NULL)
};

static struct platform_driver apple_isp_driver = {
	.driver	= {
		.name		= "apple-isp",
		.of_match_table	= apple_isp_of_match,
		.pm		= pm_ptr(&apple_isp_pm_ops),
	},
	.probe	= apple_isp_probe,
	.remove	= apple_isp_remove,
};
module_platform_driver(apple_isp_driver);

MODULE_AUTHOR("Eileen Yoon <eyn@gmx.com>");
MODULE_DESCRIPTION("Apple ISP driver");
MODULE_LICENSE("GPL");
