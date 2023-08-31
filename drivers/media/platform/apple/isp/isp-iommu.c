// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#include "isp-iommu.h"

int apple_isp_surf_vmap(struct apple_isp *isp, struct isp_surf *surf)
{
	surf->virt = dma_vmap_noncontiguous(isp->dev, surf->size, surf->sgt);
	if (surf->virt == NULL) {
		dev_err(isp->dev, "failed to vmap size 0x%llx\n", surf->size);
		return -EINVAL;
	}

	return 0;
}

static void isp_surf_vunmap(struct apple_isp *isp, struct isp_surf *surf)
{
	if (surf->virt)
		dma_vunmap_noncontiguous(isp->dev, surf->virt);
	surf->virt = NULL;
}

static void isp_surf_iommu_unmap(struct apple_isp *isp, struct isp_surf *surf)
{
	dma_free_noncontiguous(isp->dev, surf->size, surf->sgt, DMA_BIDIRECTIONAL);
}

static int isp_surf_iommu_map(struct apple_isp *isp, struct isp_surf *surf)
{
	surf->sgt = dma_alloc_noncontiguous(isp->dev, surf->size, DMA_BIDIRECTIONAL,
					    GFP_KERNEL, 0);
	if (!surf->sgt) {
		dev_err(isp->dev, "failed to alloc io memory\n");
		return -ENXIO;
	}
	surf->iova = surf->sgt->sgl->dma_address;
	return 0;
}

static void __isp_surf_init(struct apple_isp *isp, struct isp_surf *surf,
			    u64 size, bool gc)
{
	surf->virt = NULL;
	surf->size = ALIGN(size, 1UL << isp->shift);
	surf->gc = gc;
}

struct isp_surf *__apple_isp_alloc_surface(struct apple_isp *isp, u64 size, bool gc)
{
	struct isp_surf *surf;
	int err;

	surf = kzalloc(sizeof(struct isp_surf), GFP_KERNEL);
	if (!surf)
		return NULL;

	__isp_surf_init(isp, surf, size, gc);

	err = isp_surf_iommu_map(isp, surf);
	if (err) {
		dev_err(isp->dev,
			"failed to iommu_map size 0x%llx to iova 0x%llx\n",
			surf->size, surf->iova);
		goto free_surf;
	}

	refcount_set(&surf->refcount, 1);
	if (surf->gc)
		list_add_tail(&surf->head, &isp->gc);

	return surf;

free_surf:
	kfree(surf);
	return NULL;
}

struct isp_surf *apple_isp_alloc_surface_vmap(struct apple_isp *isp, u64 size)
{
	struct isp_surf *surf;
	int err;

	surf = __apple_isp_alloc_surface(isp, size, false);
	if (!surf)
		return NULL;

	err = apple_isp_surf_vmap(isp, surf);
	if (err) {
		dev_err(isp->dev, "failed to vmap iova 0x%llx - 0x%llx\n",
			surf->iova, surf->iova + surf->size);
		apple_isp_free_surface(isp, surf);
		return NULL;
	}

	return surf;
}

void apple_isp_free_surface(struct apple_isp *isp, struct isp_surf *surf)
{
	if (refcount_dec_and_test(&surf->refcount)) {
		isp_surf_vunmap(isp, surf);
		isp_surf_iommu_unmap(isp, surf);
		if (surf->gc)
			list_del(&surf->head);
		kfree(surf);
	}
}
