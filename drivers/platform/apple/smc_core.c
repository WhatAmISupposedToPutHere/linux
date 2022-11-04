// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC core framework
 * Copyright The Asahi Linux Contributors
 */

#include <linux/device.h>
#include <linux/mfd/core.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include "smc.h"

struct apple_smc;

struct smc_key_kobj {
	struct kobject kobj;
	struct apple_smc *smc;
	smc_key key;
};

struct apple_smc {
	struct device *dev;

	void *be_cookie;
	const struct apple_smc_backend_ops *be;

	struct mutex mutex;

	u32 key_count;
	smc_key first_key;
	smc_key last_key;

	struct blocking_notifier_head event_handlers;
	struct kset *key_kset;
	struct smc_key_kobj **key_kobjects;
};

static const struct mfd_cell apple_smc_devs[] = {
	{
		.name = "macsmc-gpio",
	},
	{
		.name = "macsmc-hid",
	},
	{
		.name = "macsmc-power",
	},
	{
		.name = "macsmc-reboot",
	},
	{
		.name = "macsmc-rtc",
	},
};

int apple_smc_read(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->read_key(smc->be_cookie, key, buf, size);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_read);

int apple_smc_write(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->write_key(smc->be_cookie, key, buf, size);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_write);

int apple_smc_write_atomic(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	int ret;

	/*
	 * Will fail if SMC is busy. This is only used by SMC reboot/poweroff
	 * final calls, so it doesn't really matter at that point.
	 */
	if (!mutex_trylock(&smc->mutex))
		return -EBUSY;

	ret = smc->be->write_key_atomic(smc->be_cookie, key, buf, size);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_write_atomic);

int apple_smc_rw(struct apple_smc *smc, smc_key key, void *wbuf, size_t wsize,
		 void *rbuf, size_t rsize)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->rw_key(smc->be_cookie, key, wbuf, wsize, rbuf, rsize);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_rw);

int apple_smc_get_key_by_index(struct apple_smc *smc, int index, smc_key *key)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->get_key_by_index(smc->be_cookie, index, key);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_get_key_by_index);

int apple_smc_get_key_info(struct apple_smc *smc, smc_key key, struct apple_smc_key_info *info)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->get_key_info(smc->be_cookie, key, info);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_get_key_info);

int apple_smc_find_first_key_index(struct apple_smc *smc, smc_key key)
{
	int start = 0, count = smc->key_count;
	int ret;

	if (key <= smc->first_key)
		return 0;
	if (key > smc->last_key)
		return smc->key_count;

	while (count > 1) {
		int pivot = start + ((count - 1) >> 1);
		smc_key pkey;

		ret = apple_smc_get_key_by_index(smc, pivot, &pkey);
		if (ret < 0)
			return ret;

		if (pkey == key)
			return pivot;

		pivot++;

		if (pkey < key) {
			count -= pivot - start;
			start = pivot;
		} else {
			count = pivot - start;
		}
	}

	return start;
}
EXPORT_SYMBOL(apple_smc_find_first_key_index);

int apple_smc_get_key_count(struct apple_smc *smc)
{
	return smc->key_count;
}
EXPORT_SYMBOL(apple_smc_get_key_count);

void apple_smc_event_received(struct apple_smc *smc, uint32_t event)
{
	dev_dbg(smc->dev, "Event: 0x%08x\n", event);
	blocking_notifier_call_chain(&smc->event_handlers, event, NULL);
}
EXPORT_SYMBOL(apple_smc_event_received);

int apple_smc_register_notifier(struct apple_smc *smc, struct notifier_block *n)
{
	return blocking_notifier_chain_register(&smc->event_handlers, n);
}
EXPORT_SYMBOL(apple_smc_register_notifier);

int apple_smc_unregister_notifier(struct apple_smc *smc, struct notifier_block *n)
{
	return blocking_notifier_chain_unregister(&smc->event_handlers, n);
}
EXPORT_SYMBOL(apple_smc_unregister_notifier);

void *apple_smc_get_cookie(struct apple_smc *smc)
{
	return smc->be_cookie;
}
EXPORT_SYMBOL(apple_smc_get_cookie);

struct smc_key_attribute {
	struct attribute attr;
	ssize_t (*show)(struct smc_key_kobj *kobj, struct smc_key_attribute *attr, char *buf);
	ssize_t (*store)(struct smc_key_kobj *kobj, struct smc_key_attribute *attr, const char *buf, size_t count);
};

static ssize_t smc_key_attr_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct smc_key_attribute *attribute;
	struct smc_key_kobj *key;

	attribute = container_of(attr, struct smc_key_attribute, attr);
	key = container_of(kobj, struct smc_key_kobj, kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(key, attribute, buf);
}

static ssize_t smc_key_attr_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	struct smc_key_attribute *attribute;
	struct smc_key_kobj *key;

	attribute = container_of(attr, struct smc_key_attribute, attr);
	key = container_of(kobj, struct smc_key_kobj, kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->store(key, attribute, buf, len);
}

static struct sysfs_ops smc_key_ops = {
	.show = smc_key_attr_show,
	.store = smc_key_attr_store,
};

static void smc_key_release(struct kobject *kobj)
{
	struct smc_key_kobj *obj;

	obj = container_of(kobj, struct smc_key_kobj, kobj);
	kfree(obj);
}

static ssize_t value_attr_show(struct smc_key_kobj *kobj, struct smc_key_attribute *attr, char *buf)
{
	int ret;
	struct apple_smc_key_info info;
	ret = apple_smc_get_key_info(kobj->smc, kobj->key, &info);
	if (ret < 0)
		return ret;

	ret = apple_smc_read(kobj->smc, kobj->key, buf, info.size); //u8 can't be larger than PAGE_SIZE
	if (ret < 0)
		return ret;

	switch (info.type_code) {
	case 'ui64':
	case 'ioft':
	    return sysfs_emit(buf, "%llu\n", *(u64*)buf);
	case 'ui32':
	    return sysfs_emit(buf, "%u\n", *(u32*)buf);
	case 'ui16':
	    return sysfs_emit(buf, "%u\n", *(u16*)buf);
	case 'ui8 ':
	case 'flag':
	    return sysfs_emit(buf, "%u\n", *(u8*)buf);
	case 'si64':
	    return sysfs_emit(buf, "%lld\n", *(s64*)buf);
	case 'si32':
	    return sysfs_emit(buf, "%d\n", *(s32*)buf);
	case 'si16':
	    return sysfs_emit(buf, "%d\n", *(s16*)buf);
	case 'si8 ':
	    return sysfs_emit(buf, "%d\n", *(s8*)buf);
	default:
	    return info.size;
	}
}


static ssize_t type_attr_show(struct smc_key_kobj *kobj, struct smc_key_attribute *attr, char *buf)
{
	int ret;
	struct apple_smc_key_info info;
	ret = apple_smc_get_key_info(kobj->smc, kobj->key, &info);
	if (ret < 0)
		return ret;

	info.type_code = swab32(info.type_code);
	return sysfs_emit(buf, "%.4s\n", (char *)&info.type_code);
}

static struct smc_key_attribute value_attribute =
	__ATTR(value, 0444, value_attr_show, NULL);

static struct smc_key_attribute type_attribute =
	__ATTR(type, 0444, type_attr_show, NULL);

static struct attribute *smc_key_default_attrs[] = {
	&type_attribute.attr,
	&value_attribute.attr,
	NULL,
};
ATTRIBUTE_GROUPS(smc_key_default);

static struct kobj_type smc_key_ktype = {
	.sysfs_ops = &smc_key_ops,
	.release = smc_key_release,
	.default_groups = smc_key_default_groups,
};

struct apple_smc *apple_smc_probe(struct device *dev, const struct apple_smc_backend_ops *ops, void *cookie)
{
	struct apple_smc *smc;
	struct smc_key_kobj *key_obj;
	smc_key key, swapped_key;
	u32 count, i;
	int ret;

	smc = devm_kzalloc(dev, sizeof(*smc), GFP_KERNEL);
	if (!smc)
		return ERR_PTR(-ENOMEM);

	smc->dev = dev;
	smc->be_cookie = cookie;
	smc->be = ops;
	mutex_init(&smc->mutex);
	BLOCKING_INIT_NOTIFIER_HEAD(&smc->event_handlers);

	ret = apple_smc_read_u32(smc, SMC_KEY(#KEY), &count);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret, "Failed to get key count"));
	smc->key_count = be32_to_cpu(count);

	ret = apple_smc_get_key_by_index(smc, 0, &smc->first_key);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret, "Failed to get first key"));

	ret = apple_smc_get_key_by_index(smc, smc->key_count - 1, &smc->last_key);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret, "Failed to get last key"));

	/* Enable notifications */
	apple_smc_write_flag(smc, SMC_KEY(NTAP), 1);

	dev_info(dev, "Initialized (%d keys %p4ch..%p4ch)\n",
		 smc->key_count, &smc->first_key, &smc->last_key);

	dev_set_drvdata(dev, smc);

	ret = mfd_add_devices(dev, -1, apple_smc_devs, ARRAY_SIZE(apple_smc_devs), NULL, 0, NULL);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret, "Subdevice initialization failed"));

	smc->key_kobjects = devm_kzalloc(dev, sizeof(struct smc_key_kobj *) * smc->key_count, GFP_KERNEL);
	if (!smc->key_kobjects)
		return ERR_PTR(-ENOMEM);

	smc->key_kset = kset_create_and_add("keys", NULL, &dev->kobj);
	if (!smc->key_kset)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < smc->key_count; i++) {
		key_obj = kzalloc(sizeof(*key_obj), GFP_KERNEL);
		if (!key_obj) {
			ret = -ENOMEM;
			goto put_kobjs;
		}

		ret = apple_smc_get_key_by_index(smc, i, &key);
		if (ret) {
			kfree(key_obj);
			ret = dev_err_probe(dev, ret, "Failed to get key");
			goto put_kobjs;
		}
		key_obj->key = key;
		key_obj->smc = smc;

		key_obj->kobj.kset = smc->key_kset;
		smc->key_kobjects[i] = key_obj;
		swapped_key = swab32(key);
		ret = kobject_init_and_add(&key_obj->kobj, &smc_key_ktype, NULL, "%.4s", (char *)&swapped_key);
		if (ret)
			goto put_kobjs;

		kobject_uevent(&key_obj->kobj, KOBJ_ADD);
	}
	return smc;

put_kobjs:
	for (i = 0; i < smc->key_count; i++) {
		if (smc->key_kobjects[i])
			kobject_put(&smc->key_kobjects[i]->kobj);
	}
	return ERR_PTR(ret);

}
EXPORT_SYMBOL(apple_smc_probe);

int apple_smc_remove(struct apple_smc *smc)
{
	mfd_remove_devices(smc->dev);

	/* Disable notifications */
	apple_smc_write_flag(smc, SMC_KEY(NTAP), 1);
	kset_unregister(smc->key_kset);

	return 0;
}
EXPORT_SYMBOL(apple_smc_remove);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC core");
