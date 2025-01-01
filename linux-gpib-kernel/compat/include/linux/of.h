#ifndef __COMPAT_LINUX_OF_H
#define __COMPAT_LINUX_OF_H

#include <linux/version.h>

#include_next <linux/of.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0)
#ifndef CONFIG_OF
static inline struct device_node *of_find_node_by_path(const char *path)
{
       return NULL;
}

#if (HAVE_DEV_OF_NODE != 1)
static inline struct device_node *dev_of_node(struct device *dev)
{
	return NULL;
}
#endif
#endif
#endif

#endif

