/*
    linux/uaccess.h compatibility header

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __COMPAT_LINUX_UACCESS_H
#define __COMPAT_LINUX_UACCESS_H

#include_next <linux/uaccess.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
#define COMPAT_ACCESS_OK(type, addr, size) \
	access_ok(type, addr, size)
#else
#define COMPAT_ACCESS_OK(type, addr, size) \
	access_ok(addr, size)
#endif


#endif /* __COMPAT_LINUX_UACCESS_H */

