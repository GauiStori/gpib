/*
    Copyright (C) 2004-2006 Frank Mori Hess <fmhess@users.sourceforge.net>
    Copyright (C) 2005-2006 Ian Abbott

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
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/* 
 * This header exists so we can use the CLASS_DEVICE_CREATE macro
 * without regard as to whether we are using the compat/include/linux/device.h
 * or if we have just been copied directly into a linux kernel tree
 * and are not using the compat headers.
 */

#ifndef __COMPAT_DEVICE_CREATE_H_
#define __COMPAT_DEVICE_CREATE_H_

#include <linux/device.h>

#ifndef CLASS_DEVICE_CREATE

#define CLASS_DEVICE_CREATE(cs, parent, devt, drvdata, fmt...) \
	device_create(cs, parent, devt, drvdata, fmt)

#endif // CLASS_DEVICE_CREATE

#endif // __COMPAT_DEVICE_CREATE_H_
