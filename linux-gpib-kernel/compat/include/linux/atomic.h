/*
    linux/atomic.h compatibility header

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

#ifndef __COMPAT_LINUX_ATOMIC_H
#define __COMPAT_LINUX_ATOMIC_H

#include_next <linux/atomic.h>
#include <asm/barrier.h>

#ifndef smp_mb__before_atomic
#define smp_mb__before_atomic()	mb()
#endif
#ifndef smp_mb__after_atomic
#define smp_mb__after_atomic()	mb()
#endif


#endif /* __COMPAT_LINUX_ATOMIC_H */

