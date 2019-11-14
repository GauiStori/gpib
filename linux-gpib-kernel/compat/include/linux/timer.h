/*
    linux/timer.h compatibility header

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

#ifndef __COMPAT_LINUX_TIMER_H
#define __COMPAT_LINUX_TIMER_H

#include <linux/bug.h>
#include <linux/version.h>

#include_next <linux/timer.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)

#define COMPAT_TIMER_ARG_TYPE unsigned long
#define COMPAT_FROM_TIMER(var, callback_timer, timer_fieldname) \
	container_of((struct timer_list *)(callback_timer), typeof(*var), timer_fieldname)
/* 4.14 implemented a broken timer_setup function, which we need to avoid.  This is why we have a COMPAT_TIMER_SETUP macro. */
#define COMPAT_TIMER_SETUP(timer, func, flags) compat_timer_setup((timer), (func), (flags))

static inline void compat_timer_setup(struct timer_list *timer,
	void (*func)(COMPAT_TIMER_ARG_TYPE), unsigned int flags)
{
	/* flags not supported */
	WARN_ON(flags != 0);
	
	setup_timer(timer, func, (COMPAT_TIMER_ARG_TYPE)timer);
}
#define timer_setup_on_stack(timer, func, flags) compat_timer_setup((timer), (func), (flags))
#define destroy_timer_on_stack(timer) do {} while(0)

#else

#define COMPAT_TIMER_ARG_TYPE struct timer_list *
#define COMPAT_FROM_TIMER(var, callback_timer, timer_fieldname) \
	from_timer(var, callback_timer, timer_fieldname)
#define COMPAT_TIMER_SETUP(timer, func, flags) timer_setup((timer), (func), (flags))

#endif

#endif /* __COMPAT_LINUX_TIMER_H */

