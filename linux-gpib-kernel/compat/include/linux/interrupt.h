/*
    linux/interrupt.h compatibility header

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

#ifndef __COMPAT_LINUX_INTERRUPT_H_
#define __COMPAT_LINUX_INTERRUPT_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 23)
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x) (void)(x)
#endif

/* if interrupt handler prototype has pt_regs* parameter */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
#define PT_REGS_ARG , struct pt_regs *regs
#define HAVE_PT_REGS
#else
#define PT_REGS_ARG
#undef HAVE_PT_REGS
#endif

#include_next <linux/interrupt.h>

#ifndef IRQF_SHARED
#define IRQF_SHARED SA_SHIRQ
#endif

#endif

