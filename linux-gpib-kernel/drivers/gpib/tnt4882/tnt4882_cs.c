/*======================================================================

	Support for National Instruments PCMCIA GPIB cards

	Based on dummy PCMCIA client driver by David A. Hinds

	Copyright (C) 1999 David A. Hinds
	Copyright (C) 2002 Frank Mori Hess

======================================================================*/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "tnt4882.h"

#if defined( GPIB_CONFIG_PCMCIA )

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>

/*
All the PCMCIA modules use PCMCIA_DEBUG to control debugging.  If
you do not define PCMCIA_DEBUG at all, all the debug code will be
left out.  If you compile with PCMCIA_DEBUG=0, the debug code will
be present but disabled -- but it can then be enabled for specific
modules at load time with a 'pc_debug=#' option to insmod.
*/
#define PCMCIA_DEBUG 1
#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
module_param(pc_debug, int, 0);
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
#else
#define DEBUG(n, args...)
#endif

/*====================================================================*/

static int ni_gpib_config(struct pcmcia_device  *link);
static void ni_gpib_release(struct pcmcia_device *link);
int ni_pcmcia_attach(gpib_board_t *board, const gpib_board_config_t *config);
void ni_pcmcia_detach(gpib_board_t *board);

/*
A linked list of "instances" of the dummy device.  Each actual
PCMCIA card corresponds to one device instance, and is described
by one dev_link_t structure (defined in ds.h).

You may not want to use a linked list for this -- for example, the
memory card driver uses an array of dev_link_t pointers, where minor
device numbers are used to derive the corresponding array index.

I think this dev_list is obsolete but the pointer is needed to keep
the module instance for the ni_pcmcia_attach function.
*/

static struct pcmcia_device   *curr_dev = NULL;

typedef struct local_info_t {
	struct pcmcia_device	*p_dev;
	gpib_board_t		*dev;
	int			stop;
	struct bus_operations	*bus;
} local_info_t;


/*======================================================================

ni_gpib_probe() creates an "instance" of the driver, allocating
local data structures for one device.  The device is registered
with Card Services.

======================================================================*/

static int ni_gpib_probe(struct pcmcia_device *link)
{
	local_info_t *info;
	//struct gpib_board_t *dev;

	DEBUG(0, "ni_gpib_probe(0x%p)\n", link);

	/* Allocate space for private device-specific data */
	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) return -ENOMEM;
	memset(info, 0, sizeof(*info));

	info->p_dev = link;
	link->priv = info;

	/*
	General socket configuration defaults can go here.  In this
	client, we assume very little, and rely on the CIS for almost
	everything.  In most clients, many details (i.e., number, sizes,
	and attributes of IO windows) are fixed by the nature of the
	device, and can be hard-wired here.
	*/
	link->config_flags = CONF_ENABLE_IRQ | CONF_AUTO_SET_IO;

	/* Register with Card Services */
	curr_dev = link;
	return ni_gpib_config(link);
} /* ni_gpib_attach */

/*======================================================================

	This deletes a driver "instance".  The device is de-registered
	with Card Services.  If it has been released, all local data
	structures are freed.  Otherwise, the structures will be freed
	when the device is released.

======================================================================*/

static void ni_gpib_remove(struct pcmcia_device *link)
{
	local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;

	DEBUG(0, "ni_gpib_remove(0x%p)\n", link);

	if(info->dev)
		ni_pcmcia_detach(info->dev);
	ni_gpib_release(link);

	//free_netdev(dev);
	kfree(info);
}

static int ni_gpib_config_iteration
(
	struct pcmcia_device *link,
	void *priv_data
)
{
	int retval;

	retval = pcmcia_request_io(link);
	if(retval != 0)
	{
		return retval;
	}

	return 0;
}

/*======================================================================

	ni_gpib_config() is scheduled to run after a CARD_INSERTION event
	is received, to configure the PCMCIA socket, and to make the
	device available to the system.

======================================================================*/

static int ni_gpib_config(struct pcmcia_device *link)
{
	//local_info_t *info = link->priv;
	//gpib_board_t *dev = info->dev;
	int last_ret;

	DEBUG(0, "ni_gpib_config(0x%p)\n", link);

	last_ret = pcmcia_loop_config(link, &ni_gpib_config_iteration, NULL);
	if(last_ret)
	{
		dev_warn(&link->dev, "no configuration found\n");
		ni_gpib_release(link);
		return last_ret;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	last_ret = pcmcia_request_irq(link, &link->irq);
	if (last_ret != 0) {
		dev_warn(&link->dev, "pcmcia_request_irq failed\n");
		ni_gpib_release(link);
		return -ENODEV;
	}
	printk(KERN_DEBUG "gpib_cs: IRQ_Line=%d\n", link->irq.AssignedIRQ);
#endif

	last_ret = pcmcia_enable_device(link);
	if (last_ret) {
		ni_gpib_release(link);
		return last_ret;
	}
	return 0;
} /* ni_gpib_config */

/*======================================================================

After a card is removed, ni_gpib_release() will unregister the
device, and release the PCMCIA configuration.  If the device is
still open, this will be postponed until it is closed.

======================================================================*/

static void ni_gpib_release(struct pcmcia_device *link)
{
	DEBUG(0, "ni_gpib_release(0x%p)\n", link);
	pcmcia_disable_device (link);
} /* ni_gpib_release */

static int ni_gpib_suspend(struct pcmcia_device *link)
{
	//local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;
	DEBUG(0, "ni_gpib_suspend(0x%p)\n", link);

	if (link->open)
		printk("Device still open ???\n");
		//netif_device_detach(dev);

	return 0;
}

static int ni_gpib_resume(struct pcmcia_device *link)
{
	//local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;
	DEBUG(0, "ni_gpib_resume(0x%p)\n", link);

	/*if (link->open) {
		ni_gpib_probe(dev);	/ really?
		printk("Gpib resumed ???\n");
		//netif_device_attach(dev);
	}*/
	return ni_gpib_config(link);
}

/*====================================================================*/
static struct pcmcia_device_id ni_pcmcia_ids[] =
{
	PCMCIA_DEVICE_MANF_CARD(0x010b, 0x4882),
	PCMCIA_DEVICE_MANF_CARD(0x010b, 0x0c71), // NI PCMCIA-GPIB+
	PCMCIA_DEVICE_NULL
};

MODULE_DEVICE_TABLE(pcmcia, ni_pcmcia_ids);

static struct pcmcia_driver ni_gpib_cs_driver =
{
	.owner		= THIS_MODULE,
	.drv = { .name = "ni_gpib_cs", },
	.id_table	= ni_pcmcia_ids,
	.probe		= ni_gpib_probe,
	.remove		= ni_gpib_remove,
	.suspend	= ni_gpib_suspend,
	.resume		= ni_gpib_resume,
};

int __init init_ni_gpib_cs(void)
{
	return pcmcia_register_driver(&ni_gpib_cs_driver);
}

void __exit exit_ni_gpib_cs(void)
{
	DEBUG(0, "ni_gpib_cs: unloading\n");
	pcmcia_unregister_driver(&ni_gpib_cs_driver);
}

gpib_interface_t ni_pcmcia_interface =
{
	name: "ni_pcmcia",
	attach: ni_pcmcia_attach,
	detach: ni_pcmcia_detach,
	read: tnt4882_accel_read,
	write: tnt4882_accel_write,
	command: tnt4882_command,
	take_control: tnt4882_take_control,
	go_to_standby: tnt4882_go_to_standby,
	request_system_control: tnt4882_request_system_control,
	interface_clear: tnt4882_interface_clear,
	remote_enable: tnt4882_remote_enable,
	enable_eos: tnt4882_enable_eos,
	disable_eos: tnt4882_disable_eos,
	parallel_poll: tnt4882_parallel_poll,
	parallel_poll_configure: tnt4882_parallel_poll_configure,
	parallel_poll_response: tnt4882_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: tnt4882_line_status,
	update_status: tnt4882_update_status,
	primary_address: tnt4882_primary_address,
	secondary_address: tnt4882_secondary_address,
	serial_poll_response: tnt4882_serial_poll_response,
	serial_poll_status: tnt4882_serial_poll_status,
	t1_delay: tnt4882_t1_delay,
	return_to_local: tnt4882_return_to_local,
};

gpib_interface_t ni_pcmcia_accel_interface =
{
	name: "ni_pcmcia_accel",
	attach: ni_pcmcia_attach,
	detach: ni_pcmcia_detach,
	read: tnt4882_accel_read,
	write: tnt4882_accel_write,
	command: tnt4882_command,
	take_control: tnt4882_take_control,
	go_to_standby: tnt4882_go_to_standby,
	request_system_control: tnt4882_request_system_control,
	interface_clear: tnt4882_interface_clear,
	remote_enable: tnt4882_remote_enable,
	enable_eos: tnt4882_enable_eos,
	disable_eos: tnt4882_disable_eos,
	parallel_poll: tnt4882_parallel_poll,
	parallel_poll_configure: tnt4882_parallel_poll_configure,
	parallel_poll_response: tnt4882_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: tnt4882_line_status,
	update_status: tnt4882_update_status,
	primary_address: tnt4882_primary_address,
	secondary_address: tnt4882_secondary_address,
	serial_poll_response: tnt4882_serial_poll_response,
	serial_poll_status: tnt4882_serial_poll_status,
	t1_delay: tnt4882_t1_delay,
	return_to_local: tnt4882_return_to_local,
};

int ni_pcmcia_attach(gpib_board_t *board, const gpib_board_config_t *config)
{
	local_info_t *info;
	tnt4882_private_t *tnt_priv;
	nec7210_private_t *nec_priv;
	int isr_flags = IRQF_SHARED;

	DEBUG(0, "ni_pcmcia_attach(0x%p)\n", board);

	if( curr_dev == NULL )
	{
		printk( "gpib: no NI PCMCIA board found\n" );
		return -1;
	}

	info = curr_dev->priv;
	info->dev = board;

	board->status = 0;

	if(tnt4882_allocate_private(board))
		return -ENOMEM;
	tnt_priv = board->private_data;
	tnt_priv->io_writeb = outb_wrapper;
	tnt_priv->io_readb = inb_wrapper;
	tnt_priv->io_writew = outw_wrapper;
	tnt_priv->io_readw = inw_wrapper;
	nec_priv = &tnt_priv->nec7210_priv;
	nec_priv->type = TNT4882;
	nec_priv->read_byte = nec7210_locking_ioport_read_byte;
	nec_priv->write_byte = nec7210_locking_ioport_write_byte;
	nec_priv->offset = atgpib_reg_offset;

	DEBUG(0, "ioport1 window attributes: 0x%lx\n", curr_dev->resource[0]->flags );
	if(request_region(curr_dev->resource[0]->start, resource_size(curr_dev->resource[0]), "tnt4882") == 0)
	{
		printk("gpib: ioports starting at 0x%lx are already in use\n", (unsigned long)curr_dev->resource[0]->start);
		return -EIO;
	}

	nec_priv->iobase = (void*)(unsigned long)curr_dev->resource[0]->start;

	// get irq
	if( request_irq( curr_dev->irq, tnt4882_interrupt, isr_flags, "tnt4882", board))
	{
		printk("gpib: can't request IRQ %d\n", curr_dev->irq);
		return -1;
	}
	tnt_priv->irq = curr_dev->irq;

	tnt4882_init( tnt_priv, board );

	return 0;
}

void ni_pcmcia_detach(gpib_board_t *board)
{
	tnt4882_private_t *tnt_priv = board->private_data;
	nec7210_private_t *nec_priv;
	DEBUG(0, "ni_pcmcia_detach(0x%p)\n", board);

	if(tnt_priv)
	{
		nec_priv = &tnt_priv->nec7210_priv;
		if(tnt_priv->irq)
		{
			free_irq(tnt_priv->irq, board);
		}
		if(nec_priv->iobase)
		{
			tnt4882_board_reset(tnt_priv, board);
			release_region((unsigned long)nec_priv->iobase, pcmcia_gpib_iosize);
		}
	}
	tnt4882_free_private(board);
}

#endif	// GPIB_CONFIG_PCMCIA
