/***************************************************************************
                              cb7210/cb7210_cs.c
                             -------------------
    Support for computer boards pcmcia-gpib card

    Based on gpib PCMCIA client driver written by Claus Schroeter
    (clausi@chemie.fu-berlin.de), which was adapted from the
    pcmcia skeleton example (presumably David Hinds)

    begin                : Jan 2002
    copyright            : (C) 2002 by Frank Mori Hess
    email                : fmhess@users.sourceforge.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "cb7210.h"

#if defined(GPIB_CONFIG_PCMCIA)

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/module.h>
#include <asm/io.h>

#include <pcmcia/cistpl.h>
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
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
#else
#define DEBUG(n, args...)
#endif


/*
   The event() function is this driver's Card Services event handler.
   It will be called by Card Services when an appropriate card status
   event is received.  The config() and release() entry points are
   used to configure or release a socket, in response to card insertion
   and ejection events.  They are invoked from the gpib event
   handler.
*/

static int cb_gpib_config( struct pcmcia_device  *link );
static void cb_gpib_release( struct pcmcia_device  *link );
int cb_pcmcia_attach(gpib_board_t *board, const gpib_board_config_t *config);
void cb_pcmcia_detach(gpib_board_t *board);

/*
   A linked list of "instances" of the gpib device.  Each actual
   PCMCIA card corresponds to one device instance, and is described
   by one dev_link_t structure (defined in ds.h).

   You may not want to use a linked list for this -- for example, the
   memory card driver uses an array of dev_link_t pointers, where minor
   device numbers are used to derive the corresponding array index.
*/

static  struct pcmcia_device  *curr_dev = NULL;

/*
   A dev_link_t structure has fields for most things that are needed
   to keep track of a socket, but there will usually be some device
   specific information that also needs to be kept track of.  The
   'priv' pointer in a dev_link_t structure can be used to point to
   a device-specific private data structure, like this.

   A driver needs to provide a dev_node_t structure for each device
   on a card.  In some cases, there is only one device per card (for
   example, ethernet cards, modems).  In other cases, there may be
   many actual or logical devices (SCSI adapters, memory cards with
   multiple partitions).  The dev_node_t structures need to be kept
   in a linked list starting at the 'dev' field of a dev_link_t
   structure.  We allocate them in the card's private data structure,
   because they generally can't be allocated dynamically.
*/

typedef struct local_info_t {
	struct pcmcia_device	*p_dev;
	gpib_board_t		*dev;
} local_info_t;

/*====================================================================

    gpib_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static int cb_gpib_probe( struct pcmcia_device *link )
{
	local_info_t *info;

//	int ret, i;

	DEBUG(0, "cb_gpib_probe(0x%p)\n", link);

	/* Allocate space for private device-specific data */
	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) return -ENOMEM;
	memset(info, 0, sizeof(*info));

	info->p_dev = link;
	link->priv = info;

	/* The io structure describes IO port mapping */
	link->resource[0]->end = 16;
	link->resource[0]->flags &= ~IO_DATA_PATH_WIDTH;
	link->resource[0]->flags |= IO_DATA_PATH_WIDTH_AUTO;
	link->resource[1]->end = 16;
	link->resource[1]->flags &= ~IO_DATA_PATH_WIDTH;
	link->resource[1]->flags |= IO_DATA_PATH_WIDTH_16;
	link->io_lines = 10;

	/* General socket configuration */
	link->config_flags = CONF_ENABLE_IRQ | CONF_AUTO_SET_IO;
	link->config_index = 1;
	link->config_regs = PRESENT_OPTION;

	/* Register with Card Services */
	curr_dev = link;
	return cb_gpib_config(link);
} /* gpib_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void cb_gpib_remove( struct pcmcia_device *link )
{
	local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;

	DEBUG(0, "cb_gpib_remove(0x%p)\n", link);

	if(info->dev)
		cb_pcmcia_detach(info->dev);
	cb_gpib_release(link);

	//free_netdev(dev);
	kfree(info);
}

static int cb_gpib_config_iteration
(
	struct pcmcia_device *link,
	void *priv_data
)
{
	return pcmcia_request_io(link);
}

/*======================================================================

    gpib_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    ethernet device available to the system.

======================================================================*/
/*@*/
static int cb_gpib_config( struct pcmcia_device  *link )
{
	struct pcmcia_device * handle;
	local_info_t *dev;
	int retval;

	handle = link;
	dev = link->priv;
	DEBUG(0, "cb_gpib_config(0x%p)\n", link);

	retval = pcmcia_loop_config(link, &cb_gpib_config_iteration, NULL);
	if(retval)
	{
		dev_warn(&link->dev, "no configuration found\n");
		cb_gpib_release(link);
		return -ENODEV;
	}

	DEBUG(0, "gpib_cs: manufacturer: 0x%x card: 0x%x\n", link->manf_id, link->card_id);


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	retval = pcmcia_request_irq(link, &link->irq);
	if (retval != 0) {
		dev_warn(&link->dev, "pcmcia_request_irq failed\n");
		cb_gpib_release(link);
		return -ENODEV;
	}
	printk(KERN_DEBUG "gpib_cs: IRQ_Line=%d\n", link->irq.AssignedIRQ);
#endif

	/*
	   This actually configures the PCMCIA socket -- setting up
	   the I/O windows and the interrupt mapping.
	*/
	retval = pcmcia_enable_device(link);
	if(retval != 0)
	{
		dev_warn(&link->dev, "pcmcia_enable_device failed\n");
		cb_gpib_release(link);
	    return -ENODEV;
	}

	printk(KERN_DEBUG "gpib device loaded\n");
	return 0;
} /* gpib_config */

/*======================================================================

    After a card is removed, gpib_release() will unregister the net
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.

======================================================================*/

static void cb_gpib_release( struct pcmcia_device *link )
{
	DEBUG(0, "cb_gpib_release(0x%p)\n", link);
	pcmcia_disable_device (link);
} /* gpib_release */

static int cb_gpib_suspend(struct pcmcia_device *link)
{
	//local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;
	DEBUG(0, "cb_gpib_suspend(0x%p)\n", link);

	if (link->open)
		printk("Device still open ???\n");
		//netif_device_detach(dev);

	return 0;
}

static int cb_gpib_resume(struct pcmcia_device *link)
{
	//local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;
	DEBUG(0, "cb_gpib_resume(0x%p)\n", link);

	/*if (link->open) {
		ni_gpib_probe(dev);	/ really?
		printk("Gpib resumed ???\n");
		//netif_device_attach(dev);
	}*/
	return cb_gpib_config(link);
}

/*====================================================================*/

static struct pcmcia_device_id cb_pcmcia_ids[] =
{
	PCMCIA_DEVICE_MANF_CARD(0x01c5, 0x0005),
	PCMCIA_DEVICE_NULL
};
MODULE_DEVICE_TABLE(pcmcia, cb_pcmcia_ids);

static struct pcmcia_driver cb_gpib_cs_driver =
{
	.owner		= THIS_MODULE,
	.drv = { .name = "cb_gpib_cs", },
	.id_table	= cb_pcmcia_ids,
	.probe		= cb_gpib_probe,
	.remove		= cb_gpib_remove,
	.suspend	= cb_gpib_suspend,
	.resume		= cb_gpib_resume,
};

int cb_pcmcia_init_module(void)
{
    pcmcia_register_driver(&cb_gpib_cs_driver);
    return 0;
}

void cb_pcmcia_cleanup_module(void)
{
	DEBUG(0, "cb_gpib_cs: unloading\n");
	pcmcia_unregister_driver(&cb_gpib_cs_driver);
}

gpib_interface_t cb_pcmcia_unaccel_interface =
{
	name: "cbi_pcmcia_unaccel",
	attach: cb_pcmcia_attach,
	detach: cb_pcmcia_detach,
	read: cb7210_read,
	write: cb7210_write,
	command: cb7210_command,
	take_control: cb7210_take_control,
	go_to_standby: cb7210_go_to_standby,
	request_system_control: cb7210_request_system_control,
	interface_clear: cb7210_interface_clear,
	remote_enable: cb7210_remote_enable,
	enable_eos: cb7210_enable_eos,
	disable_eos: cb7210_disable_eos,
	parallel_poll: cb7210_parallel_poll,
	parallel_poll_configure: cb7210_parallel_poll_configure,
	parallel_poll_response: cb7210_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: cb7210_line_status,
	update_status: cb7210_update_status,
	primary_address: cb7210_primary_address,
	secondary_address: cb7210_secondary_address,
	serial_poll_response: cb7210_serial_poll_response,
	serial_poll_status: cb7210_serial_poll_status,
	t1_delay: cb7210_t1_delay,
	return_to_local: cb7210_return_to_local,
};

gpib_interface_t cb_pcmcia_interface =
{
	name: "cbi_pcmcia",
	attach: cb_pcmcia_attach,
	detach: cb_pcmcia_detach,
	read: cb7210_accel_read,
	write: cb7210_accel_write,
	command: cb7210_command,
	take_control: cb7210_take_control,
	go_to_standby: cb7210_go_to_standby,
	request_system_control: cb7210_request_system_control,
	interface_clear: cb7210_interface_clear,
	remote_enable: cb7210_remote_enable,
	enable_eos: cb7210_enable_eos,
	disable_eos: cb7210_disable_eos,
	parallel_poll: cb7210_parallel_poll,
	parallel_poll_configure: cb7210_parallel_poll_configure,
	parallel_poll_response: cb7210_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: cb7210_line_status,
	update_status: cb7210_update_status,
	primary_address: cb7210_primary_address,
	secondary_address: cb7210_secondary_address,
	serial_poll_response: cb7210_serial_poll_response,
	serial_poll_status: cb7210_serial_poll_status,
	t1_delay: cb7210_t1_delay,
	return_to_local: cb7210_return_to_local,
};

gpib_interface_t cb_pcmcia_accel_interface =
{
	name: "cbi_pcmcia_accel",
	attach: cb_pcmcia_attach,
	detach: cb_pcmcia_detach,
	read: cb7210_accel_read,
	write: cb7210_accel_write,
	command: cb7210_command,
	take_control: cb7210_take_control,
	go_to_standby: cb7210_go_to_standby,
	request_system_control: cb7210_request_system_control,
	interface_clear: cb7210_interface_clear,
	remote_enable: cb7210_remote_enable,
	enable_eos: cb7210_enable_eos,
	disable_eos: cb7210_disable_eos,
	parallel_poll: cb7210_parallel_poll,
	parallel_poll_configure: cb7210_parallel_poll_configure,
	parallel_poll_response: cb7210_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: cb7210_line_status,
	update_status: cb7210_update_status,
	primary_address: cb7210_primary_address,
	secondary_address: cb7210_secondary_address,
	serial_poll_response: cb7210_serial_poll_response,
	serial_poll_status: cb7210_serial_poll_status,
	t1_delay: cb7210_t1_delay,
	return_to_local: cb7210_return_to_local,
};

int cb_pcmcia_attach( gpib_board_t *board, const gpib_board_config_t *config )
{
	cb7210_private_t *cb_priv;
	nec7210_private_t *nec_priv;
	int retval;

	if(curr_dev == NULL)
	{
		printk("no cb pcmcia cards found\n");
		return -1;
	}

	retval = cb7210_generic_attach(board);
	if(retval) return retval;

	cb_priv = board->private_data;
	nec_priv = &cb_priv->nec7210_priv;

	if(request_region(curr_dev->resource[0]->start, resource_size(curr_dev->resource[0]), "cb7210") == 0)
	{
		printk("gpib: ioports starting at 0x%lx are already in use\n", (unsigned long)curr_dev->resource[0]->start);
		return -EIO;
	}
	nec_priv->iobase = (void*)(unsigned long)curr_dev->resource[0]->start;
	cb_priv->fifo_iobase = curr_dev->resource[0]->start;

	if(request_irq(curr_dev->irq, cb7210_interrupt, IRQF_SHARED,
		"cb7210", board))
	{
		printk("cb7210: failed to request IRQ %d\n", curr_dev->irq);
		return -1;
	}
	cb_priv->irq = curr_dev->irq;

	return cb7210_init( cb_priv, board );
}

void cb_pcmcia_detach(gpib_board_t *board)
{
	cb7210_private_t *cb_priv = board->private_data;
	nec7210_private_t *nec_priv;

	if(cb_priv)
	{
		nec_priv = &cb_priv->nec7210_priv;
		gpib_free_pseudo_irq(board);
		if(cb_priv->irq)
			free_irq(cb_priv->irq, board);
		if(nec_priv->iobase)
		{
			nec7210_board_reset(nec_priv, board);
			release_region(nec7210_iobase(cb_priv), cb7210_iosize);
		}
	}
	cb7210_generic_detach(board);
}

#endif /* CONFIG_PCMCIA */
