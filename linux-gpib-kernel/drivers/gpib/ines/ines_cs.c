/***************************************************************************
                          nec7210/ines_cs.c  -  description
                             -------------------
   support for ines PCMCIA GPIB boards.  Based on Claus Schroeter's
   pcmcia gpib driver, which used the skeleton example (David Hinds probably).

    copyright            : (C) 1999 Axel Dziemba (axel.dziemba@ines.de)
                           (C) 2002 by Frank Mori Hess
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

#include "ines.h"

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
#include <pcmcia/cisreg.h>

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

static int ines_gpib_config( struct pcmcia_device  *link );
static void ines_gpib_release( struct pcmcia_device  *link );
int ines_pcmcia_attach(gpib_board_t *board, const gpib_board_config_t *config);
int ines_pcmcia_accel_attach(gpib_board_t *board, const gpib_board_config_t *config);
void ines_pcmcia_detach(gpib_board_t *board);

/*
   A linked list of "instances" of the gpib device.  Each actual
   PCMCIA card corresponds to one device instance, and is described
   by one dev_link_t structure (defined in ds.h).

   You may not want to use a linked list for this -- for example, the
   memory card driver uses an array of dev_link_t pointers, where minor
   device numbers are used to derive the corresponding array index.
*/

static struct pcmcia_device *curr_dev = NULL;

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
	u_short manfid;
	u_short cardid;
} local_info_t;


/*
    gpib_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

*/

static int ines_gpib_probe( struct pcmcia_device *link )
{
	local_info_t *info;

//	int ret, i;

	DEBUG(0, "ines_gpib_probe(0x%p)\n", link);

	/* Allocate space for private device-specific data */
	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) return -ENOMEM;
	memset(info, 0, sizeof(*info));

	info->p_dev = link;
	link->priv = info;

	/* The io structure describes IO port mapping */
	link->resource[0]->end = 32;
	link->resource[0]->flags &= ~IO_DATA_PATH_WIDTH;
	link->resource[0]->flags |= IO_DATA_PATH_WIDTH_8;
	link->io_lines = 5;

	/* General socket configuration */
	link->config_flags = CONF_ENABLE_IRQ | CONF_AUTO_SET_IO;

	/* Register with Card Services */
	curr_dev = link;
	return ines_gpib_config(link);
} /* gpib_attach */

/*

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

*/
static void ines_gpib_remove( struct pcmcia_device *link )
{
	local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;

	DEBUG(0, "ines_gpib_remove(0x%p)\n", link);

	if(info->dev)
		ines_pcmcia_detach(info->dev);
	ines_gpib_release(link);

	//free_netdev(dev);
	kfree(info);
}

static int ines_gpib_config_iteration
(
	struct pcmcia_device *link,
	void *priv_data
)
{
	return pcmcia_request_io(link);
}

/*
    gpib_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    ethernet device available to the system.

*/
static int ines_gpib_config( struct pcmcia_device *link )
{
	local_info_t *dev;
	int retval;
	void *virt;

	dev = link->priv;
	DEBUG(0, "ines_gpib_config(0x%p)\n", link);

	retval = pcmcia_loop_config(link, &ines_gpib_config_iteration, NULL);
	if(retval)
	{
		dev_warn(&link->dev, "no configuration found\n");
		ines_gpib_release(link);
		return -ENODEV;
	}

	printk(KERN_DEBUG "ines_cs: manufacturer: 0x%x card: 0x%x\n",
		link->manf_id, link->card_id);

	/*  for the ines card we have to setup the configuration registers in
		attribute memory here
	*/
	link->resource[2]->flags |= WIN_MEMORY_TYPE_AM | WIN_DATA_WIDTH_8 | WIN_ENABLE;
	link->resource[2]->end = 0x1000;
	retval = pcmcia_request_window(link, link->resource[2], 250);
	if (retval) {
		dev_warn(&link->dev, "pcmcia_request_window failed\n");
		ines_gpib_release(link);
		return -ENODEV;
	}
	retval = pcmcia_map_mem_page(link, link->resource[2], 0);
	if (retval) {
		dev_warn(&link->dev, "pcmcia_map_mem_page failed\n");
		ines_gpib_release(link);
		return -ENODEV;
	}
	virt = ioremap( link->resource[2]->start, resource_size(link->resource[2]) );
	writeb( ( link->resource[2]->start >> 2 ) & 0xff, virt + 0xf0 ); // IOWindow base
	iounmap( ( void* ) virt );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	retval = pcmcia_request_irq(link, &link->irq);
	if (retval != 0) {
		dev_warn(&link->dev, "pcmcia_request_irq failed\n");
		ines_gpib_release(link);
		return -ENODEV;
	}
	printk(KERN_DEBUG "gpib_cs: IRQ_Line=%d\n", link->irq.AssignedIRQ);
#endif

	/*
	This actually configures the PCMCIA socket -- setting up
	the I/O windows and the interrupt mapping.
	*/
	retval = pcmcia_enable_device(link);
	if (retval) {
		ines_gpib_release(link);
		return -ENODEV;
	}
	printk(KERN_DEBUG "ines gpib device loaded\n");
	return 0;
} /* gpib_config */

/*

    After a card is removed, gpib_release() will unregister the net
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.

*/

static void ines_gpib_release( struct pcmcia_device *link )
{
	DEBUG(0, "ines_gpib_release(0x%p)\n", link);
	pcmcia_disable_device (link);
} /* gpib_release */

static int ines_gpib_suspend(struct pcmcia_device *link)
{
	//local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;
	DEBUG(0, "ines_gpib_suspend(0x%p)\n", link);

	if (link->open)
		printk("Device still open ???\n");
		//netif_device_detach(dev);

	return 0;
}

static int ines_gpib_resume(struct pcmcia_device *link)
{
	//local_info_t *info = link->priv;
	//struct gpib_board_t *dev = info->dev;
	DEBUG(0, "ines_gpib_resume(0x%p)\n", link);

	/*if (link->open) {
		ni_gpib_probe(dev);	/ really?
		printk("Gpib resumed ???\n");
		//netif_device_attach(dev);
	}*/
	return ines_gpib_config(link);
}


static struct pcmcia_device_id ines_pcmcia_ids[] =
{
	PCMCIA_DEVICE_MANF_CARD(0x01b4, 0x4730),
	PCMCIA_DEVICE_NULL
};
MODULE_DEVICE_TABLE(pcmcia, ines_pcmcia_ids);

static struct pcmcia_driver ines_gpib_cs_driver =
{
	.owner		= THIS_MODULE,
	.drv = { .name = "ines_gpib_cs", },
	.id_table	= ines_pcmcia_ids,
	.probe		= ines_gpib_probe,
	.remove		= ines_gpib_remove,
	.suspend	= ines_gpib_suspend,
	.resume		= ines_gpib_resume,
};

int ines_pcmcia_init_module(void)
{
	pcmcia_register_driver(&ines_gpib_cs_driver);
	return 0;
}

void ines_pcmcia_cleanup_module(void)
{
	DEBUG(0, "ines_cs: unloading\n");
	pcmcia_unregister_driver(&ines_gpib_cs_driver);
}

gpib_interface_t ines_pcmcia_unaccel_interface =
{
	name: "ines_pcmcia_unaccel",
	attach: ines_pcmcia_attach,
	detach: ines_pcmcia_detach,
	read: ines_read,
	write: ines_write,
	command: ines_command,
	take_control: ines_take_control,
	go_to_standby: ines_go_to_standby,
	request_system_control: ines_request_system_control,
	interface_clear: ines_interface_clear,
	remote_enable: ines_remote_enable,
	enable_eos: ines_enable_eos,
	disable_eos: ines_disable_eos,
	parallel_poll: ines_parallel_poll,
	parallel_poll_configure: ines_parallel_poll_configure,
	parallel_poll_response: ines_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: ines_line_status,
	update_status: ines_update_status,
	primary_address: ines_primary_address,
	secondary_address: ines_secondary_address,
	serial_poll_response: ines_serial_poll_response,
	serial_poll_status: ines_serial_poll_status,
	t1_delay: ines_t1_delay,
	return_to_local: ines_return_to_local,
};

gpib_interface_t ines_pcmcia_accel_interface =
{
	name: "ines_pcmcia_accel",
	attach: ines_pcmcia_accel_attach,
	detach: ines_pcmcia_detach,
	read: ines_accel_read,
	write: ines_accel_write,
	command: ines_command,
	take_control: ines_take_control,
	go_to_standby: ines_go_to_standby,
	request_system_control: ines_request_system_control,
	interface_clear: ines_interface_clear,
	remote_enable: ines_remote_enable,
	enable_eos: ines_enable_eos,
	disable_eos: ines_disable_eos,
	parallel_poll: ines_parallel_poll,
	parallel_poll_configure: ines_parallel_poll_configure,
	parallel_poll_response: ines_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: ines_line_status,
	update_status: ines_update_status,
	primary_address: ines_primary_address,
	secondary_address: ines_secondary_address,
	serial_poll_response: ines_serial_poll_response,
	serial_poll_status: ines_serial_poll_status,
	t1_delay: ines_t1_delay,
	return_to_local: ines_return_to_local,
};

gpib_interface_t ines_pcmcia_interface =
{
	name: "ines_pcmcia",
	attach: ines_pcmcia_accel_attach,
	detach: ines_pcmcia_detach,
	read: ines_accel_read,
	write: ines_accel_write,
	command: ines_command,
	take_control: ines_take_control,
	go_to_standby: ines_go_to_standby,
	request_system_control: ines_request_system_control,
	interface_clear: ines_interface_clear,
	remote_enable: ines_remote_enable,
	enable_eos: ines_enable_eos,
	disable_eos: ines_disable_eos,
	parallel_poll: ines_parallel_poll,
	parallel_poll_configure: ines_parallel_poll_configure,
	parallel_poll_response: ines_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: ines_line_status,
	update_status: ines_update_status,
	primary_address: ines_primary_address,
	secondary_address: ines_secondary_address,
	serial_poll_response: ines_serial_poll_response,
	serial_poll_status: ines_serial_poll_status,
	t1_delay: ines_t1_delay,
	return_to_local: ines_return_to_local,
};

irqreturn_t ines_pcmcia_interrupt(int irq, void *arg PT_REGS_ARG)
{
	gpib_board_t *board = arg;
	return ines_interrupt(board);
}

int ines_common_pcmcia_attach( gpib_board_t *board )
{
	ines_private_t *ines_priv;
	nec7210_private_t *nec_priv;
	int retval;

	if(curr_dev == NULL)
	{
		printk("no ines pcmcia cards found\n");
		return -1;
	}

	retval = ines_generic_attach(board);
	if(retval) return retval;

	ines_priv = board->private_data;
	nec_priv = &ines_priv->nec7210_priv;

	if(request_region(curr_dev->resource[0]->start, resource_size(curr_dev->resource[0]), "ines_gpib") == 0)
	{
		printk("ines_gpib: ioports at 0x%lx already in use\n", (unsigned long)(curr_dev->resource[0]->start));
		return -1;
	}

	nec_priv->iobase = (void*)(unsigned long)curr_dev->resource[0]->start;

	nec7210_board_reset( nec_priv, board );

	if(request_irq(curr_dev->irq, ines_pcmcia_interrupt, IRQF_SHARED, "pcmcia-gpib", board))
	{
		printk("gpib: can't request IRQ %d\n", curr_dev->irq);
		return -1;
	}
	ines_priv->irq = curr_dev->irq;

	return 0;
}

int ines_pcmcia_attach( gpib_board_t *board, const gpib_board_config_t *config)
{
	ines_private_t *ines_priv;
	int retval;

	retval = ines_common_pcmcia_attach( board );
	if( retval < 0 ) return retval;

	ines_priv = board->private_data;
	ines_online( ines_priv, board, 0 );

	return 0;
}

int ines_pcmcia_accel_attach( gpib_board_t *board , const gpib_board_config_t *config)
{
	ines_private_t *ines_priv;
	int retval;

	retval = ines_common_pcmcia_attach( board );
	if( retval < 0 ) return retval;

	ines_priv = board->private_data;
	ines_online( ines_priv, board, 1 );

	return 0;
}

void ines_pcmcia_detach(gpib_board_t *board)
{
	ines_private_t *ines_priv = board->private_data;
	nec7210_private_t *nec_priv;

	if(ines_priv)
	{
		nec_priv = &ines_priv->nec7210_priv;
		if(ines_priv->irq)
			free_irq(ines_priv->irq, board);
		if(nec_priv->iobase)
		{
			nec7210_board_reset(nec_priv, board);
			release_region((unsigned long)(nec_priv->iobase), ines_pcmcia_iosize);
		}
	}
	ines_free_private(board);
}

#endif /* CONFIG_PCMCIA */
