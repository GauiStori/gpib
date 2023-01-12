/***************************************************************************
                          agilent_82357a/agilent_82357a.c
                             -------------------
 driver for Agilent 82357A usb to gpib adapters

    begin                : 2004-10-31
    copyright            : (C) 2004 by Frank Mori Hess
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

#define _GNU_SOURCE

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "agilent_82357a.h"
#include "gpibP.h"
#include "tms9914.h"

MODULE_LICENSE("GPL");

#define MAX_NUM_82357A_INTERFACES 128
static struct usb_interface *agilent_82357a_driver_interfaces[MAX_NUM_82357A_INTERFACES];
DEFINE_MUTEX(agilent_82357a_hotplug_lock);

unsigned int agilent_82357a_update_status( gpib_board_t *board, unsigned int clear_mask );

static void agilent_82357a_bulk_complete(struct urb *urb PT_REGS_ARG)
{
	agilent_82357a_urb_context_t *context = urb->context;

//	printk("debug: %s: status=0x%x, error_count=%i, actual_length=%i\n", __FUNCTION__,
//		urb->status, urb->error_count, urb->actual_length);

	up(&context->complete);
}

static void agilent_82357a_timeout_handler(COMPAT_TIMER_ARG_TYPE t)
{
	agilent_82357a_private_t *a_priv = COMPAT_FROM_TIMER(a_priv, t, bulk_timer);
	agilent_82357a_urb_context_t *context = &a_priv->context;
	context->timed_out = 1;
	up(&context->complete);
}

int agilent_82357a_send_bulk_msg(agilent_82357a_private_t *a_priv, void *data, int data_length, int *actual_data_length,
	int timeout_msecs)
{
	struct usb_device *usb_dev;
	int retval;
	unsigned int out_pipe;
	agilent_82357a_urb_context_t *context = &a_priv->context;

	*actual_data_length = 0;
	retval = mutex_lock_interruptible(&a_priv->bulk_alloc_lock);
	if(retval) return retval;
	if(a_priv->bus_interface == NULL)
	{
		mutex_unlock(&a_priv->bulk_alloc_lock);
		return -ENODEV;
	}
	if(a_priv->bulk_urb)
	{
		mutex_unlock(&a_priv->bulk_alloc_lock);
		return -EAGAIN;
	}
	a_priv->bulk_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(a_priv->bulk_urb == NULL)
	{
		mutex_unlock(&a_priv->bulk_alloc_lock);
		return -ENOMEM;
	}
	usb_dev = interface_to_usbdev(a_priv->bus_interface);
	out_pipe = usb_sndbulkpipe(usb_dev, a_priv->bulk_out_endpoint);
	sema_init(&context->complete, 0);
	context->timed_out = 0;
	usb_fill_bulk_urb(a_priv->bulk_urb, usb_dev, out_pipe, data, data_length,
		&agilent_82357a_bulk_complete, context);

	if(timeout_msecs)
		mod_timer(&a_priv->bulk_timer, jiffies + msecs_to_jiffies(timeout_msecs));

	//printk("%s: submitting urb\n", __FUNCTION__);
	retval = usb_submit_urb(a_priv->bulk_urb, GFP_KERNEL);
	if(retval)
	{
		printk("%s: failed to submit bulk out urb, retval=%i\n", __FUNCTION__, retval);
		mutex_unlock(&a_priv->bulk_alloc_lock);
		goto cleanup;
	}
	mutex_unlock(&a_priv->bulk_alloc_lock);
	if(down_interruptible(&context->complete))
	{
		printk("%s: interrupted\n", __FUNCTION__);
		retval = -ERESTARTSYS;
		goto cleanup;
	}
	if(context->timed_out)
	{
		retval = -ETIMEDOUT;
	}else
	{
		retval = a_priv->bulk_urb->status;
		*actual_data_length = a_priv->bulk_urb->actual_length;
	}
cleanup:
	if(timeout_msecs)
	{
		if(timer_pending(&a_priv->bulk_timer))
			del_timer_sync(&a_priv->bulk_timer);
	}
	mutex_lock(&a_priv->bulk_alloc_lock);
	if(a_priv->bulk_urb)
	{
		usb_kill_urb(a_priv->bulk_urb);
		usb_free_urb(a_priv->bulk_urb);
		a_priv->bulk_urb = NULL;
	}
	mutex_unlock(&a_priv->bulk_alloc_lock);
	return retval;
}

int agilent_82357a_receive_bulk_msg(agilent_82357a_private_t *a_priv, void *data, int data_length, int *actual_data_length,
	int timeout_msecs)
{
	struct usb_device *usb_dev;
	int retval;
	unsigned int in_pipe;
	agilent_82357a_urb_context_t *context = &a_priv->context;

	*actual_data_length = 0;
	retval = mutex_lock_interruptible(&a_priv->bulk_alloc_lock);
	if(retval) return retval;
	if(a_priv->bus_interface == NULL)
	{
		mutex_unlock(&a_priv->bulk_alloc_lock);
		return -ENODEV;
	}
	if(a_priv->bulk_urb)
	{
		mutex_unlock(&a_priv->bulk_alloc_lock);
		return -EAGAIN;
	}
	a_priv->bulk_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(a_priv->bulk_urb == NULL)
	{
		mutex_unlock(&a_priv->bulk_alloc_lock);
		return -ENOMEM;
	}
	usb_dev = interface_to_usbdev(a_priv->bus_interface);
	in_pipe = usb_rcvbulkpipe(usb_dev, AGILENT_82357_BULK_IN_ENDPOINT);
	sema_init(&context->complete, 0);
	context->timed_out = 0;
	usb_fill_bulk_urb(a_priv->bulk_urb, usb_dev, in_pipe, data, data_length,
		&agilent_82357a_bulk_complete, context);

	if(timeout_msecs)
		mod_timer(&a_priv->bulk_timer, jiffies + msecs_to_jiffies(timeout_msecs));

	//printk("%s: submitting urb\n", __FUNCTION__);
	retval = usb_submit_urb(a_priv->bulk_urb, GFP_KERNEL);
	if(retval)
	{
		printk("%s: failed to submit bulk out urb, retval=%i\n", __FUNCTION__, retval);
		mutex_unlock(&a_priv->bulk_alloc_lock);
		goto cleanup;
	}
	mutex_unlock(&a_priv->bulk_alloc_lock);
	if(down_interruptible(&context->complete))
	{
		printk("%s: interrupted\n", __FUNCTION__);
		retval = -ERESTARTSYS;
		goto cleanup;
	}
	if(context->timed_out)
	{
		retval = -ETIMEDOUT;
		goto cleanup;
	}
	retval = a_priv->bulk_urb->status;
	*actual_data_length = a_priv->bulk_urb->actual_length;
cleanup:
	if(timeout_msecs)
		del_timer_sync(&a_priv->bulk_timer);

	mutex_lock(&a_priv->bulk_alloc_lock);
	if(a_priv->bulk_urb)
	{
		usb_kill_urb(a_priv->bulk_urb);
		usb_free_urb(a_priv->bulk_urb);
		a_priv->bulk_urb = NULL;
	}
	mutex_unlock(&a_priv->bulk_alloc_lock);
	return retval;
}

int agilent_82357a_receive_control_msg(agilent_82357a_private_t *a_priv, __u8 request, __u8 requesttype, __u16 value,
	__u16 index, void *data, __u16 size, int timeout_msecs)
{
	struct usb_device *usb_dev;
	int retval;
	unsigned int in_pipe;

	retval = mutex_lock_interruptible(&a_priv->control_alloc_lock);
	if(retval) return retval;
	if(a_priv->bus_interface == NULL)
	{
		mutex_unlock(&a_priv->control_alloc_lock);
		return -ENODEV;
	}
	usb_dev = interface_to_usbdev(a_priv->bus_interface);
	in_pipe = usb_rcvctrlpipe(usb_dev, AGILENT_82357_CONTROL_ENDPOINT);
	retval = USB_CONTROL_MSG(usb_dev, in_pipe, request, requesttype, value, index, data, size, timeout_msecs);
	mutex_unlock(&a_priv->control_alloc_lock);
	return retval;
}

static void agilent_82357a_dump_raw_block(const uint8_t *raw_data, int length)
{
	int i;

	printk("%s:", __FUNCTION__);
	for(i = 0; i < length; ++i)
	{
		if(i % 8 == 0)
			printk("\n");
		printk(" %2x", raw_data[i]);
	}
	printk("\n");
}

int agilent_82357a_write_registers(agilent_82357a_private_t *a_priv, const struct agilent_82357a_register_pairlet *writes,
	int num_writes)
{
	int retval;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written, bytes_read;
	int i = 0;
	int j;
	static const int bytes_per_write = 2;
	static const int header_length = 2;
	static const int max_writes = 31;

	if(num_writes > max_writes)
	{
		printk("%s: bug! num_writes=%i too large\n", __FUNCTION__, num_writes);
		return -EIO;
	}
	out_data_length = num_writes * bytes_per_write + header_length;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}
	out_data[i++] = DATA_PIPE_CMD_WR_REGS;
	out_data[i++] = num_writes;
	for(j = 0; j < num_writes; j++)
	{
		out_data[i++] = writes[j].address;
		out_data[i++] = writes[j].value;
	}
	if(i > out_data_length)
	{
		printk("%s: bug! buffer overrun\n", __FUNCTION__);
	}
	retval = mutex_lock_interruptible(&a_priv->bulk_transfer_lock);
	if(retval)
	{
		kfree(out_data);
		return retval;
	}
	retval = agilent_82357a_send_bulk_msg(a_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval)
	{
		printk("%s: agilent_82357a_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FUNCTION__,
			retval, bytes_written, i);
		mutex_unlock(&a_priv->bulk_transfer_lock);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FUNCTION__);
		mutex_unlock(&a_priv->bulk_transfer_lock);
		return -ENOMEM;
	}
	retval = agilent_82357a_receive_bulk_msg(a_priv, in_data, in_data_length, &bytes_read, 1000);
	mutex_unlock(&a_priv->bulk_transfer_lock);
	if(retval)
	{
		printk("%s: agilent_82357a_receive_bulk_msg returned %i, bytes_read=%i\n", __FUNCTION__, retval, bytes_read);
		agilent_82357a_dump_raw_block(in_data, bytes_read);
		kfree(in_data);
		return -EIO;
	}
	if(in_data[0] != (0xff & ~DATA_PIPE_CMD_WR_REGS))
	{
		printk("%s: error, bulk command=0x%x != ~DATA_PIPE_CMD_WR_REGS\n", __FUNCTION__, in_data[0]);
		return -EIO;
	}
	if(in_data[1])
	{
		printk("%s: nonzero error code 0x%x in DATA_PIPE_CMD_WR_REGS response\n", __FUNCTION__, in_data[1]);
		return -EIO;
	}
	kfree(in_data);
	return 0;
}

int agilent_82357a_read_registers(agilent_82357a_private_t *a_priv, struct agilent_82357a_register_pairlet *reads,
	int num_reads, int blocking)
{
	int retval;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written, bytes_read;
	int i = 0;
	int j;
	static const int header_length = 2;
	static const int max_reads = 62;

	if(num_reads > max_reads)
	{
		printk("%s: bug! num_reads=%i too large\n", __FUNCTION__, num_reads);
	}
	out_data_length = num_reads + header_length;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}
	out_data[i++] = DATA_PIPE_CMD_RD_REGS;
	out_data[i++] = num_reads;
	for(j = 0; j < num_reads; j++)
	{
		out_data[i++] = reads[j].address;
	}
	if(i > out_data_length)
	{
		printk("%s: bug! buffer overrun\n", __FUNCTION__);
	}
	if(blocking)
	{
		retval = mutex_lock_interruptible(&a_priv->bulk_transfer_lock);
		if(retval)
		{
			kfree(out_data);
			return retval;
		}
	}else
	{
		retval = mutex_trylock(&a_priv->bulk_transfer_lock);
		if(retval == 0)
		{
			kfree(out_data);
			return -EAGAIN;
		}
	}
	retval = agilent_82357a_send_bulk_msg(a_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval)
	{
		printk("%s: agilent_82357a_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FUNCTION__,
			retval, bytes_written, i);
		mutex_unlock(&a_priv->bulk_transfer_lock);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FUNCTION__);
		mutex_unlock(&a_priv->bulk_transfer_lock);
		return -ENOMEM;
	}
	retval = agilent_82357a_receive_bulk_msg(a_priv, in_data, in_data_length, &bytes_read, 10000);
	mutex_unlock(&a_priv->bulk_transfer_lock);
	if(retval)
	{
		printk("%s: agilent_82357a_receive_bulk_msg returned %i, bytes_read=%i\n", __FUNCTION__, retval, bytes_read);
		agilent_82357a_dump_raw_block(in_data, bytes_read);
		kfree(in_data);
		return -EIO;
	}
	i = 0;
	if(in_data[i++] != (0xff & ~DATA_PIPE_CMD_RD_REGS))
	{
		printk("%s: error, bulk command=0x%x != ~DATA_PIPE_CMD_RD_REGS\n", __FUNCTION__, in_data[0]);
		return -EIO;
	}
	if(in_data[i++])
	{
		printk("%s: nonzero error code 0x%x in DATA_PIPE_CMD_RD_REGS response\n", __FUNCTION__, in_data[1]);
		return -EIO;
	}
	for(j = 0; j < num_reads; j++)
	{
		reads[j].value = in_data[i++];
	}
	kfree(in_data);
	return 0;
}

static int agilent_82357a_abort(agilent_82357a_private_t *a_priv, int flush)
{
	int retval = 0;
	int receive_control_retval;
	uint16_t wIndex = 0;
	uint8_t *status_data;
	static const unsigned status_data_len = 2;

	status_data = kmalloc(status_data_len, GFP_KERNEL);
	if(status_data == NULL) return -ENOMEM;

	if(flush)
		wIndex |= XA_FLUSH;
	receive_control_retval = agilent_82357a_receive_control_msg(a_priv, agilent_82357a_control_request, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		XFER_ABORT, wIndex, status_data, status_data_len, 100);
	if(receive_control_retval < 0)
	{
		printk("%s: agilent_82357a_receive_control_msg() returned %i\n", __FUNCTION__, receive_control_retval);
		retval = -EIO;
		goto cleanup;
	}
	if(status_data[0] != (~XFER_ABORT & 0xff))
	{
		printk("%s: error, major code=0x%x != ~XFER_ABORT\n", __FUNCTION__, status_data[0]);
		retval = -EIO;
		goto cleanup;
	}
	switch(status_data[1])
	{
	case UGP_SUCCESS:
		retval = 0;
		break;
	case UGP_ERR_FLUSHING:
		if(flush)
		{
			retval = 0;
			break;
		}
		fallthrough;
		// fallthrough;
	case UGP_ERR_FLUSHING_ALREADY:
	default:
		printk("%s: abort returned error code=0x%x\n", __FUNCTION__, status_data[1]);
		retval = -EIO;
		break;
	}

cleanup:
	kfree(status_data);
	return retval;
}

// interface functions
int agilent_82357a_read(gpib_board_t *board, uint8_t *buffer, size_t length, int *end, size_t *nbytes)
{
	int retval;
	agilent_82357a_private_t *a_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written, bytes_read;
	int i = 0;
	uint8_t trailing_flags;
	unsigned long start_jiffies = jiffies;
	int msec_timeout;

	*nbytes = 0;
	*end = 0;
	out_data_length = 0x9;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL) return -ENOMEM;
	out_data[i++] = DATA_PIPE_CMD_READ;
	out_data[i++] = 0;	//primary address when ARF_NO_ADDR is not set
	out_data[i++] = 0;	//secondary address when ARF_NO_ADDR is not set
	out_data[i] = ARF_NO_ADDRESS | ARF_END_ON_EOI;
	if(a_priv->eos_mode & REOS)
		out_data[i] |= ARF_END_ON_EOS_CHAR;
	++i;
	out_data[i++] = length & 0xff;
	out_data[i++] = (length >> 8) & 0xff;
	out_data[i++] = (length >> 16) & 0xff;
	out_data[i++] = (length >> 24) & 0xff;
	out_data[i++] = a_priv->eos_char;
	msec_timeout = (board->usec_timeout + 999) / 1000;
	retval = mutex_lock_interruptible(&a_priv->bulk_transfer_lock);
	if(retval)
	{
		kfree(out_data);
		return retval;
	}
	retval = agilent_82357a_send_bulk_msg(a_priv, out_data, i, &bytes_written, msec_timeout);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		printk("%s: agilent_82357a_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FUNCTION__, retval, bytes_written, i);
		mutex_unlock(&a_priv->bulk_transfer_lock);
		if(retval < 0) return retval;
		return -EIO;
	}
	in_data_length = length + 1;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&a_priv->bulk_transfer_lock);
		return -ENOMEM;
	}
	if(board->usec_timeout != 0)
		msec_timeout -= jiffies_to_msecs(jiffies - start_jiffies) - 1;
	if(msec_timeout >= 0)
	{
		retval = agilent_82357a_receive_bulk_msg(a_priv, in_data, in_data_length,
			&bytes_read, msec_timeout);
	}else
	{
		retval = -ETIMEDOUT;
		bytes_read = 0;
	}
	if(retval == -ETIMEDOUT)
	{
		int extra_bytes_read;
		int extra_bytes_retval;
		agilent_82357a_abort(a_priv, 1);
		extra_bytes_retval = agilent_82357a_receive_bulk_msg(a_priv, in_data + bytes_read, in_data_length - bytes_read,
			&extra_bytes_read, 100);
		printk("%s: agilent_82357a_receive_bulk_msg timed out, bytes_read=%i, extra_bytes_read=%i\n",
			__FUNCTION__, bytes_read, extra_bytes_read);
		bytes_read += extra_bytes_read;
		if(extra_bytes_retval)
		{
			printk("%s: extra_bytes_retval=%i, bytes_read=%i\n", __FUNCTION__,
				extra_bytes_retval, bytes_read);
			agilent_82357a_abort(a_priv, 0);
		}
	}else if(retval)
	{
		printk("%s: agilent_82357a_receive_bulk_msg returned %i, bytes_read=%i\n", __FUNCTION__,
			retval, bytes_read);
		agilent_82357a_abort(a_priv, 0);
	}
	mutex_unlock(&a_priv->bulk_transfer_lock);
	if(bytes_read > length + 1)
	{
		bytes_read = length + 1;
		printk("%s: bytes_read > length? truncating", __FUNCTION__);
	}
	//printk("%s: received response:\n", __FUNCTION__);
	//agilent_82357a_dump_raw_block(in_data, in_data_length);
	if(bytes_read >= 1)
	{
		memcpy(buffer, in_data, bytes_read - 1);
		trailing_flags = in_data[bytes_read - 1];
		*nbytes = bytes_read - 1;
		if(trailing_flags & (ATRF_EOI | ATRF_EOS)) *end = 1;
	}
	kfree(in_data);
	//FIXME check trailing flags for error
	return retval;
}

static ssize_t agilent_82357a_generic_write(gpib_board_t *board, uint8_t *buffer, size_t length,
	int send_commands, int send_eoi, size_t *bytes_written)
{
	int retval;
	agilent_82357a_private_t *a_priv = board->private_data;
	uint8_t *out_data = NULL;
	uint8_t *status_data = NULL;
	int out_data_length;
	int raw_bytes_written;
	int i = 0, j;
	int msec_timeout;
	unsigned short bsr, adsr;
	struct agilent_82357a_register_pairlet read_reg;

	*bytes_written = 0;
	out_data_length = length + 0x8;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL) return -ENOMEM;
	out_data[i++] = DATA_PIPE_CMD_WRITE;
	out_data[i++] = 0; // primary address when AWF_NO_ADDRESS is not set
	out_data[i++] = 0; // secondary address when AWF_NO_ADDRESS is not set
	out_data[i] = AWF_NO_ADDRESS | AWF_NO_FAST_TALKER_FIRST_BYTE;
	if(send_commands)
		out_data[i] |= AWF_ATN | AWF_NO_FAST_TALKER;
	if(send_eoi)
		out_data[i] |= AWF_SEND_EOI;
	++i;
	out_data[i++] = length & 0xff;
	out_data[i++] = (length >> 8) & 0xff;
	out_data[i++] = (length >> 16) & 0xff;
	out_data[i++] = (length >> 24) & 0xff;
	for(j = 0; j < length; j++)
		out_data[i++] = buffer[j];
	//printk("%s: sending bulk msg(), send_commands=%i\n", __FUNCTION__, send_commands);

	smp_mb__before_atomic();
	clear_bit(AIF_WRITE_COMPLETE_BN, &a_priv->interrupt_flags);
	smp_mb__after_atomic();

	msec_timeout = (board->usec_timeout + 999) / 1000;
	retval = mutex_lock_interruptible(&a_priv->bulk_transfer_lock);
	if(retval)
	{
		kfree(out_data);
		return retval;
	}
	retval = agilent_82357a_send_bulk_msg(a_priv, out_data, i, &raw_bytes_written, msec_timeout);
	kfree(out_data);
	if(retval || raw_bytes_written != i)
	{
		agilent_82357a_abort(a_priv, 0);
		printk("%s: agilent_82357a_send_bulk_msg returned %i, raw_bytes_written=%i, i=%i\n", __FUNCTION__,
			retval, raw_bytes_written, i);
		mutex_unlock(&a_priv->bulk_transfer_lock);
		if(retval < 0) return retval;
		return -EIO;
	}
	//printk("%s: waiting for write complete\n", __FUNCTION__);
	retval = wait_event_interruptible(board->wait, test_bit(AIF_WRITE_COMPLETE_BN, &a_priv->interrupt_flags) ||
		test_bit(TIMO_NUM, &board->status));
	if(retval)
	{
		printk("%s: wait write complete interrupted\n", __FUNCTION__);
		agilent_82357a_abort(a_priv, 0);
		mutex_unlock(&a_priv->bulk_transfer_lock);
		return -ERESTARTSYS;
	}

	if(test_bit(AIF_WRITE_COMPLETE_BN, &a_priv->interrupt_flags) == 0)
	{
	        GPIB_DPRINTK("%s: write timed out ibs %i, tmo %i\n", __FUNCTION__,
			test_bit(TIMO_NUM, &board->status),msec_timeout);

		agilent_82357a_abort(a_priv, 0);

		mutex_unlock(&a_priv->bulk_transfer_lock);

		read_reg.address = BSR;
		retval = agilent_82357a_read_registers(a_priv, &read_reg, 1, 1);
		if ( retval )
		{
			printk("%s: agilent_82357a_read_registers() returned error\n",  __FUNCTION__);
			return -ETIMEDOUT;
		}

		bsr = read_reg.value;
		GPIB_DPRINTK("%s: write aborted bsr 0x%hx\n", __FUNCTION__, bsr );

		if (send_commands) /* check for no listeners */
		{
			if ((bsr & BSR_ATN_BIT) && !(bsr & ( BSR_NDAC_BIT | BSR_NRFD_BIT )))
			{
				GPIB_DPRINTK("%s: No listener on command\n", __FUNCTION__ );
				clear_bit(TIMO_NUM, &board->status);
				return -ENOTCONN; // no listener on bus
			}
		} else {
			read_reg.address = ADSR;
			retval = agilent_82357a_read_registers(a_priv, &read_reg, 1, 1);
			if ( retval )
			{
				printk("%s: agilent_82357a_read_registers() returned error\n",  __FUNCTION__);
				return -ETIMEDOUT;
			}
			adsr = read_reg.value;
			if ((adsr & HR_TA) && !(bsr & ( BSR_NDAC_BIT | BSR_NRFD_BIT )))
			{
				GPIB_DPRINTK("%s: No listener on write\n", __FUNCTION__ );
				clear_bit(TIMO_NUM, &board->status);
				return -ECOMM;
			}
		}

		return -ETIMEDOUT;
	}

	status_data = kmalloc(STATUS_DATA_LEN, GFP_KERNEL);
	if(status_data == NULL)
	{
		mutex_unlock(&a_priv->bulk_transfer_lock);
		return -ENOMEM;
	}

	// printk("%s: receiving control msg\n", __FUNCTION__);
	retval = agilent_82357a_receive_control_msg(a_priv, agilent_82357a_control_request,
						USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
						XFER_STATUS, 0, status_data, STATUS_DATA_LEN, 100);
	mutex_unlock(&a_priv->bulk_transfer_lock);
	if(retval < 0)
	{
		printk("%s: agilent_82357a_receive_control_msg() returned %i\n", __FUNCTION__, retval);
		kfree(status_data);
		return -EIO;
	}
	*bytes_written  = status_data[2];
	*bytes_written |= status_data[3] << 8;
	*bytes_written |= status_data[4] << 16;
	*bytes_written |= status_data[5] << 24;

	kfree(status_data);
	//printk("%s: write completed, bytes_written=%i\n", __FUNCTION__, (int)*bytes_written);
	return 0;
}

static int agilent_82357a_write(gpib_board_t *board, uint8_t *buffer, size_t length, int send_eoi, size_t *bytes_written)
{
	return agilent_82357a_generic_write(board, buffer, length, 0, send_eoi, bytes_written);
}

int agilent_82357a_command(gpib_board_t *board, uint8_t *buffer, size_t length, size_t *bytes_written)
{
	return agilent_82357a_generic_write(board, buffer, length, 1, 0, bytes_written);
}

int agilent_82357a_take_control(gpib_board_t *board, int synchronous)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	const int timeout = 10;
	struct agilent_82357a_register_pairlet write;
	int retval;
	int i;

	write.address = AUXCR;
	if(synchronous)
	{
		write.value = AUX_TCS;
	}else
		write.value = AUX_TCA;
	retval = agilent_82357a_write_registers(a_priv, &write, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
	}
	// busy wait until ATN is asserted
	for(i = 0; i < timeout; ++i)
	{
		agilent_82357a_update_status(board, 0);
		if(test_bit( ATN_NUM, &board->status ))
			break;
		udelay(1);
	}
	if( i == timeout )
	{
		return -ETIMEDOUT;
	};
	return 0;
}

int agilent_82357a_go_to_standby(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet write;
	int retval;

	write.address = AUXCR;
	write.value = AUX_GTS;
	retval = agilent_82357a_write_registers(a_priv, &write, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
	}
	return 0;
}

//FIXME should change prototype to return int
void agilent_82357a_request_system_control(gpib_board_t *board, int request_control)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet writes[2];
	int retval;
	int i = 0;

	/* 82357B needs bit to be set in 9914 AUXCR register */
	writes[i].address = AUXCR;
	if(request_control)
	{
		writes[i].value = AUX_RQC;
		a_priv->hw_control_bits |= SYSTEM_CONTROLLER;
	}else
	{
		writes[i].value = AUX_RLC;
		a_priv->is_cic = 0;
		a_priv->hw_control_bits &= ~SYSTEM_CONTROLLER;
	}
	++i;
	writes[i].address = HW_CONTROL;
	writes[i].value = a_priv->hw_control_bits;
	++i;
	retval = agilent_82357a_write_registers(a_priv, writes, i);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
	}
	return;// retval;
}
void agilent_82357a_interface_clear(gpib_board_t *board, int assert)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet write;
	int retval;

	write.address = AUXCR;
	write.value = AUX_SIC;
	if(assert)
	{
		write.value |= AUX_CS;
		a_priv->is_cic = 1;
	}
	retval = agilent_82357a_write_registers(a_priv, &write, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
	}
	return;
}
void agilent_82357a_remote_enable(gpib_board_t *board, int enable)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet write;
	int retval;

	write.address = AUXCR;
	write.value = AUX_SRE;
	if(enable)
	{
		write.value |= AUX_CS;
	}
	retval = agilent_82357a_write_registers(a_priv, &write, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
	}
	return;// 0;
}

int agilent_82357a_enable_eos(gpib_board_t *board, uint8_t eos_byte, int compare_8_bits)
{
	agilent_82357a_private_t *a_priv = board->private_data;

	if(compare_8_bits == 0)
	{
		printk("%s: hardware only supports 8-bit EOS compare", __FUNCTION__);
		return -EOPNOTSUPP;
	}
	a_priv->eos_char = eos_byte;
	a_priv->eos_mode = REOS | BIN;
	return 0;
}

void agilent_82357a_disable_eos(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv = board->private_data;

	a_priv->eos_mode &= ~REOS;
}

unsigned int agilent_82357a_update_status( gpib_board_t *board, unsigned int clear_mask )
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet address_status;
	int retval;

	smp_mb__before_atomic();

	board->status &= ~clear_mask;
	if(a_priv->is_cic)
		set_bit( CIC_NUM, &board->status);
	else
		clear_bit( CIC_NUM, &board->status);
	address_status.address = ADSR;
	retval = agilent_82357a_read_registers(a_priv, &address_status, 1, 0);
	if(retval)
	{
		if (retval != -EAGAIN)
			printk("%s: agilent_82357a_read_registers() returned error\n", __FUNCTION__);
		return board->status;
	}
	// check for remote/local
	if(address_status.value & HR_REM)
		set_bit( REM_NUM, &board->status );
	else
		clear_bit( REM_NUM, &board->status );
	// check for lockout
	if(address_status.value & HR_LLO)
		set_bit( LOK_NUM, &board->status );
	else
		clear_bit( LOK_NUM, &board->status );
	// check for ATN
	if(address_status.value & HR_ATN)
	{
		set_bit( ATN_NUM, &board->status );
	}else
	{
		clear_bit( ATN_NUM, &board->status );
	}
	// check for talker/listener addressed
	if(address_status.value & HR_TA)
	{
		set_bit( TACS_NUM, &board->status );
	}else
		clear_bit( TACS_NUM, &board->status );
	if(address_status.value & HR_LA)
	{
		set_bit(LACS_NUM, &board->status);
	}else
		clear_bit( LACS_NUM, &board->status );

	smp_mb__after_atomic();

	return board->status;
}

int agilent_82357a_primary_address(gpib_board_t *board, unsigned int address)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet write;
	int retval;

	// put primary address in address0
	write.address = ADR;
	write.value = address & ADDRESS_MASK;
	retval = agilent_82357a_write_registers(a_priv, &write, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
		return retval;
	}
	return retval;
}

int agilent_82357a_secondary_address(gpib_board_t *board, unsigned int address, int enable)
{
	if(enable)
		printk("%s: warning: assigning a secondary address not supported\n", __FUNCTION__);
	return  -EOPNOTSUPP;
}

int agilent_82357a_parallel_poll(gpib_board_t *board, uint8_t *result)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet writes[2];
	struct agilent_82357a_register_pairlet read;
	int retval;

	// execute parallel poll
	writes[0].address = AUXCR;
	writes[0].value = AUX_CS | AUX_RPP;
	writes[1].address = HW_CONTROL;
	writes[1].value = a_priv->hw_control_bits & ~NOT_PARALLEL_POLL;
	retval = agilent_82357a_write_registers(a_priv, writes, 2);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
		return retval;
	}
	udelay(2);	//silly, since usb write will take way longer
	read.address = CPTR;
	retval = agilent_82357a_read_registers(a_priv, &read, 1, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_read_registers() returned error\n", __FUNCTION__);
		return retval;
	}
	*result = read.value;
	// clear parallel poll state
	writes[0].address = HW_CONTROL;
	writes[0].value = a_priv->hw_control_bits | NOT_PARALLEL_POLL;
	writes[1].address = AUXCR;
	writes[1].value = AUX_RPP;
	retval = agilent_82357a_write_registers(a_priv, writes, 2);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n",  __FUNCTION__);
		return retval;
	}
	return 0;
}
void agilent_82357a_parallel_poll_configure(gpib_board_t *board, uint8_t config)
{
	//board can only be system controller
	return;// 0;
}
void agilent_82357a_parallel_poll_response(gpib_board_t *board, int ist)
{
	//board can only be system controller
	return;// 0;
}
void agilent_82357a_serial_poll_response(gpib_board_t *board, uint8_t status)
{
	//board can only be system controller
	return;// 0;
}
uint8_t agilent_82357a_serial_poll_status( gpib_board_t *board )
{
	//board can only be system controller
	return 0;
}
void agilent_82357a_return_to_local( gpib_board_t *board )
{
	//board can only be system controller
	return;// 0;
}
int agilent_82357a_line_status( const gpib_board_t *board )
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet bus_status;
	int retval;
	int status = ValidALL;

	bus_status.address = BSR;
	retval = agilent_82357a_read_registers(a_priv, &bus_status, 1, 0);
	if(retval)
	{
		if (retval != -EAGAIN)
			printk("%s: agilent_82357a_read_registers() returned error\n", __FUNCTION__);
		return retval;
	}
	if( bus_status.value & BSR_REN_BIT )
		status |= BusREN;
	if( bus_status.value & BSR_IFC_BIT )
		status |= BusIFC;
	if( bus_status.value & BSR_SRQ_BIT )
		status |= BusSRQ;
	if( bus_status.value & BSR_EOI_BIT )
		status |= BusEOI;
	if( bus_status.value & BSR_NRFD_BIT )
		status |= BusNRFD;
	if( bus_status.value & BSR_NDAC_BIT )
		status |= BusNDAC;
	if( bus_status.value & BSR_DAV_BIT )
		status |= BusDAV;
	if( bus_status.value & BSR_ATN_BIT )
		status |= BusATN;
	return status;
}

static unsigned short nanosec_to_fast_talker_bits(unsigned int *nanosec)
{
	static const int nanosec_per_bit = 21;
	static const int max_value = 0x72;
	static const int min_value = 0x11;
	unsigned short bits;

	bits = (*nanosec + nanosec_per_bit / 2) / nanosec_per_bit;
	if(bits < min_value) bits = min_value;
	if(bits > max_value) bits = max_value;
	*nanosec = bits * nanosec_per_bit;
	return bits;
}

unsigned int agilent_82357a_t1_delay( gpib_board_t *board, unsigned int nanosec )
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet write;
	int retval;

	write.address = FAST_TALKER_T1;
	write.value = nanosec_to_fast_talker_bits(&nanosec);
	retval = agilent_82357a_write_registers(a_priv, &write, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
	}
	return nanosec;
}

void agilent_82357a_interrupt_complete(struct urb *urb PT_REGS_ARG)
{
	gpib_board_t *board = urb->context;
	agilent_82357a_private_t *a_priv = board->private_data;
	int retval;
	uint8_t *transfer_buffer = urb->transfer_buffer;
	unsigned long interrupt_flags;

#if 0
	int i;

	printk("%s: status=0x%x, error_count=%i, actual_length=%i transfer_buffer:\n", __FUNCTION__,
		urb->status, urb->error_count, urb->actual_length);
	for(i = 0; i < urb->actual_length; ++i)
	{
		printk("%2x ", transfer_buffer[i]);
	}
	printk("\n");
#endif
	switch (urb->status) {
	/* success */
	case 0:
		break;
	/* unlinked, don't resubmit */
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default: /* other error, resubmit */
		retval = usb_submit_urb(a_priv->interrupt_urb, GFP_ATOMIC);
		if(retval)
		{
			printk("%s: failed to resubmit interrupt urb\n", __FUNCTION__);
		}
		return;
	}

	smp_mb__before_atomic();

	interrupt_flags = transfer_buffer[0];
	if(test_bit(AIF_READ_COMPLETE_BN, &interrupt_flags))
		set_bit(AIF_READ_COMPLETE_BN, &a_priv->interrupt_flags);
	if(test_bit(AIF_WRITE_COMPLETE_BN, &interrupt_flags))
		set_bit(AIF_WRITE_COMPLETE_BN, &a_priv->interrupt_flags);
	if(test_bit(AIF_SRQ_BN, &interrupt_flags))
		set_bit(SRQI_NUM, &board->status);

	smp_mb__after_atomic();

	wake_up_interruptible(&board->wait);

	retval = usb_submit_urb(a_priv->interrupt_urb, GFP_ATOMIC);
	if(retval)
	{
		printk("%s: failed to resubmit interrupt urb\n", __FUNCTION__);
	}
}

static int agilent_82357a_setup_urbs(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct usb_device *usb_dev;
	int int_pipe;
	int retval;

	retval = mutex_lock_interruptible(&a_priv->interrupt_alloc_lock);
	if(retval) return retval;
	if(a_priv->bus_interface == NULL)
	{
		retval = -ENODEV;
		goto setup_exit;
	}

	a_priv->interrupt_buffer = kmalloc(INTERRUPT_BUF_LEN, GFP_KERNEL);
	if(a_priv->interrupt_buffer == NULL)
	{
		retval = -ENOMEM;
		goto setup_exit;
	}
	a_priv->interrupt_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(a_priv->interrupt_urb == NULL)
	{
		retval = -ENOMEM;
		goto setup_exit;
	}
	usb_dev = interface_to_usbdev(a_priv->bus_interface);
	int_pipe = usb_rcvintpipe(usb_dev, a_priv->interrupt_in_endpoint);
	usb_fill_int_urb(a_priv->interrupt_urb, usb_dev, int_pipe, a_priv->interrupt_buffer,
		INTERRUPT_BUF_LEN, &agilent_82357a_interrupt_complete, board, 1);
	retval = usb_submit_urb(a_priv->interrupt_urb, GFP_KERNEL);
	if(retval)
	{
		usb_free_urb(a_priv->interrupt_urb);
		a_priv->interrupt_urb = NULL;
		printk("%s: failed to submit first interrupt urb, retval=%i\n", __FUNCTION__, retval);
		goto setup_exit;
	}
	mutex_unlock(&a_priv->interrupt_alloc_lock);
	return 0;

 setup_exit:
	if(a_priv->interrupt_buffer)
	        kfree(a_priv->interrupt_buffer);
	mutex_unlock(&a_priv->interrupt_alloc_lock);
	return retval;
}

#if 0
static int agilent_82357a_reset_usb_configuration(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct usb_device *usb_dev;
	int retval;

	if(a_priv->bus_interface == NULL)
	{
		return -ENODEV;
	}
	usb_dev = interface_to_usbdev(a_priv->bus_interface);
	retval = usb_reset_configuration(usb_dev);
	if(retval)
	{
		printk("%s: usb_reset_configuration() returned %i\n", __FUNCTION__, retval);
	}
	return retval;
}
#endif

static void agilent_82357a_cleanup_urbs(agilent_82357a_private_t *a_priv)
{
	if(a_priv && a_priv->bus_interface)
	{
		if(a_priv->interrupt_urb)
			usb_kill_urb(a_priv->interrupt_urb);
		if(a_priv->bulk_urb)
			usb_kill_urb(a_priv->bulk_urb);
	}
};

static int agilent_82357a_allocate_private(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv;

	board->private_data = kmalloc(sizeof(agilent_82357a_private_t), GFP_KERNEL);
	if(board->private_data == NULL)
		return -ENOMEM;
	a_priv = board->private_data;
	memset(a_priv, 0, sizeof(agilent_82357a_private_t));
	mutex_init(&a_priv->bulk_transfer_lock);
	mutex_init(&a_priv->bulk_alloc_lock);
	mutex_init(&a_priv->control_alloc_lock);
	mutex_init(&a_priv->interrupt_alloc_lock);
	return 0;
}

static void agilent_82357a_free_private(agilent_82357a_private_t *a_priv)
{
	if(a_priv->interrupt_urb)
		usb_free_urb(a_priv->interrupt_urb);
	if(a_priv->interrupt_buffer)
	        kfree(a_priv->interrupt_buffer);
	kfree(a_priv);
	return;
}

static int agilent_82357a_init(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet hw_control;
	struct agilent_82357a_register_pairlet writes[0x20];
	int retval;
	int i;
	unsigned int nanosec;

	i = 0;
	writes[i].address = LED_CONTROL;
	writes[i].value = FAIL_LED_ON;
	++i;
	writes[i].address = RESET_TO_POWERUP;
	writes[i].value = RESET_SPACEBALL;
	++i;
	retval = agilent_82357a_write_registers(a_priv, writes, i);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
		return -EIO;
	}
	set_current_state(TASK_INTERRUPTIBLE);
	if(schedule_timeout(usec_to_jiffies(2000)))
		return -ERESTARTSYS;
	i = 0;
	writes[i].address = AUXCR;
	writes[i].value = AUX_NBAF;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_HLDE;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_TON;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_LON;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_RSV2;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_INVAL;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_RPP;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_STDL;
	++i;
	writes[i].address = AUXCR;
	writes[i].value = AUX_VSTDL;
	++i;
	writes[i].address = FAST_TALKER_T1;
	nanosec = 800;
	writes[i].value = nanosec_to_fast_talker_bits(&nanosec);
	board->t1_nano_sec = nanosec;
	++i;
	writes[i].address = ADR;
	writes[i].value = board->pad & ADDRESS_MASK;
	++i;
	writes[i].address = PPR;
	writes[i].value = 0;
	++i;
	writes[i].address = SPMR;
	writes[i].value = 0;
	++i;
	writes[i].address = PROTOCOL_CONTROL;
	writes[i].value = WRITE_COMPLETE_INTERRUPT_EN;
	++i;
	writes[i].address = IMR0;
	writes[i].value = HR_BOIE | HR_BIIE;
	++i;
	writes[i].address = IMR1;
	writes[i].value = HR_SRQIE;
	++i;
	// turn off reset state
	writes[i].address = AUXCR;
	writes[i].value = AUX_CHIP_RESET;
	++i;
	writes[i].address = LED_CONTROL;
	writes[i].value = FIRMWARE_LED_CONTROL;
	++i;
	if(i > sizeof(writes) / sizeof(writes[0]))
	{
		printk("%s: bug! writes[] overflow\n", __FUNCTION__);
		return -EFAULT;
	}
	retval = agilent_82357a_write_registers(a_priv, writes, i);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
		return -EIO;
	}
	hw_control.address = HW_CONTROL;
	retval = agilent_82357a_read_registers(a_priv, &hw_control, 1, 1);
	if(retval)
	{
		printk("%s: agilent_82357a_read_registers() returned error\n", __FUNCTION__);
		return -EIO;
	}
	a_priv->hw_control_bits = (hw_control.value & ~0x7) | NOT_TI_RESET | NOT_PARALLEL_POLL;
;
	return 0;
}

static inline int agilent_82357a_device_match(struct usb_interface *interface, const gpib_board_config_t *config)
{
	struct usb_device * const usbdev = interface_to_usbdev(interface);

	if(gpib_match_device_path(&interface->dev, config->device_path) == 0)
	{
		return 0;
	}
	if(config->serial_number != NULL &&
		strcmp(usbdev->serial, config->serial_number) != 0)
	{
		return 0;
	};

	return 1;
}

int agilent_82357a_attach(gpib_board_t *board, const gpib_board_config_t *config)
{
	int retval;
	int i;
	unsigned product_id;
	agilent_82357a_private_t *a_priv;
	struct usb_device *usb_dev;

	if(mutex_lock_interruptible(&agilent_82357a_hotplug_lock))
		return -ERESTARTSYS;
	//printk("%s: enter\n", __FUNCTION__);
	retval = agilent_82357a_allocate_private(board);
	if(retval < 0)
	{
		mutex_unlock(&agilent_82357a_hotplug_lock);
		return retval;
	}
	a_priv = board->private_data;
	for(i = 0; i < MAX_NUM_82357A_INTERFACES; ++i)
	{
		if(agilent_82357a_driver_interfaces[i] && usb_get_intfdata(agilent_82357a_driver_interfaces[i]) == NULL &&
			agilent_82357a_device_match(agilent_82357a_driver_interfaces[i], config))
		{
			a_priv->bus_interface = agilent_82357a_driver_interfaces[i];
			usb_set_intfdata(agilent_82357a_driver_interfaces[i], board);
			usb_dev = interface_to_usbdev(a_priv->bus_interface);
			dev_info(&usb_dev->dev,"bus %d dev num %d attached to gpib minor %d, agilent usb interface %i\n",
			       usb_dev->bus->busnum, usb_dev->devnum, board->minor, i);
			break;
		}
	}
	if(i == MAX_NUM_82357A_INTERFACES)
	{
		mutex_unlock(&agilent_82357a_hotplug_lock);
		printk("No Agilent 82357 gpib adapters found, have you loaded its firmware?\n");
		return -ENODEV;
	}
	product_id = USBID_TO_CPU(interface_to_usbdev(a_priv->bus_interface)->descriptor.idProduct);
	switch(product_id)
	{
	case USB_DEVICE_ID_AGILENT_82357A:
		a_priv->bulk_out_endpoint = AGILENT_82357A_BULK_OUT_ENDPOINT;
		a_priv->interrupt_in_endpoint = AGILENT_82357A_INTERRUPT_IN_ENDPOINT;
		break;
	case USB_DEVICE_ID_AGILENT_82357B:
		a_priv->bulk_out_endpoint = AGILENT_82357B_BULK_OUT_ENDPOINT;
		a_priv->interrupt_in_endpoint = AGILENT_82357B_INTERRUPT_IN_ENDPOINT;
		break;
	default:
		printk("bug, unhandled product_id in switch?\n");
		return -EIO;
		break;
	}
#if 0
	retval = agilent_82357a_reset_usb_configuration(board);
	if(retval < 0)
	{
		mutex_unlock(&agilent_82357a_hotplug_lock);
		return retval;
	}
#endif
	retval = agilent_82357a_setup_urbs(board);
	if(retval < 0)
	{
		mutex_unlock(&agilent_82357a_hotplug_lock);
		return retval;
	}
	//printk("%s: finished setup_urbs()()\n", __FUNCTION__);

	COMPAT_TIMER_SETUP(&a_priv->bulk_timer, agilent_82357a_timeout_handler, 0);

	retval = agilent_82357a_init(board);

	if(retval < 0)
	{
		mutex_unlock(&agilent_82357a_hotplug_lock);
		return retval;
	}
	//printk("%s: finished init()\n", __FUNCTION__);

	printk("%s: attached\n", __FUNCTION__);
	mutex_unlock(&agilent_82357a_hotplug_lock);
	return retval;
}

static int agilent_82357a_go_idle(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv = board->private_data;
	struct agilent_82357a_register_pairlet writes[0x20];
	int retval;
	int i;

	i = 0;
	// turn on tms9914 reset state
	writes[i].address = AUXCR;
	writes[i].value = AUX_CS | AUX_CHIP_RESET;
	++i;
	a_priv->hw_control_bits &= ~NOT_TI_RESET;
	writes[i].address = HW_CONTROL;
	writes[i].value = a_priv->hw_control_bits;
	++i;
	writes[i].address = PROTOCOL_CONTROL;
	writes[i].value = 0;
	++i;
	writes[i].address = IMR0;
	writes[i].value = 0;
	++i;
	writes[i].address = IMR1;
	writes[i].value = 0;
	++i;
	writes[i].address = LED_CONTROL;
	writes[i].value = 0;
	++i;
	if(i > sizeof(writes) / sizeof(writes[0]))
	{
		printk("%s: bug! writes[] overflow\n", __FUNCTION__);
		return -EFAULT;
	}
	retval = agilent_82357a_write_registers(a_priv, writes, i);
	if(retval)
	{
		printk("%s: agilent_82357a_write_registers() returned error\n", __FUNCTION__);
		return -EIO;
	}
	return 0;
}

void agilent_82357a_detach(gpib_board_t *board)
{
	agilent_82357a_private_t *a_priv;

	mutex_lock(&agilent_82357a_hotplug_lock);
	//printk("%s: enter\n", __FUNCTION__);
	a_priv = board->private_data;
	if(a_priv)
	{
		if(a_priv->bus_interface)
		{
			agilent_82357a_go_idle(board);
			usb_set_intfdata(a_priv->bus_interface, NULL);
		}
		mutex_lock(&a_priv->control_alloc_lock);
		mutex_lock(&a_priv->bulk_alloc_lock);
		mutex_lock(&a_priv->interrupt_alloc_lock);
		agilent_82357a_cleanup_urbs(a_priv);
		agilent_82357a_free_private(a_priv);
	}
	printk("%s: detached\n", __FUNCTION__);
	mutex_unlock(&agilent_82357a_hotplug_lock);
}

gpib_interface_t agilent_82357a_gpib_interface =
{
	name : "agilent_82357a",
	attach : agilent_82357a_attach,
	detach : agilent_82357a_detach,
	read : agilent_82357a_read,
	write : agilent_82357a_write,
	command : agilent_82357a_command,
	take_control : agilent_82357a_take_control,
	go_to_standby : agilent_82357a_go_to_standby,
	request_system_control : agilent_82357a_request_system_control,
	interface_clear : agilent_82357a_interface_clear,
	remote_enable : agilent_82357a_remote_enable,
	enable_eos : agilent_82357a_enable_eos,
	disable_eos : agilent_82357a_disable_eos,
	parallel_poll : agilent_82357a_parallel_poll,
	parallel_poll_configure : agilent_82357a_parallel_poll_configure,
	parallel_poll_response : agilent_82357a_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status : agilent_82357a_line_status,
	update_status : agilent_82357a_update_status,
	primary_address : agilent_82357a_primary_address,
	secondary_address : agilent_82357a_secondary_address,
	serial_poll_response : agilent_82357a_serial_poll_response,
	serial_poll_status : agilent_82357a_serial_poll_status,
	t1_delay : agilent_82357a_t1_delay,
	return_to_local : agilent_82357a_return_to_local,
	no_7_bit_eos : 1,
	skip_check_for_command_acceptors: 1
};

// Table with the USB-devices: just now only testing IDs
static struct usb_device_id agilent_82357a_driver_device_table [] =
{
	{USB_DEVICE(USB_VENDOR_ID_AGILENT, USB_DEVICE_ID_AGILENT_82357A)},
	{USB_DEVICE(USB_VENDOR_ID_AGILENT, USB_DEVICE_ID_AGILENT_82357B)},
	{} /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, agilent_82357a_driver_device_table);

static int agilent_82357a_driver_probe(struct usb_interface *interface,
	const struct usb_device_id *id)
{
	int i;
	char *path;
	static const int pathLength = 1024;

	GPIB_DPRINTK("agilent_82357a_driver_probe\n");
	if(mutex_lock_interruptible(&agilent_82357a_hotplug_lock))
		return -ERESTARTSYS;
	usb_get_dev(interface_to_usbdev(interface));
	for(i = 0; i < MAX_NUM_82357A_INTERFACES; ++i)
	{
		if(agilent_82357a_driver_interfaces[i] == NULL)
		{
			agilent_82357a_driver_interfaces[i] = interface;
			usb_set_intfdata(interface, NULL);
			GPIB_DPRINTK("set bus interface %i to address 0x%p\n", i, interface);
			break;
		}
	}
	if(i == MAX_NUM_82357A_INTERFACES)
	{
		usb_put_dev(interface_to_usbdev(interface));
		mutex_unlock(&agilent_82357a_hotplug_lock);
		printk("out of space in agilent_82357a_driver_interfaces[]\n");
		return -1;
	}
	path = kmalloc(pathLength, GFP_KERNEL);
	if(path == NULL)
	{
		usb_put_dev(interface_to_usbdev(interface));
		mutex_unlock(&agilent_82357a_hotplug_lock);
		return -ENOMEM;
	}
	usb_make_path(interface_to_usbdev(interface), path, pathLength);
	printk("probe succeeded for path: %s\n", path);
	kfree(path);
	mutex_unlock(&agilent_82357a_hotplug_lock);
	return 0;
}

static void agilent_82357a_driver_disconnect(struct usb_interface *interface)
{
	int i;

	mutex_lock(&agilent_82357a_hotplug_lock);
//	printk("%s: enter\n", __FUNCTION__);
	for(i = 0; i < MAX_NUM_82357A_INTERFACES; ++i)
	{
		if(agilent_82357a_driver_interfaces[i] == interface)
		{
			gpib_board_t *board = usb_get_intfdata(interface);

			if(board)
			{
				agilent_82357a_private_t *a_priv = board->private_data;
				if(a_priv)
				{
					mutex_lock(&a_priv->control_alloc_lock);
					mutex_lock(&a_priv->bulk_alloc_lock);
					mutex_lock(&a_priv->interrupt_alloc_lock);
					agilent_82357a_cleanup_urbs(a_priv);
					a_priv->bus_interface = NULL;
					mutex_unlock(&a_priv->interrupt_alloc_lock);
					mutex_unlock(&a_priv->bulk_alloc_lock);
					mutex_unlock(&a_priv->control_alloc_lock);
				}
			}
			GPIB_DPRINTK("nulled agilent_82357a_driver_interfaces[%i]\n", i);
			agilent_82357a_driver_interfaces[i] = NULL;
			break;
		}
	}
	if(i == MAX_NUM_82357A_INTERFACES)
	{
		printk("unable to find interface in agilent_82357a_driver_interfaces[]? bug?\n");
	}
	usb_put_dev(interface_to_usbdev(interface));
//	printk("%s: exit\n", __FUNCTION__);
	mutex_unlock(&agilent_82357a_hotplug_lock);
}

static struct usb_driver agilent_82357a_bus_driver =
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	.owner = THIS_MODULE,
#endif
	.name = "agilent_82357a_gpib",
	.probe = agilent_82357a_driver_probe,
	.disconnect = agilent_82357a_driver_disconnect,
	.id_table = agilent_82357a_driver_device_table,
};

static int __init agilent_82357a_init_module(void)
{
	int i;

	pr_info("agilent_82357a_gpib driver loading");
	for(i = 0; i < MAX_NUM_82357A_INTERFACES; ++i)
		agilent_82357a_driver_interfaces[i] = NULL;
	usb_register(&agilent_82357a_bus_driver);
	gpib_register_driver(&agilent_82357a_gpib_interface, THIS_MODULE);

	return 0;
}

static void __exit agilent_82357a_exit_module(void)
{
	pr_info("agilent_82357a_gpib driver unloading");
//	printk("%s: enter\n", __FUNCTION__);
	gpib_unregister_driver(&agilent_82357a_gpib_interface);
	usb_deregister(&agilent_82357a_bus_driver);
//	printk("%s: exit\n", __FUNCTION__);
}

module_init(agilent_82357a_init_module);
module_exit(agilent_82357a_exit_module);

