/***************************************************************************
 *                      lpvo_usb_gpib/lpvo_usb_gpib.c                      *
 *                           -------------------                           *
 *  This code has been developed at the Department of Physics (University  *
 *  of Florence, Italy) to support in linux-gpib the open usb-gpib adapter *
 *  implemented at the University of Ljubljana (lpvo.fe.uni-lj.si/gpib)    *
 *                                                                         *
 *  begin                : April 2011                                      *
 *  copyright            : (C) 2011 Marcello Carla'                        *
 *  email                : carla@fi.infn.it                                *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/



/* base module includes */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/file.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/sched/signal.h>
#include <asm/uaccess.h>

#include "gpibP.h"

MODULE_LICENSE("GPL");

#define NAME "lpvo_usb_gpib"
#define HERE NAME, (char *) __FUNCTION__

/* standard and extended command sets of the usb-gpib adapter */

#define USB_GPIB_ON      "\nIB\n"
#define USB_GPIB_OFF     "\nIBO\n"
#define USB_GPIB_IBm0    "\nIBm0\n"   /* do not assert REN with IFC */
#define USB_GPIB_IBm1    "\nIBm1\n"   /* assert REN with IFC */
#define USB_GPIB_IBCL    "\nIBZ\n"
#define USB_GPIB_STATUS  "\nIBS\n"
#define USB_GPIB_READ    "\nIB?\n"
#define USB_GPIB_READ_1  "\nIBB\n"
#define USB_GPIB_EOI     "\nIBe0\n"
#define USB_GPIB_FTMO    "\nIBf0\n"    /* disable first byte timeout */
#define USB_GPIB_tTMOz   "\nIBt0\n"    /* disable byte timeout */

/* incomplete commands */

#define USB_GPIB_tTMO    "\nIBt"      /* set byte timeout */
#define USB_GPIB_TTMO    "\nIBT"      /* set total timeout */

#define USB_GPIB_DEBUG_ON    "\nIBDE\xAA\n"
#define USB_GPIB_SET_LISTEN  "\nIBDT0\n"
#define USB_GPIB_SET_TALK    "\nIBDT1\n"
#define USB_GPIB_SET_LINES   "\nIBDC \n"
#define USB_GPIB_SET_DATA    "\nIBDM \n"
#define USB_GPIB_READ_LINES  "\nIBD?C\n"
#define USB_GPIB_READ_DATA   "\nIBD?M\n"
#define USB_GPIB_READ_BUS    "\nIBD??\n"

/* command sequences */

#define USB_GPIB_UNTALK "\nIBC_\n"
#define USB_GPIB_UNLISTEN "\nIBC?\n"

/* special characters used by the adapter */

#define DLE '\020'
#define STX '\02'
#define ETX '\03'
#define ACK '\06'
#define NODATA '\03'
#define NODAV '\011'

#define IB_BUS_REN  0x01
#define IB_BUS_IFC  0x02
#define IB_BUS_NDAC 0x04
#define IB_BUS_NRFD 0x08
#define IB_BUS_DAV  0x10
#define IB_BUS_EOI  0x20
#define IB_BUS_ATN  0x40
#define IB_BUS_SRQ  0x80

#define INBUF_SIZE 128

struct char_buf {               /* used by one_char() routine */
	char * inbuf;
	int last;
	int nchar;
};

typedef struct {                /* private data to the device */
	struct file * f;        /* the 'file' structure for the tty-usb line */
	uint8_t eos;            /* eos character */
	short eos_flags;        /* eos mode */
	struct timespec before  ;  /* time value for timings */
        int timeout;            /* current value for timeout */
} usb_gpib_private_t;

/*
 *    ***  The following code is for diagnostics and debug  ***
 */

/*
 * DIA_LOG - if ENABLE_DIA_LOG is set to 1  *** a lot of ***  diagnostic
 *           messages are generated to syslog, describing whatever is
 *           going on.
 *
 */

#define ENABLE_DIA_LOG 0

#if ENABLE_DIA_LOG
#define DIA_LOG(format,...) \
	(printk (KERN_ALERT "%s:%s - "format, HERE, __VA_ARGS__))
#else
#define DIA_LOG(format,...)
#endif

#define SHOW_STATUS(board) {				\
		DIA_LOG("# - board %p\n", board);			\
		DIA_LOG("# - buffer_length %d\n", board->buffer_length); \
		DIA_LOG("# - status %lx\n", board->status);		\
		DIA_LOG("# - use_count %d\n", board->use_count);	\
		DIA_LOG("# - pad %x\n", board->pad);			\
		DIA_LOG("# - sad %x\n", board->sad);			\
		DIA_LOG("# - timeout %d\n", board->usec_timeout);	\
		DIA_LOG("# - ppc %d\n", board->parallel_poll_configuration); \
		DIA_LOG("# - t1delay %d\n", board->t1_nano_sec);	\
		DIA_LOG("# - online %d\n", board->online);		\
		DIA_LOG("# - autopoll %d\n", board->autospollers);	\
		DIA_LOG("# - autopoll task %p\n", board->autospoll_task); \
		DIA_LOG("# - minor %d\n", board->minor);		\
		DIA_LOG("# - master %d\n", board->master);		\
		DIA_LOG("# - list %d\n", board->ist);			\
	}
/*
  n = 0;
  list_for_each (l, &board->device_list) n++;
  TTY_LOG ("%s:%s - devices in list %d\n", a, b, n);
*/

/*
 * TTY_LOG - write a message to the current work terminal (if any)
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
#define TTY_LOG(format,...) {						\
		char buf[128];	                                        \
                struct tty_struct *tty=get_current_tty();		\
		if (tty) {						\
		  snprintf (buf, 128, format, __VA_ARGS__);		\
		  tty->driver->write (tty, buf, strlen(buf));	\
		  tty->driver->write (tty, "\r", 1);		\
		}							\
	}
#else
#define TTY_LOG(format,...) {						\
		char buf[128];	                                        \
                struct tty_struct *tty=get_current_tty();		\
		if (tty) {						\
		  snprintf (buf, 128, format, __VA_ARGS__);		\
		  tty->driver->ops->write (tty, buf, strlen(buf));	\
		  tty->driver->ops->write (tty, "\r", 1);		\
		}							\
	}
#endif

/*
 *   usec_diff : take difference in MICROsec between two 'timespec'
 *               (unix time in sec and NANOsec)
 */

inline int usec_diff (struct timespec * a, struct timespec * b) {
	return ((a->tv_sec - b->tv_sec)*1000000 +
		(a->tv_nsec - b->tv_nsec)/1000);
}

/*
 *   ***  these routines are specific to the usb-gpib adapter  ***
 */

/**
 * write_loop() - Send a byte sequence to the adapter
 *
 * @f:        the 'file' structure for the port
 * @msg:      the byte sequence.
 * @leng:     the byte sequence length.
 *
 */

int write_loop (struct file *f, char * msg, int leng) {
	int nchar = 0, val;

	do {
		val = f->f_op->write (f, msg+nchar, leng-nchar, &f->f_pos);
		if (val < 1) {
                        printk (KERN_ALERT "%s:%s - write error: %d %d/%d\n",
                                HERE, val, nchar, leng);
			return -EIO;
		}
                nchar +=val;
	} while (nchar < leng);
	return leng;
}

/**
 * send_command() - Send a byte sequence and return a single byte reply.
 *
 * @board:    the gpib_board_struct data area for this gpib interface
 * @msg:      the byte sequence.
 * @leng      the byte sequence length; can be given as zero and is
 *            computed automatically, but if 'msg' contains a zero byte,
 *            it has to be given explicitly.
 */

int send_command (gpib_board_t *board, char * msg, int leng) {

	mm_segment_t oldfs;
	char buffer[1]={0};
	int nchar=0;
	struct file *f = ((usb_gpib_private_t *)board->private_data)->f;

	if (!leng) leng = strlen(msg);

	oldfs = get_fs();
	set_fs (KERNEL_DS);

	if (write_loop (f, msg, leng) == -EIO) {
		set_fs (oldfs);
		return -EIO;
	};

	nchar = f->f_op->read (f, buffer, 1, &f->f_pos);
	set_fs (oldfs);

	if (nchar != 1) {
		printk (KERN_ALERT "%s:%s - read error: %x %x\n",
			HERE, nchar, buffer[0]);
		return -EIO;
	}

	return buffer[0] & 0xff;
}

/**
*
* set_control_line() - Set the value of a single gpib control line
*
* @board:    the gpib_board_struct data area for this gpib interface
* @line:     line mask
* @value:    line new value (0/1)
*
*/

int set_control_line (gpib_board_t *board, int line, int value) {

	char msg[]=USB_GPIB_SET_LINES;
	int retval;
	int leng=strlen(msg);

	DIA_LOG ("setting line %x to %x\n", line, value);

	retval = send_command (board, USB_GPIB_READ_LINES, 0);

	DIA_LOG ("old line values: %x\n", retval);

	if (retval == -EIO) return retval;

	msg[leng-2] = value ? (retval & ~line): retval | line;

	retval = send_command (board, msg, 0);

	DIA_LOG ("operation result: %x\n", retval);

	return retval;
}

/**
* one_char() - read one single byte from input buffer
*
* @board:      the gpib_board_struct data area for this gpib interface
* @char_buf:   the routine private data structure
*/

static int one_char(gpib_board_t *board, struct char_buf * b) {

	struct timespec before, after;
	struct file *f = ((usb_gpib_private_t *)board->private_data)->f;

	if (b->nchar) {
		DIA_LOG ("-> %x\n", b->inbuf[b->last - b->nchar]);
		return b->inbuf[b->last - b->nchar--];
	}
	getnstimeofday (&before);
	b->last = b->nchar =
		f->f_op->read (f, b->inbuf, INBUF_SIZE, &f->f_pos);
	getnstimeofday (&after);

	DIA_LOG ("read %d bytes in %d usec\n",
		b->nchar, usec_diff(&after, &before));

	if (b->nchar > 0) {
		DIA_LOG ("-> %x\n", b->inbuf[b->last - b->nchar]);
		return b->inbuf[b->last - b->nchar--];
	} else if (b->nchar == 0) {
		printk (KERN_ALERT "%s:%s - read returned EOF\n", HERE);
                return -EIO;
	} else {
		printk (KERN_ALERT "%s:%s - read error %d\n", HERE, b->nchar);
		TTY_LOG ("\n *** %s *** Read Error - %s\n", NAME,
			 "Reset the adapter with 'gpib_config'\n");
                return -EIO;
        }
}

/**
 * set_timeout() - set single byte / total timeouts on the adapter
 *
 * @board:    the gpib_board_struct data area for this gpib interface
 *
 *         For sake of speed, the operation is performed only if it
 *         modifies the current (saved) value. Minimum allowed timeout
 *         is 30 ms (T30ms -> 8); timeout disable (TNONE -> 0) currently
 *         not supported.
 */

void set_timeout (gpib_board_t *board) {

        int n, val;
	char command[sizeof(USB_GPIB_TTMO)+6];
        usb_gpib_private_t * data = board->private_data;

        if (data->timeout == board->usec_timeout) return;

	n = (board->usec_timeout + 32767) / 32768;
	if (n < 2) n = 2;

	DIA_LOG ("Set timeout to %d us -> %d\n", board->usec_timeout, n);

        sprintf (command, "%s%d\n", USB_GPIB_tTMO, n > 255 ? 255 : n);
        val = send_command (board, command, 0);

        if (val == ACK) {
                        if (n > 65535) n = 65535;
                        sprintf (command, "%s%d\n", USB_GPIB_TTMO, n);
                        val = send_command (board, command, 0);
        }

        if (val != ACK)
		printk (KERN_ALERT "%s:%s - error in timeout set: <%s>\n",
                        HERE, command);
        else {
                data->timeout = board->usec_timeout;
        }
}

/*
 *    now the standard interface functions - attach and detach
 */

/**
 * usb_gpib_attach() - activate the usb-gpib converter board
 *
 * @board:    the gpib_board_struct data area for this gpib interface
 * @config:   firmware data, if any (from gpib_config -I <file>)
 *
 * The channel name is ttyUSBn, with n=0 by default. Other values for n
 * passed with gpib_config -b <n>.
 *
 * In this routine I trust that when an error code is returned
 * detach() will be called. Always.
 */

int usb_gpib_attach(gpib_board_t *board, const gpib_board_config_t *config) {

	int retval;
	char device[]="/dev/ttyUSBxx";
	int base = (long int) config->ibbase;
	struct file *f;
	mm_segment_t oldfs;

	printk (KERN_ALERT "%s:%s configuring %s#%d as ttyUSB%d\n", HERE,
		board->interface->name, board->minor, base);

	board->private_data = kzalloc(sizeof(usb_gpib_private_t), GFP_KERNEL);
	if (board->private_data == NULL) return -ENOMEM;

	if (base > 99) return -EIO;
	snprintf (device, sizeof(device), "/dev/ttyUSB%d", base<0 ? 0 : base);

	oldfs = get_fs();
	set_fs (KERNEL_DS);

	f = filp_open (device, O_RDWR | O_SYNC, 0777);
	((usb_gpib_private_t *)board->private_data)->f = f;

	set_fs (oldfs);

	DIA_LOG ("found %p %ld\n", f, IS_ERR(f));

	if (IS_ERR(f)) {
		TTY_LOG ("%s:%s - %s is not a valid usb->gpib adapter.\n",
			HERE, device);
		printk (KERN_ALERT "%s:%s - no device found\n", HERE);
		kfree (board->private_data);
		board->private_data = NULL;
		return -ENODEV;
	}

	f->f_pos = 0;

	SHOW_STATUS (board);

	retval = send_command (board, USB_GPIB_ON, 0);
	DIA_LOG ("USB_GPIB_ON returns %x\n", retval);
	if (retval != ACK) return -EIO;

	/* We must setup debug mode because we need the extended instruction
	set to cope with the Core (gpib_common) point of view */

	retval = send_command (board, USB_GPIB_DEBUG_ON, 0);
	DIA_LOG ("USB_GPIB_DEBUG_ON returns %x\n", retval);
	if (retval != ACK) return -EIO;

	/* We must keep REN off after an IFC because so it is
	assumed by the Core */

	retval = send_command (board, USB_GPIB_IBm0, 0);
	DIA_LOG ("USB_GPIB_IBm0 returns %x\n", retval);
	if (retval != ACK) return -EIO;

	retval = set_control_line (board, IB_BUS_REN, 0);
	if (retval != ACK) return -EIO;

        retval = send_command (board, USB_GPIB_FTMO, 0);
	DIA_LOG ("USB_GPIB_FTMO returns %x\n", retval);
	if (retval != ACK) return -EIO;

	SHOW_STATUS(board);
	TTY_LOG ("Module '%s' has been succesfully configured\n", NAME);
	return 0;
}

/**
 * usb_gpib_detach() - deactivate the usb-gpib converter board
 *
 * @board:    the gpib_board data area for this gpib interface
 *
 */

void usb_gpib_detach(gpib_board_t *board) {

	mm_segment_t oldfs;
	struct file *f;

	SHOW_STATUS(board);

	if (board->private_data) {
		f = ((usb_gpib_private_t *)board->private_data)->f;
		if (f) {
			oldfs = get_fs();
			set_fs (KERNEL_DS);

			write_loop (f, USB_GPIB_OFF, strlen(USB_GPIB_OFF));
			msleep(100);
			printk (KERN_NOTICE "%s:%s - GPIB off\n", HERE);
			filp_close(f, 0);

			set_fs (oldfs);

			printk (KERN_NOTICE "%s:%s - ttyUSB off\n",
				HERE);
		}

		if (board->private_data) kfree (board->private_data);
	}

	DIA_LOG ("done %p\n", board);
	TTY_LOG ("Module '%s' has been detached\n", NAME);
}


/*
 *   Other functions follow in alphabetical order
 */

/* command */

int usb_gpib_command(gpib_board_t *board,
			uint8_t *buffer,
		size_t length,
		size_t *bytes_written) {

	int i, retval;
	char command[6]="IBc \n";

	DIA_LOG ("enter %p\n", board);

        set_timeout (board);

	for ( i=0 ; i<length ; i++ ) {
		command[3] = buffer[i];
		retval = send_command (board, command, 5);
		DIA_LOG ("%d %x %x\n", i, buffer[i], retval);
		if (retval != 0x06) return -EIO;
		++(*bytes_written);
	}
	return 0;
}

/**
 * disable_eos() - Disable END on eos byte (END on EOI only)
 *
 * @board:    the gpib_board data area for this gpib interface
 *
 *   With the lpvo adapter eos can only be handled via software.
 *   Cannot do nothing here, but remember for future use.
 */

void usb_gpib_disable_eos(gpib_board_t *board) {

	((usb_gpib_private_t *)board->private_data)->eos_flags &= ~REOS;
	DIA_LOG ("done: %x\n",
		((usb_gpib_private_t *)board->private_data)->eos_flags);
}

/**
 * enable_eos() - Enable END for reads when eos byte is received.
 *
 * @board:    the gpib_board data area for this gpib interface
 * @eos_byte: the 'eos' byte
 * @compare_8_bits: if zero ignore eigthth bit when comparing
 *
 */

int usb_gpib_enable_eos(gpib_board_t *board,
			uint8_t eos_byte,
			int compare_8_bits) {

	usb_gpib_private_t * pd = (usb_gpib_private_t *)board->private_data;

	DIA_LOG ("enter with %x\n", eos_byte);
	pd->eos = eos_byte;
	pd->eos_flags = REOS;
	if( compare_8_bits ) pd->eos_flags |= BIN;
	return 0;
}

/**
 * go_to_standby() - De-assert ATN
 *
 * @board:    the gpib_board data area for this gpib interface
 */

int usb_gpib_go_to_standby(gpib_board_t *board) {

	int retval = set_control_line (board, IB_BUS_ATN, 0);

	DIA_LOG ("done with %x\n", retval);

	if (retval == ACK) return 0;
	return -EIO;
}

/**
 * interface_clear() - Assert or de-assert IFC
 *
 * @board:    the gpib_board data area for this gpib interface
 * assert:    1: assert IFC;  0: de-assert IFC
 *
 *    Currently on the assert request we issue the lpvo IBZ
 *    command that cycles IFC low for 100 usec, then we ignore
 *    the de-assert request.
 */

void usb_gpib_interface_clear(gpib_board_t *board, int assert) {

	int retval=0;

	DIA_LOG ("enter with %d\n", assert);

	if (assert) {
		retval = send_command (board, USB_GPIB_IBCL, 0);

		smp_mb__before_atomic();
		set_bit(CIC_NUM, &board->status);
		smp_mb__after_atomic();
	}

	DIA_LOG ("done with %d %d\n", assert, retval);
	return;
}

/**
 * line_status() - Read the status of the bus lines.
 *
 *  @board:    the gpib_board data area for this gpib interface
 *
 *    We can read all lines.
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,13,0)
#define WQT wait_queue_t
#define WQH task_list
#define WQE task_list
#else       
#define WQT wait_queue_entry_t
#define WQH head
#define WQE entry
#endif

int usb_gpib_line_status (const gpib_board_t *board ) {

	int buffer;
	int line_status = ValidALL;   /* all lines will be read */
	struct list_head *p, *q;
	WQT * item;
        unsigned long flags;
        int sleep=0;

        /* if we are on the wait queue (board->wait), do not hurry
           reading status line; instead, pause a little */

        spin_lock_irqsave ((spinlock_t *) &(board->wait.lock), flags);
	q = (struct list_head *) &(board->wait.WQH);
        list_for_each(p, q) {
		item = container_of(p, WQT, WQE);
                if (item->private == current) {
                        sleep = 20;
                        break;
                }
                /* pid is: ((struct task_struct *) item->private)->pid); */
	}
        spin_unlock_irqrestore ((spinlock_t *) &(board->wait.lock), flags);
        if (sleep) {
                DIA_LOG ("we are on the wait queue - sleep %d ms\n", sleep);
                msleep(sleep);
        }

	buffer = send_command ((gpib_board_t *)board, USB_GPIB_STATUS, 0);

	if (buffer == -EIO) {
		printk (KERN_ALERT "%s:%s - line status read failed\n", HERE);
		return 0;
	}

	if((buffer & 0x01) == 0) line_status |= BusREN;
	if((buffer & 0x02) == 0) line_status |= BusIFC;
	if((buffer & 0x04) == 0) line_status |= BusNDAC;
	if((buffer & 0x08) == 0) line_status |= BusNRFD;
	if((buffer & 0x10) == 0) line_status |= BusDAV;
	if((buffer & 0x20) == 0) line_status |= BusEOI;
	if((buffer & 0x40) == 0) line_status |= BusATN;
	if((buffer & 0x80) == 0) line_status |= BusSRQ;

	DIA_LOG ("done with %x %x\n", buffer, line_status);

	return line_status;
}

/* parallel_poll */

int usb_gpib_parallel_poll(gpib_board_t *board, uint8_t *result) {

	/* request parallel poll asserting ATN | EOI;
	we suppose ATN already asserted */

	int retval;

	DIA_LOG ("enter %p\n", board);

	retval = set_control_line (board, IB_BUS_EOI, 1);
	if (retval != ACK) {
		printk (KERN_ALERT "%s:%s - assert EOI failed\n", HERE);
		return -EIO;
	}

	*result = send_command (board, USB_GPIB_READ_DATA, 0);

	DIA_LOG ("done with %x\n", *result);

	retval = set_control_line (board, IB_BUS_EOI, 0);
	if (retval != 0x06) {
		printk (KERN_ALERT "%s:%s - unassert EOI failed\n", HERE);
		return -EIO;
	}

	return 0;
}

/* read */

static int usb_gpib_read (gpib_board_t *board,
			uint8_t *buffer,
			size_t length,
			int *end,
			size_t *bytes_read) {

#define MAX_READ_EXCESS 16384

	struct char_buf b={NULL,0};

	int retval;
	mm_segment_t oldfs;
	char c;
	struct timespec before, after;
	int read_count = MAX_READ_EXCESS;
	usb_gpib_private_t * pd = (usb_gpib_private_t *)board->private_data;

	DIA_LOG ("enter %p\n", board);

	*bytes_read = 0;      /* by default, things go wrong */
	*end = 0;

	set_timeout (board);

	/* single byte read has a special handling */

	if (length == 1) {

		char inbuf[2]={0,0};

		/* read a single character */

		oldfs = get_fs();
		set_fs (KERNEL_DS);

		getnstimeofday (&before);

		if (write_loop (pd->f, USB_GPIB_READ_1,
				strlen(USB_GPIB_READ_1)) == -EIO) {
			set_fs (oldfs);
			return -EIO;
		}

		retval = pd->f->f_op->read (pd->f, inbuf, 1,
					&pd->f->f_pos);
		retval += pd->f->f_op->read (pd->f, inbuf+1, 1,
					&pd->f->f_pos);
		getnstimeofday (&after);

		set_fs (oldfs);

		DIA_LOG ("single read: %x %x %x in %d\n", retval,
			inbuf[0], inbuf[1],
			usec_diff (&after, &before));

		/* good char / last char? */

		if (retval == 2 && inbuf[1] == ACK) {
			buffer[0] = inbuf[0];
			*bytes_read = 1;
			return 0;
		}
		if (retval < 2) return -EIO;
		else return -ETIME;
		return 0;
	}

	/* allocate buffer for multibyte read */

	b.inbuf = kmalloc(INBUF_SIZE, GFP_KERNEL);
	if (!b.inbuf) return -ENOMEM;

	/* send read command and check <DLE><STX> sequence */

	oldfs = get_fs();
	set_fs (KERNEL_DS);

	if (write_loop (pd->f, USB_GPIB_READ, strlen(USB_GPIB_READ)) == -EIO) {
		retval = -EIO;
		goto read_return;
	}

	if (one_char(board, &b) != DLE || one_char(board, &b) != STX) {

		printk (KERN_ALERT "%s:%s - wrong <DLE><STX> sequence\n",
			HERE);
		retval = -EIO;
		goto read_return;
	}

	/* get data flow */

	while (1) {
		if ((c = one_char(board, &b)) == -EIO) {
			retval = -EIO;
			goto read_return;
		}


		if (c != DLE || (c = one_char(board, &b)) == DLE) {

			/* data byte - store into buffer */

			if (*bytes_read == length) break; /* data overflow */
			buffer[(*bytes_read)++] = c;
			if (c == pd->eos) {
				*end = 1;
				break;
			}

		} else {

			/* we are in the closing <DLE><ETX> sequence */

			if (c == ETX) {
				c = one_char(board, &b);
				if (c == ACK) {
					*end = 1;
					retval = 0;
					goto read_return;
				} else {
					printk (KERN_ALERT "%s:%s - %s %x\n",
						HERE,
						"Wrong end of message", c);
					retval = -ETIME;
					goto read_return;
				}
			} else {
				printk (KERN_ALERT "%s:%s - %s\n", HERE,
					"lone <DLE> in stream");
				retval = -EIO;
				goto read_return;
			}
		}
	}

	/* we had a data overflow - flush excess data */

	while (read_count--) {
		if (one_char(board, &b) != DLE) continue;
		c = one_char(board, &b);
		if (c == DLE) continue;
		if (c == ETX) {
			c = one_char(board, &b);
			if (c == ACK) {
				if (MAX_READ_EXCESS - read_count > 1)
					printk (KERN_ALERT "%s:%s - %s\n", HERE,
                                        	"small buffer - maybe some data lost");
                                retval = 0;
				goto read_return;
			}
			break;
		}
	}

	printk (KERN_ALERT "%s:%s - no input end - GPIB board in odd state\n",
		HERE);
	retval = -EIO;

read_return:
	set_fs (oldfs);
	kfree (b.inbuf);

	DIA_LOG("done with byte/status: %d %x %d\n",
                (int) *bytes_read, retval, *end);

        if (retval == 0 || retval == -ETIME) {
                if (send_command(board,
                                 USB_GPIB_UNTALK, sizeof(USB_GPIB_UNTALK)
                                 == 0x06)) return retval;
                return  -EIO;
        }

	return retval;
}

/* remote_enable */

void usb_gpib_remote_enable(gpib_board_t *board, int enable) {

	int retval;

	retval = set_control_line (board, IB_BUS_REN, enable ? 1 : 0);
	if (retval != ACK)
		printk (KERN_ALERT "%s:%s - could not set REN line: %x\n",
			HERE, retval);

	DIA_LOG ("done with %x\n", retval);
}

/* request_system_control */

void usb_gpib_request_system_control(gpib_board_t *board,
				int request_control ) {

	smp_mb__before_atomic();
	if (request_control) {
		set_bit(CIC_NUM, &board->status);
	} else {
		clear_bit(CIC_NUM, &board->status);
	}
	smp_mb__after_atomic();

	DIA_LOG ("done with %d -> %lx\n", request_control, board->status);

	return;
}

/* take_control */
/* beware: the sync flag is ignored; what is its real meaning? */

int usb_gpib_take_control(gpib_board_t *board, int sync) {

	int retval;

	retval = set_control_line (board, IB_BUS_ATN, 1);

	DIA_LOG ("done with %d %x\n", sync, retval);

	if (retval == ACK) return 0;
	return -EIO;
}

/* update_status */

unsigned int usb_gpib_update_status( gpib_board_t *board,
				unsigned int clear_mask ) {

	/* There is nothing we can do here, I guess */

	board->status &= ~clear_mask;

	DIA_LOG ("done with %x %lx\n", clear_mask, board->status);

	return board->status;
}

/* write */
/* beware: DLE characters are not escaped - can only send ASCII data */

static int usb_gpib_write(gpib_board_t *board,
			uint8_t *buffer,
			size_t length,
			int send_eoi,
			size_t *bytes_written) {

	int retval;
	char * msg;

	DIA_LOG ("enter %p\n", board);

	set_timeout (board);

	msg = kmalloc(length+8, GFP_KERNEL);
	if (!msg) return -ENOMEM;

	memcpy (msg, "\nIB\020\002",5);
	memcpy (msg+5, buffer, length);
	memcpy (msg+5+length, "\020\003\n",3);

	retval = send_command (board, msg, length+8);
	kfree (msg);

	DIA_LOG ("<%.*s> -> %x\n", (int) length, buffer, retval);

	if (retval != ACK) return -EPIPE;

	*bytes_written = length;

        if (send_command(board, USB_GPIB_UNLISTEN, sizeof(USB_GPIB_UNLISTEN))
            != 0x06) return  -EPIPE;

	return length;
}

/*
 *  ***  following functions not implemented yet  ***
 */

/* parallel_poll configure */

void usb_gpib_parallel_poll_configure( gpib_board_t *board,
					uint8_t configuration ) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
}

/* parallel_poll_response */

void usb_gpib_parallel_poll_response( gpib_board_t *board, int ist ) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
}

/* primary_address */

void usb_gpib_primary_address(gpib_board_t *board, unsigned int address) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
}

/* return_to_local */

void usb_gpib_return_to_local( gpib_board_t *board ) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
}

/* secondary_address */

void usb_gpib_secondary_address(gpib_board_t *board,
				unsigned int address,
				int enable) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
}

/* serial_poll_response */

void usb_gpib_serial_poll_response(gpib_board_t *board, uint8_t status) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
}

/* serial_poll_status */

uint8_t usb_gpib_serial_poll_status( gpib_board_t *board ) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
	return 0;
}

/* t1_delay */

unsigned int usb_gpib_t1_delay( gpib_board_t *board, unsigned int nano_sec ) {

	printk (KERN_ALERT "%s:%s - currently a NOP\n", HERE);
	return 0;
}

/*
 *   ***  module dispatch table and init/exit functions  ***
 */

gpib_interface_t usb_gpib_interface =
{
	name: NAME,
	attach: usb_gpib_attach,
	detach: usb_gpib_detach,
	read: usb_gpib_read,
	write: usb_gpib_write,
	command: usb_gpib_command,
	take_control: usb_gpib_take_control,
	go_to_standby: usb_gpib_go_to_standby,
	request_system_control: usb_gpib_request_system_control,
	interface_clear: usb_gpib_interface_clear,
	remote_enable: usb_gpib_remote_enable,
	enable_eos: usb_gpib_enable_eos,
	disable_eos: usb_gpib_disable_eos,
	parallel_poll: usb_gpib_parallel_poll,
	parallel_poll_configure: usb_gpib_parallel_poll_configure,
	parallel_poll_response: usb_gpib_parallel_poll_response,
	local_parallel_poll_mode: NULL, // XXX
	line_status: usb_gpib_line_status,
	update_status: usb_gpib_update_status,
	primary_address: usb_gpib_primary_address,
	secondary_address: usb_gpib_secondary_address,
	serial_poll_response: usb_gpib_serial_poll_response,
	serial_poll_status: usb_gpib_serial_poll_status,
	t1_delay: usb_gpib_t1_delay,
	return_to_local: usb_gpib_return_to_local,
};

static int __init usb_gpib_init_module( void ) {

	gpib_register_driver(&usb_gpib_interface, THIS_MODULE);

	printk (KERN_ALERT "%s:%s - done\n", HERE);
	return 0;
}

static void __exit usb_gpib_exit_module( void ) {
	gpib_unregister_driver(&usb_gpib_interface);
}

module_init( usb_gpib_init_module );
module_exit( usb_gpib_exit_module );

