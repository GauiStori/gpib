/***************************************************************************
                              sys/ibcmd.c
                             -------------------

    copyright            : (C) 2001, 2002 by Frank Mori Hess
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

#include "gpibP.h"
#include <linux/delay.h>

/* After ATN is asserted, it should cause any connected devices
 * to start listening for command bytes and leave acceptor idle state.
 * So if ATN is asserted and neither NDAC or NRFD are asserted,
 * then there are no devices and ibcmd should error out immediately.
 * Some gpib hardware sees itself asserting NDAC/NRFD when it
 * is controller in charge, in which case this check will
 * do nothing useful (but shouldn't cause any harm either).
 * Drivers that don't need this check (ni_usb for example) may
 * set the skip_check_for_command_acceptors flag in their
 * gpib_interface_struct to avoid useless overhead.
 */
static int check_for_command_acceptors (gpib_board_t *board)
{
	int lines;
	
	if (board->interface->skip_check_for_command_acceptors) return 0;
	if (board->interface->line_status == NULL) return 0;
	
	udelay(2); // allow time for devices to respond to ATN if it was just asserted
	
	lines = board->interface->line_status(board);
	if (lines < 0) return lines;
	
	if (lines & ValidATN)
	{
		if ((lines & BusATN) == 0)
		{
			printk("gpib: ATN not asserted in check_for_command_acceptors?");
			return 0;
		}
	}
	
	if ((lines & ValidNRFD) &&
		(lines & ValidNDAC))
	{
		if ((lines & BusNRFD) == 0 &&
			(lines & BusNDAC) == 0)
		{
			return -ENOTCONN;
		}
	}

	return 0;
}

/*
 * IBCMD
 * Write cnt command bytes from buf to the GPIB.  The
 * command operation terminates only on I/O complete.
 *
 * NOTE:
 *      1.  Prior to beginning the command, the interface is
 *          placed in the controller active state.
 *      2.  Before calling ibcmd for the first time, ibsic
 *          must be called to initialize the GPIB and enable
 *          the interface to leave the controller idle state.
 */
int ibcmd( gpib_board_t *board, uint8_t *buf, size_t length, size_t *bytes_written )
{
	ssize_t ret = 0;
	int status;

	*bytes_written = 0;

	status = ibstatus( board );

	if((status & CIC) == 0)
	{
		printk("gpib: cannot send command when not controller-in-charge\n");
		return -EIO;
	}

	osStartTimer( board, board->usec_timeout );

	ret = ibcac( board, 1, 1 );
	if( ret == 0 )
	{
		ret = check_for_command_acceptors(board);
		if( ret == 0 )
		{
			ret = board->interface->command(board, buf, length, bytes_written);
		}
	}

	osRemoveTimer(board);

	if( io_timed_out( board ) )
		ret = -ETIMEDOUT;

	return ret;
}


