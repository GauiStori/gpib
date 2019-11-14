/***************************************************************************
                                    nec7210/cmd.c
                             -------------------

    begin                : Dec 2001
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

#include "board.h"

int nec7210_command(gpib_board_t *board, nec7210_private_t *priv, uint8_t
 *buffer, size_t length, size_t *bytes_written)
{
	int retval = 0;
	unsigned long flags;

	*bytes_written = 0;

	smp_mb__before_atomic();
	clear_bit( BUS_ERROR_BN, &priv->state );
	smp_mb__after_atomic();

	while(*bytes_written < length)
	{
		if(wait_event_interruptible(board->wait, test_bit(COMMAND_READY_BN, &priv->state) ||
			test_bit( BUS_ERROR_BN, &priv->state ) || test_bit(TIMO_NUM, &board->status)))
		{
			printk("gpib command wait interrupted\n");
			retval = -ERESTARTSYS;
			break;
		}
		if(test_bit(TIMO_NUM, &board->status))
		{
			break;
		}
		if(test_and_clear_bit( BUS_ERROR_BN, &priv->state))
		{
			printk("nec7210: bus error on command byte\n");
			break;
		}

		spin_lock_irqsave( &board->spinlock, flags );
		clear_bit( COMMAND_READY_BN, &priv->state );
		write_byte( priv, buffer[*bytes_written], CDOR );
		spin_unlock_irqrestore( &board->spinlock, flags );

		++(*bytes_written);

		if(need_resched())
			schedule();
	}
	// wait for last byte to get sent
	if(wait_event_interruptible(board->wait, test_bit(COMMAND_READY_BN, &priv->state) ||
		test_bit( BUS_ERROR_BN, &priv->state ) || test_bit(TIMO_NUM, &board->status)))
	{
		printk("gpib command wait interrupted\n");
		retval = -ERESTARTSYS;
	}
	if(test_bit(TIMO_NUM, &board->status))
	{
		printk("gpib command timed out\n");
		retval = -ETIMEDOUT;
	}
	if(test_and_clear_bit( BUS_ERROR_BN, &priv->state))
	{
		printk("nec7210: bus error on command byte\n");
		retval = -EIO;
	}

	return retval;
}

EXPORT_SYMBOL(nec7210_command);








