/***************************************************************************
                              ostimer.c
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

#include "ibsys.h"

/*
 * Timer functions
 */

/* Watchdog timeout routine */

void watchdog_timeout( COMPAT_TIMER_ARG_TYPE t )
{
	gpib_board_t *board = COMPAT_FROM_TIMER(board, t, timer);
	smp_mb__before_atomic();
	set_bit( TIMO_NUM, &board->status );
	smp_mb__after_atomic();

	wake_up_interruptible( &board->wait );
}

/* install timer interrupt handler */
void osStartTimer( gpib_board_t *board, unsigned int usec_timeout )
/* Starts the timeout task  */
{
	if( timer_pending( &board->timer ) )
	{
		printk("gpib: bug! timer already running?\n");
		return;
	}
	smp_mb__before_atomic();
	clear_bit( TIMO_NUM, &board->status );
	smp_mb__after_atomic();

	if( usec_timeout > 0 )
	{
		board->timer.function = watchdog_timeout;
		mod_timer(&board->timer, jiffies + usec_to_jiffies( usec_timeout ));   /* set number of ticks */
	}
}

void osRemoveTimer( gpib_board_t *board )
/* Removes the timeout task */
{
	if( timer_pending( &board->timer ) )
		del_timer_sync( &board->timer );
}

int io_timed_out( gpib_board_t *board )
{
	if( test_bit( TIMO_NUM, &board->status ) ) return 1;
	return 0;
}

