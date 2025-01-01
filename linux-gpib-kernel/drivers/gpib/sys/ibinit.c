/***************************************************************************
                               sys/ibinit.c
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
#include "autopoll.h"
#include <linux/kthread.h>
#include <linux/vmalloc.h>

static int autospoll_wait_should_wake_up(gpib_board_t *board)
{
	int retval;

	mutex_lock(&board->big_gpib_mutex);

	smp_mb__before_atomic();
	retval = board->master && board->autospollers > 0 &&
		!atomic_read(&board->stuck_srq) &&
		test_and_clear_bit(SRQI_NUM, &board->status);
	smp_mb__after_atomic();

	mutex_unlock(&board->big_gpib_mutex);
	return retval;
}

static int autospoll_thread(void *board_void)
{
	gpib_board_t *board = board_void;
	int retval = 0;

	GPIB_DPRINTK("entering autospoll thread\n" );

	while(1)
	{
		wait_event_interruptible(board->wait,
			kthread_should_stop() ||
			autospoll_wait_should_wake_up(board));
		GPIB_DPRINTK("autospoll wait satisfied\n" );
		if(kthread_should_stop()) break;

		mutex_lock(&board->big_gpib_mutex);
		/* make sure we are still good after we have
		 * lock */
		if(board->autospollers <= 0 || board->master == 0)
		{
			mutex_unlock(&board->big_gpib_mutex);
			continue;
		}
		mutex_unlock(&board->big_gpib_mutex);

		if(try_module_get(board->provider_module))
		{
			retval = autopoll_all_devices(board);
			module_put(board->provider_module);
		}else
			printk("gpib%i: %s: try_module_get() failed!\n", board->minor, __FUNCTION__);
		if(retval <= 0)
		{
			printk("gpib%i: %s: struck SRQ\n", board->minor, __FUNCTION__);

			smp_mb__before_atomic();
			atomic_set(&board->stuck_srq, 1);	// XXX could be better
			set_bit(SRQI_NUM, &board->status);
			smp_mb__after_atomic();
		}
	}
	printk("gpib%i: exiting autospoll thread\n", board->minor);
	return retval;
}

int ibonline(gpib_board_t *board)
{
	int retval;

	if( board->online ) return -EBUSY;
	if(board->interface == NULL) return -ENODEV;
	retval = gpib_allocate_board( board );
	if( retval < 0 ) return retval;

	board->dev = NULL;
	board->local_ppoll_mode = 0;
	retval = board->interface->attach(board, &board->config);
	if(retval < 0)
	{
		board->interface->detach(board);
		printk("gpib: interface attach failed\n");
		return retval;
	}
	/* nios2nommu on 2.6.11 uclinux kernel has weird problems
	with autospoll thread causing huge slowdowns */
#ifndef CONFIG_NIOS2
	board->autospoll_task = kthread_run(&autospoll_thread, board, "gpib%d_autospoll_kthread", board->minor);
	retval = IS_ERR(board->autospoll_task);
	if(retval)
	{
		printk("gpib: failed to create autospoll thread\n");
		board->interface->detach(board);
		return retval;
	}
#endif
	board->online = 1;
	GPIB_DPRINTK( "gpib: board online\n" );

	return 0;
}

/* XXX need to make sure board is generally not in use (grab board lock?) */
int iboffline( gpib_board_t *board )
{
	int retval;

	if( board->online == 0 )
	{
		return 0;
	}
	if(board->interface == NULL) return -ENODEV;

	if(board->autospoll_task != NULL && !IS_ERR(board->autospoll_task))
	{
		retval = kthread_stop(board->autospoll_task);
		if(retval)
			printk("gpib: kthread_stop returned %i\n", retval);
		board->autospoll_task = NULL;
	}

	board->interface->detach( board );
	gpib_deallocate_board( board );
	board->online = 0;
	GPIB_DPRINTK( "gpib: board offline\n" );

	return 0;
}

