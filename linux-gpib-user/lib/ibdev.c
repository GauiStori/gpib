/***************************************************************************
                                 ibdev.c
                             -------------------
    begin                : Tues Feb 12 2002
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

#include "ib_internal.h"
#include <stdlib.h>

#if 0
static int is_device_addr( int minor, int pad, int sad )
{
	ibBoard_t *board;
	unsigned int board_pad;
	int board_sad;

	board = &ibBoard[ minor ];

	if( query_pad( board, &board_pad ) < 0 )
	{
		fprintf( stderr, "failed to query pad\n" );
		return -1;
	}
	if( query_sad( board, &board_sad ) < 0 )
	{
		fprintf( stderr, "failed to query sad\n" );
		return -1;
	}

	if( gpib_address_equal( board_pad, board_sad, pad, sad ) == 0 )
	{
		return 1;
	}

	return 0;
}
#endif

int ibdev( int minor, int pad, int sad, int timo, int eot, int eosmode )
{
	int retval,ud;
	ibConf_t new_conf;
	int no_show_error = getenv("IB_NO_ERROR") ? 1 : 0;
	ibBoard_t *board;
	unsigned  board_pad;
	int my_sad, board_sad;

	retval = ibParseConfigFile();
	if(retval < 0)
	{
		if (errno) {
			setIberr( EDVR );
			setIbcnt( errno );
		} else {
			setIberr( ECNF );
		}
		setIbsta( ERR );
		sync_globals();
		return -1;
	}

	/*  Check for valid address arguments */

	if (pad < 0 || pad > gpib_addr_max) {
		if (!no_show_error)
			fprintf(stderr,"ibdev: invalid pad %d, expected 0 <= pad <= 30\n", pad);
		setIberr( EARG );
		sync_globals();
		return -1;
	}

	if (!sad) /* 0 means no address */
		my_sad = -1;
	else
		my_sad = sad - sad_offset;

	if (my_sad < -1 || my_sad > gpib_sad_max) {
		if (!no_show_error)
			fprintf(stderr,"ibdev: invalid sad 0x%02x, expected 0, or 0x60 <= sad <= 0x7f\n", sad);
		setIberr( EARG );
		sync_globals();
		return -1;
	}

	init_ibconf( &new_conf );
	new_conf.settings.pad = pad;
	new_conf.settings.sad = my_sad;                     /* device address                   */
	new_conf.settings.board = minor;                    /* board number                     */
	new_conf.settings.eos = eosmode & 0xff;             /* local eos modes                  */
	new_conf.settings.eos_flags = eosmode & 0xff00;
	new_conf.settings.usec_timeout = timeout_to_usec( timo );
	if( eot )
		new_conf.settings.send_eoi = 1;
	else
		new_conf.settings.send_eoi = 0;
	new_conf.defaults = new_conf.settings;
	new_conf.is_interface = 0;
	new_conf.error_msg_disable = no_show_error;

	ud = my_ibdev( new_conf );

	if (ud < 0)
		return -1;

// check for address conflicts with board addresses

	board = interfaceBoard(&new_conf);

	if( query_pad( board, &board_pad ) < 0 ) {
		goto ibdev_err;
	}
	if( query_sad( board, &board_sad ) < 0 ) {
		goto ibdev_err;
	}

	if (board_pad == pad) {
		fprintf(stderr,"ibdev: address conflict with board pad\n");
		setIberr( EARG );
		goto ibdev_err;
	}

	return ud;
ibdev_err:
	retval = close_gpib_handle(&new_conf);
	if( retval < 0 ) {
		if (!no_show_error)
			fprintf( stderr, "ibdev: cleanup failed\n" );
		sync_globals();
		return -1;
	}
	release_descriptor( ud );
	sync_globals();
	return -1;
}

int my_ibdev( ibConf_t new_conf )
{
	int ud;
	ibConf_t *conf;

	ud = insert_descriptor(new_conf, -1);
	if( ud < 0 )
	{
		if (!new_conf.error_msg_disable)
			fprintf( stderr, "libgpib: ibdev failed to get descriptor\n" );
		setIbsta( ERR );
		return -1;
	}

	conf = enter_library( ud );
	if( conf == NULL )
	{
		exit_library( ud, 1 );
		release_descriptor( ud );
		return -1;
	}
	// XXX do local lockout if appropriate

	exit_library( ud, 0 );
	return ud;
}
