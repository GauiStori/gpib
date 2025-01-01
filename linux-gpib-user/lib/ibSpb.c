/***************************************************************************
                          lib/ibSpb.c
                             -------------------

    copyright            : (C) 2017 by Dave Penkler
    email                : dpenkler@gmail.com
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

int internal_ibspb( ibConf_t *conf, short *sp_bytes )
{
	int retval;
	ibBoard_t *board;
	spoll_bytes_ioctl_t spb;
	if( conf->is_interface != 0 )
	{
		setIberr( EARG );
		return -1;
	}

	board = interfaceBoard( conf );

	spb.pad = conf->settings.pad;
	spb.sad = conf->settings.sad;

	retval = ioctl( board->fileno, IBSPOLL_BYTES, &spb );
	if( retval < 0 )
	{
		switch( errno )
		{
			default:
				setIbcnt( errno );
				setIberr( EDVR );
				break;
		}
		return -1;
	}
	*sp_bytes = spb.num_bytes;
	return 0;
}

int ibspb(int ud, short *sp_bytes )
{
	ibConf_t *conf;
	int retval;

	conf = general_enter_library( ud, 1, 1 );
	if( conf == NULL )
		return general_exit_library( ud, 1, 0, 0, 0, 0, 1 );

	retval = internal_ibspb( conf, sp_bytes );
	if( retval < 0 )
	{
		return general_exit_library( ud, 1, 0, 0, 0, 0, 1 );
	}

	return general_exit_library( ud, 0, 0, 0, 0, 0, 1 );
}
