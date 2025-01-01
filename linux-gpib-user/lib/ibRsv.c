/***************************************************************************
                          lib/ibRsv.c
                             -------------------

    copyright            : (C) 2001,2002,2003 by Frank Mori Hess
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

int internal_ibrsv2( ibConf_t *conf, int status_byte, int new_reason_for_service )
{
	ibBoard_t *board;
	request_service2_t cmd;
	int retval;
	const int MSS = status_byte & request_service_bit;
	
	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	if( (status_byte & 0xff) != status_byte)
	{
		setIberr( EARG );
		return -1;
	}
	
	board = interfaceBoard( conf );

	cmd.status_byte = status_byte;
	cmd.new_reason_for_service = new_reason_for_service;
	
	/* prefer using IBRSV if it is sufficient, since it is supported
	 * by older versions of the kernel modules. */
	if( MSS == 0 || ( MSS && new_reason_for_service ) )
	{
		retval = ioctl( board->fileno, IBRSV, &cmd.status_byte );
	}else
	{
		retval = ioctl( board->fileno, IBRSV2, &cmd );
	}
	if( retval < 0 )
	{
		if( errno == EOPNOTSUPP )
		{
			setIberr( ECAP );
		}else
		{
			setIberr( EDVR );
			setIbcnt( errno );
		}
		return retval;
	}

	return 0;
}

/* FIXME: NI's version returns old status byte in iberr on success.
 * Why that is at all useful, I do not know. */
int ibrsv( int ud, int status_byte )
{
	return ibrsv2( ud, status_byte, status_byte & request_service_bit);
}

int ibrsv2( int ud, int status_byte, int new_reason_for_service )
{
	ibConf_t *conf;
	int retval;

	conf = enter_library( ud );
	if( conf == NULL )
		return exit_library( ud, 1 );

	retval = internal_ibrsv2( conf, status_byte, new_reason_for_service );
	if( retval < 0 )
	{
		return exit_library( ud, 1 );
	}

	return exit_library( ud, 0 );
}

