/***************************************************************************
			  lib/ibSad.c
		     -------------------

    copyright		 : (C) 2001,2002,2003 by Frank Mori Hess
    email		 : fmhess@users.sourceforge.net
 ***************************************************************************/

/***************************************************************************
 *									   *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
 *   (at your option) any later version.				   *
 *									   *
 ***************************************************************************/

#include "ib_internal.h"

int internal_ibsad( ibConf_t *conf, int address )
{
	ibBoard_t *board;
	sad_ioctl_t sad_cmd;
	int sad;
	int retval;

	board = interfaceBoard( conf );

	if (!address)
		sad = -1;
	else
		sad = address - sad_offset;

	if( sad < -1 || sad > gpib_sad_max )
	{
		setIberr( EARG );
		return -1;
	}

	sad_cmd.handle = conf->handle;
	sad_cmd.sad = sad;
	retval = ioctl( board->fileno, IBSAD, &sad_cmd );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: failed to change gpib secondary address\n" );
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	conf->settings.sad = sad;

	return 0;
}

int ibsad( int ud, int v )
{
	ibConf_t *conf;
	int retval;

	conf = enter_library( ud );
	if( conf == NULL )
		return exit_library( ud, 1 );

	retval = internal_ibsad( conf, v );
	if( retval < 0 )
	{
		return exit_library( ud, 1 );
	}

	return exit_library( ud, 0 );
}
