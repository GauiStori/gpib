/***************************************************************************
                          lib/ibSad.c
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

int internal_ibsad( ibConf_t *conf, int address )
{
	ibBoard_t *board;
	int sad = address - sad_offset;
	int retval;

	board = interfaceBoard( conf );

	if( sad > 30 )
	{
		setIberr( EARG );
		return -1;
	}

	retval = gpibi_change_address( conf, conf->settings.pad, sad );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: failed to change gpib address\n" );
		return -1;
	}
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
