/***************************************************************************
                                 slave_read_to_file.c
                             -------------------

Example program which uses gpib c library.  I use this with
master_write_from_file in order to test read/write speed between two boards.

    copyright            : (C) 2003 by Frank Mori Hess
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

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include "gpib/ib.h"
char *myProg;

void usage(int brief) {
	fprintf(stderr,"Usage: %s [-h] [-b <board index>] <file name>\n", myProg);
	if (brief) exit(1);
	fprintf(stderr,"  Default <board index> is 0\n");
	exit(0);
}

int main( int argc, char *argv[] )
{
	int board = 0;
	int eos_mode = 0;
	char *file_path;
	int status;
	int c;

	myProg = argv[0];
	while ((c = getopt (argc, argv, "b:h")) != -1)
	{
		switch (c)
		{
		case 'b': board = atoi(optarg); break;
		case 'h': usage(0); break;
		default:  usage(1);
		}
	}

	if (optind == argc)
	{
		fprintf( stderr, "Must provide file path as argument\n" );
		usage(0);
	}

	file_path = argv[ optind ];

	status = ibeos( board, eos_mode );
	if( status & ERR )
	{
		fprintf( stderr, "ibeos() failed\n" );
		fprintf( stderr, "%s\n", gpib_error_string( ThreadIberr() ) );
		return -1;
	}

	status = ibtmo( board, TNONE );
	if( status & ERR )
	{
		fprintf( stderr, "ibtmo() failed\n" );
		fprintf( stderr, "%s\n", gpib_error_string( ThreadIberr() ) );
		return -1;
	}

	status = ibwait( board, LACS );
	if( ( status & LACS ) == 0 )
	{
		fprintf( stderr, "ibwait() for LACS failed\n" );
		fprintf( stderr, "%s\n", gpib_error_string( ThreadIberr() ) );
		return -1;
	}

	status = ibrdf( board, file_path );
	if( status & ERR )
	{
		fprintf( stderr, "ibrdf() failed\n" );
		fprintf( stderr, "%s\n", gpib_error_string( ThreadIberr() ) );
		return -1;
	}

	return 0;
}

