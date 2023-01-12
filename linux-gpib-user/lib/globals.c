/***************************************************************************
                              lib/globals.c
                             -------------------

    begin                : Oct 2002
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
#include <pthread.h>
#include <stdlib.h>


volatile int iberr = 0;
volatile int ibsta = 0;
volatile int ibcnt = 0;
volatile long ibcntl = 0;

static pthread_key_t ibsta_key;
static pthread_key_t iberr_key;
static pthread_key_t ibcntl_key;

static pthread_key_t async_ibsta_key;
static pthread_key_t async_iberr_key;
static pthread_key_t async_ibcntl_key;

static pthread_once_t global_keys_once = PTHREAD_ONCE_INIT;

static void global_keys_alloc()
{
	int retval;

	retval = pthread_key_create( &ibsta_key, free );
	if( retval ) fprintf( stderr, "libgpib: failed to allocate TSD key!\n" );
	retval = pthread_key_create( &iberr_key, free );
	if( retval ) fprintf( stderr, "libgpib: failed to allocate TSD key!\n" );
	retval = pthread_key_create( &ibcntl_key, free );
	if( retval ) fprintf( stderr, "libgpib: failed to allocate TSD key!\n" );

	retval = pthread_key_create( &async_ibsta_key, free );
	if( retval ) fprintf( stderr, "libgpib: failed to allocate TSD key!\n" );
	retval = pthread_key_create( &async_iberr_key, free );
	if( retval ) fprintf( stderr, "libgpib: failed to allocate TSD key!\n" );
	retval = pthread_key_create( &async_ibcntl_key, free );
	if( retval ) fprintf( stderr, "libgpib: failed to allocate TSD key!\n" );
}

void globals_alloc( void )
{
	int *ibsta_p, *iberr_p, *ibcntl_p;
	int *async_ibsta_p, *async_iberr_p, *async_ibcntl_p;

	pthread_once( &global_keys_once, global_keys_alloc );
	if( pthread_getspecific( ibsta_key ) == NULL )
	{
		ibsta_p = malloc( sizeof( int ) );
		if( ibsta_p == NULL )
		fprintf( stderr, "libgpib: failed to allocate ibsta!\n" );
		iberr_p = malloc( sizeof( int ) );
		if( iberr_p == NULL )
		fprintf( stderr, "libgpib: failed to allocate iberr!\n" );
		ibcntl_p = malloc( sizeof( long ) );
		if( ibcntl_p == NULL )
		fprintf( stderr, "libgpib: failed to allocate ibcntl!\n" );
		async_ibsta_p = malloc( sizeof( int ) );
		if( async_ibsta_p == NULL )
		fprintf( stderr, "libgpib: failed to allocate async_ibsta!\n" );
		async_iberr_p = malloc( sizeof( int ) );
		if( async_iberr_p == NULL )
		fprintf( stderr, "libgpib: failed to allocate async_iberr!\n" );
		async_ibcntl_p = malloc( sizeof( long ) );
		if( async_ibcntl_p == NULL )
		fprintf( stderr, "libgpib: failed to allocate async_ibcntl!\n" );

		*ibsta_p = 0;
		*iberr_p = 0;
		*ibcntl_p = 0;
		*async_ibsta_p = 0;
		*async_iberr_p = 0;
		*async_ibcntl_p = 0;

		pthread_setspecific( ibsta_key, ibsta_p );
		pthread_setspecific( iberr_key, iberr_p );
		pthread_setspecific( ibcntl_key, ibcntl_p );
		pthread_setspecific( async_ibsta_key, async_ibsta_p );
		pthread_setspecific( async_iberr_key, async_iberr_p );
		pthread_setspecific( async_ibcntl_key, async_ibcntl_p );
	}
}

void setIbsta( int status )
{
	int *thread_ibsta;

	globals_alloc();
	thread_ibsta = pthread_getspecific( ibsta_key );
	if( thread_ibsta == NULL )
	{
		fprintf( stderr, "libgpib: failed to set ibsta TSD\n" );
		return;
	}
	*thread_ibsta = status;
}

void setIberr( int error )
{
	int *thread_iberr;

	globals_alloc();
	thread_iberr = pthread_getspecific( iberr_key );
	if( thread_iberr == NULL )
	{
		fprintf( stderr, "libgpib: failed to set iberr TSD\n" );
		return;
	}
	*thread_iberr = error;
}

void setIbcnt( long count )
{
	int *thread_ibcntl;

	globals_alloc();
	thread_ibcntl = pthread_getspecific( ibcntl_key );
	if( thread_ibcntl == NULL )
	{
		fprintf( stderr, "libgpib: failed to set ibcntl TSD\n" );
		return;
	}
	*thread_ibcntl = count;
}

void setAsyncIbsta( int status )
{
	int *async_thread_ibsta;

	globals_alloc();
	async_thread_ibsta = pthread_getspecific( async_ibsta_key );
	if( async_thread_ibsta == NULL )
	{
		fprintf( stderr, "libgpib: failed to set async_ibsta TSD\n" );
		return;
	}
	*async_thread_ibsta = status;
}

void setAsyncIberr( int error )
{
	int *async_thread_iberr;

	globals_alloc();
	async_thread_iberr = pthread_getspecific( async_iberr_key );
	if( async_thread_iberr == NULL )
	{
		fprintf( stderr, "libgpib: failed to set async_iberr TSD\n" );
		return;
	}
	*async_thread_iberr = error;
}

void setAsyncIbcnt( long count )
{
	int *async_thread_ibcntl;

	globals_alloc();
	async_thread_ibcntl = pthread_getspecific( async_ibcntl_key );
	if( async_thread_ibcntl == NULL )
	{
		fprintf( stderr, "libgpib: failed to set async_ibcntl TSD\n" );
		return;
	}
	*async_thread_ibcntl = count;
}

int ThreadIbsta( void )
{
	int *thread_ibsta;

	globals_alloc();

	thread_ibsta = pthread_getspecific( ibsta_key );
	if( thread_ibsta == NULL )
	{
		fprintf( stderr, "libgpib: failed to get ibsta TSD\n" );
		return -1;
	}

	return *thread_ibsta;
}

int ThreadIberr( void )
{
	int *thread_iberr;

	globals_alloc();

	thread_iberr = pthread_getspecific( iberr_key );
	if( thread_iberr == NULL )
	{
		fprintf( stderr, "libgpib: failed to get iberr TSD\n" );
		return -1;
	}

	return *thread_iberr;
}

int ThreadIbcnt( void )
{
	return ThreadIbcntl();
}

long ThreadIbcntl( void )
{
	int *thread_ibcntl;

	globals_alloc();
	thread_ibcntl = pthread_getspecific( ibcntl_key );
	if( thread_ibcntl == NULL )
	{
		fprintf( stderr, "libgpib: failed to get ibcntl TSD\n" );
		return -1;
	}

	return *thread_ibcntl;
}

int AsyncIbsta( void )
{
	int *async_thread_ibsta;

	globals_alloc();

	async_thread_ibsta = pthread_getspecific( async_ibsta_key );
	if( async_thread_ibsta == NULL )
	{
		fprintf( stderr, "libgpib: failed to get async_ibsta TSD\n" );
		return -1;
	}

	return *async_thread_ibsta;
}

int AsyncIberr( void )
{
	int *async_thread_iberr;

	globals_alloc();

	async_thread_iberr = pthread_getspecific( async_iberr_key );
	if( async_thread_iberr == NULL )
	{
		fprintf( stderr, "libgpib: failed to get async_iberr TSD\n" );
		return -1;
	}

	return *async_thread_iberr;
}

int AsyncIbcnt( void )
{
	return AsyncIbcntl();
}

long AsyncIbcntl( void )
{
	int *async_thread_ibcntl;

	globals_alloc();
	async_thread_ibcntl = pthread_getspecific( async_ibcntl_key );
	if( async_thread_ibcntl == NULL )
	{
		fprintf( stderr, "libgpib: failed to get async_ibcntl TSD\n" );
		return -1;
	}

	return *async_thread_ibcntl;
}

void sync_globals( void )
{
	ibsta = ThreadIbsta();
	iberr = ThreadIberr();
	ibcntl = ThreadIbcntl();
	ibcnt = ibcntl;
}
