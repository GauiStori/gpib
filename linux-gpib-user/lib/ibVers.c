/***************************************************************************
                          lib/ibVers.c
                          ------------

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "ibVers.h"
#define xstr(s) str(s)
#define str(s) #s
static char *my_version = xstr(GPIB_SCM_VERSION);

void ibvers( char **vers )
{
  *vers = my_version;
}
