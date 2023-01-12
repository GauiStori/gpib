/***************************************************************************
                             findlisteners.c
                            ------------------

   An example program to list the devices connected to a board.

    copyright : (C) 2020 by Dave Penkler
    email     : dpenkler@gmail.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <gpib/ib.h>
#include <getopt.h>

static char* myProg;

static void myError(int erc, char * mess) {
  int sys_errno = ThreadIbcnt();
  fprintf(stderr,"%s: error: %s", myProg, mess);
  fprintf(stderr," - %s\n", gpib_error_string(erc));
  if (erc == EDVR)
    fprintf(stderr," system error: %s\n",strerror(sys_errno));
  exit(0);
}

int findListeners(int ud, int from, int to) {
  int ibsta, erc, pad, n = 0;
  short stat;
  int bpad;  /* board primary address */
  int timeout; /* old timeoout */
 
  if (ibask(ud, IbaPAD, &bpad) & ERR)    /* Get board primary address */
    myError(ThreadIberr(), "ibask IbaPAD failed"); 
  if (ibask(ud, IbaTMO, &timeout) & ERR) /* Remember old timeout */
    myError(ThreadIberr(), "ibask IbaTMO failed"); 
  if (ibtmo(ud, T30ms) & ERR) /* Set a shortish timeout for now */
    myError(ThreadIberr(), "ibtmo failed"); 
  for (pad=from; pad<=to; pad++) {
    if (ibln(ud, pad, NO_SAD, &stat) & ERR) { /* check for listener at pad */
      ibsta = ThreadIbsta();
      erc = ThreadIberr();
      ibtmo(ud,timeout); /* Restore old timeout */
      if ((erc == ENOL) || (ibsta & TIMO)) { /* No listeners on the bus */
	return(0);
      } else {
	myError(erc, "unexpected error");
      }
    } else if (stat) {
      printf("Listener at pad %2d", pad);
      if (pad == bpad) printf(" (board) ");
      else n++;
      printf("\n");
    }
  }
  ibtmo(ud,timeout); /* Restore old timeout */
  return n;
}

void usage(int brief) {
  fprintf(stderr,"Usage: %s [-h] [-d <device pad>] [[-m <minor>] | <board name>]\n", myProg);
  if (brief) exit(0);
  fprintf(stderr,"  Where the optional <board name> is the name of the board from gpib.conf\n");
  fprintf(stderr,"  If the <device pad> is not specified all pads are scanned.\n");
  fprintf(stderr,"  Default <minor> is 0\n");
  fprintf(stderr,"  If a <board name> is specified the <minor> is ignored.\n");
  exit(0);
}

int main(int argc, char ** argv) {
  int n;
  char *board;
  int ud, device;
  int gotdev=0;
  int minor = 0;
  int gotmin = 0;
  int from = 0;
  int to = 30;
  int c;
  myProg = argv[0];
  
  while ((c = getopt (argc, argv, "m:d:h")) != -1) {
    switch (c)  {
    case 'm': minor  = atoi(optarg); gotmin++; break; 
    case 'd': device = atoi(optarg); gotdev++; break; 
    case 'h': usage(0); break;
    default: usage(0);
    }
  }

  if (gotdev) {
    if (device < 0 || device > 30) {
      fprintf(stderr,"%s: invalid pad specified 0 <= pad <= 30\n",myProg);
      exit(1);
    } else {
      from = to = device;
    }
  }
  
  if (optind < argc) {
    gotmin = 0; /* ignore minor if one was specified */
    board = argv[optind];
    ud = ibfind(board);
    if (ud < 0) {
      fprintf(stderr, "%s: can't find board \"%s\"\n",myProg, board);
      exit(0);
    }
    if (ibask(ud,IbaBNA,&minor) & ERR) /* get minor of board */
      myError(ThreadIberr(), "ibask ibaBNA failed");
   } else {
    gotmin++;
    ud = minor;
  }

  if (gotdev) printf("%s: Scanning pad %d", myProg, device);
  else  printf("%s: Scanning pads from %d to %d", myProg, from, to);
  
  if (gotmin) printf(" on minor %d\n", minor);
  else printf(" on board \"%s\" minor %d\n", board, minor);
    
  n = findListeners(ud, from, to);

  printf("%s: %d device%s found.\n",myProg, n, (n==1) ? "" : "s");
  exit(n); /* tell invoking script, if any, how many devices were found */
}
