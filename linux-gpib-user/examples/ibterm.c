/***************************************************************************
                                 ibterm.c
                            ------------------

 Another example program which uses the linux gpib C library:

   An interactive terminal program for sending commands to an instrument
   and printing its response.

    copyright : (C) 2013 by Dave Penkler
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

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <stdlib.h>
#include <getopt.h>
#ifdef READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif
#include "gpib/ib.h"

#define PROMPT "ibterm>"
#define DEVICE_BUFFER_SIZE 2048
#define false 0
#define true  1

/* Global variable declarations */
static char * mPrompt   = PROMPT;        // Prompt string
static char * mProg;                     // Programme name
static char * mHist     = (char *)NULL;  // History filename pointer
static int mAutoRead    = true;          // Automatically read from device
static int mHex         = false;         // Force hex output flag
static int devdesc  = -1;  // device descriptor
static int minor    = 0;   // gpib driver major
static int pad      = -1;  // device primary bus address
static int sad      = 0;   // device secondary bus address
static int send_eoi = 1;   // Assert eoi with last byte in ibwrt
static int eos_mode = 0;   // No end of string processing
static int timeout  = 10;  // 300 milliseconds
static char errmes[256];   // error message buffer

static const char * help_string =
  "ibterm help info -\n"
  "\nSummary:\n"
  "An interactive terminal program for sending commands to a device over an \n"
  "IEEE488 general purpose instrument bus and printing the responses.\n"
  "\nOptions:\n"
  " -d <device primary address (pad)>                (required, no default)\n"
  " -m <minor>                                       (optional, default=0)\n"
  " -s <secondary address of device (sad)>           (optional, default=0)\n"
  " -i <assert EIO with last byte sent (eoi) flag>   (optional, default=1)\n"
  " -e <ASCII code of end of string (eos) character> (optional, default=0)\n"
  " -r <terminate read on eos character (reos) flag> (optional, default=0)\n"
  " -b <binary eos character match mode (bin)  flag> (optional, default=0)\n"
  " -x <assert EOI when tranmitting eos (xeos) flag> (optional, default=0)\n"
  " -t <timeout>       (optional, default=10 i.e. 300milliseconds)\n"
  " -p <prompt string> (optional, default=\"" PROMPT "\")\n"
#ifdef READLINE
  " -f <history file>  (optional, default=\".ibterm_hist_<pad>\")\n"
#endif
  " -N No automatic read on device, enter return at prompt to read.\n"
  " -X forces hexadecimal output.\n"
  " -h prints this help info and exits.\n"
  "\nOperation:\n"
  "  loop:\n"
  "    Prompt on stdout\n"
  "    Read a line of text from stdin\n"
  "    Write the text (if any) to the device at pad\n"
  "    If -N is not set, or no text was entered\n"
  "        Attempt to read response from the device\n"
  "        If no response is received before timeout go to loop\n"
  "        else print the response on stdout\n"
  "    Go back to loop.\n"
  "\nNotes:\n"
  "  To quit the programme enter the EOF character (Ctrl-d) at the prompt.\n"
  "  For interactivity, timeout should not be greater than 13 i.e. 10 secs.\n"
  "  The timeout values are documented under the ibtmo() entry of the \n"
  "  section Linux-GPIB Reference: http://linux-gpib.sourceforge.net/doc_html\n"
  "  A device read can always be triggered by hitting <Enter> at the prompt.\n"
  "  Interrupting the programme while reading from the device may cause hangs.\n"
#ifdef READLINE
  "\nAlso:\n"
  "  See the readline(3) manpage for editing input and searching history.\n"
#endif
  ;

static const char * usage_options =
  " -d pad \\\n"
  "         [-m minor] [-s sad] [-i eoi] [-e eos]"
  " [-r reos] [-b bin] [-x xeos] \\\n"
  "         [-t timeout] [-p prompt]"
#ifdef READLINE
  " [-f history_file]"
#endif
  " [-N] [-X]\n";

#define EMES(var) fputs(var,stderr)

static void abend(const char * mess) {
  EMES(mess);
  EMES(mProg);
  EMES(": Aborted\n");
  if (devdesc >= 0) ibonl(devdesc,0);
  exit(1);
}

static void usage(int abort) {
  EMES("\nUsage:\n  ");
  EMES(mProg);
  EMES(usage_options);
  if (abort) {
    sprintf(errmes,"Try '%s -h' for more information.\n",mProg);
    abend(errmes);
  }
  exit(1);
}

/* Print buffer contents in parallel hexadecimal and ASCII */
static const char * hexdig = "0123456789ABCDEF";
static char * template =
  "12345678 |                         | "
             "                        |                 ";
void prhex(unsigned char * buf, int len) {
  int i,j,k,l,m;
  char *c,ob[80];

  for (i=0;i<len;i++)  {
    if ((i % 16) == 0) {
      k = 8; l = 63;
      strcpy(ob,template);
      for (j=0,c=&ob[8],m=i;j<8;m >>= 4,j++)
	*--c = hexdig[m & 0xf];
    }
    k += 1 + 2*((i % 8) == 0);
    ob[k++] = hexdig[(buf[i] & 0xf0)>>4];
    ob[k++] = hexdig[(buf[i] & 0xf)];
    ob[l++] = (buf[i] < 32 || buf[i] > 127) ? ' ' : buf[i];
    if (k == 60) { puts(ob); k = 0; }
   }
  if (k) puts(ob);
}

#define CHECK_FLAG(var, flag)			\
if (var != 0 && var != 1)  abend(#flag " flag must be 1 or 0.\n");

#define CHECK_ADDR(var)		  \
if (var < 0 || var > 30)  abend(#var " must be between 0 and 30.\n");

#define CHECK_SADDR(var) 	\
if (var < 0x60 || var > 0x7e)  abend("linux-gpib requires the secondary address to be offset by 96,\n   that is sad must be between 96 and 126.\n");

void parse_options(int argc, char ** argv) {
  int eos_char  = 0;   // End of string character
  int reos_mode = 0;   // Don't terminate read on eos_char
  int bin_mode  = 0;   // Match only 7 bits of eos_char
  int xeos_mode = 0;   // Don't assert EOI when sending eos_char
  int c;

  mProg = argv[0];

  while ((c = getopt (argc, argv, "d:m:s:i:e:r:b:x:t:f:p:NXh")) != -1)
    switch (c)  {
    case 'd': pad       = atoi(optarg);    break;
    case 'm': minor     = atoi(optarg);    break;
    case 's': sad       = atoi(optarg);    break;
    case 'i': send_eoi  = atoi(optarg);    break;
    case 'e': eos_char  = atoi(optarg);    break;
    case 'r': reos_mode = atoi(optarg);    break;
    case 'b': bin_mode  = atoi(optarg);    break;
    case 'x': xeos_mode = atoi(optarg);    break;
    case 't': timeout   = atoi(optarg);    break;
    case 'f': mHist     = optarg;          break;
    case 'p': mPrompt   = optarg;          break;
    case 'N': mAutoRead = false;           break;
    case 'X': mHex      = true;            break;
    case 'h':
      EMES(help_string);
      usage(0);
      break;
    default: usage(1);
    }
  if (pad == -1) {
    EMES("No primary device address (pad) specified!\n");
    usage(1);
  }
  CHECK_ADDR(pad);
  if (sad) CHECK_SADDR(sad);
  CHECK_FLAG(send_eoi, eoi);
  CHECK_FLAG(reos_mode,reos);
  CHECK_FLAG(bin_mode, bin);
  CHECK_FLAG(xeos_mode,xeos);
  if (eos_char < 0 || eos_char > 255) {
    abend("eos character must be between 0 and 255,\n");
  }
  if (!bin_mode && eos_char > 127) {
    EMES("Warning eos is 8 bits but compares are set to 7.\n");
  }
  if (timeout < 1 || timeout > 15) abend("<timeout> must be between 1 and 15.");
  eos_mode = eos_char;
  if (reos_mode) eos_mode |= REOS;
  if (bin_mode)  eos_mode |= BIN;
  if (xeos_mode) eos_mode |= XEOS;
  if (mHist == NULL) {
    mHist = malloc(32);
    sprintf(mHist,".ibterm_hist_%02d",pad);
  }
}

static void showError(char * mess) {
  fprintf(stderr,"ibterm error: %s\n", mess);
  fprintf(stderr," - %s\n", gpib_error_string(ThreadIberr()));
}

#ifndef READLINE
#define READ_BUF_SIZE 2048
char * readline(char * prompt) {
  char * buf, * ret;
  int len;
  fwrite(prompt,1,strlen(prompt),stdout);
  fflush(stdout);
  if (!(buf = malloc(READ_BUF_SIZE))) {
    abend("no memory");
  }
  ret = fgets(buf,READ_BUF_SIZE,stdin);
  if (ret) {
    len = strlen(ret);
    if (ret[len-1] == '\n') ret[len-1] = 0;
  } else  free(buf);
  return ret;
}
  void read_history(char * buf) {};
  void add_history(char * buf) {};
  void write_history(char * buf) {};
#endif

int main (int argc, char ** argv) {
  char * line;
  int i, devdatalen,datasent;
  unsigned char devbuf[DEVICE_BUFFER_SIZE];
  int printable;

  parse_options(argc, argv);

  printf("Attempting to open /dev/gpib%i\n"
	 "pad = %d, sad = %d, timeout = %d, send_eoi = %d, eos_mode = 0x%04x\n",
	 minor,pad,sad,timeout,send_eoi,eos_mode);
  devdesc = ibdev(minor, pad, sad, timeout, send_eoi, eos_mode);
  if (devdesc < 0) {
    showError("open failed");
    abend("ibdev error\n");
  }

  read_history(mHist);

  /* prompt user - read term - write device - read device - print term loop */
  while (line = readline(mPrompt)) { /* prompt  and read from user */

    /* write to device */
    if (*line) { /* send to device only if we got something */
     if (ibwrt(devdesc,line,strlen(line)) & ERR ) {
	sprintf(errmes,"Unable to write to device at pad %d\n",pad);
	showError(errmes);
	free(line);
	continue;
      }
      add_history (line);
      datasent = true;
    } else  datasent = false;

    free(line); /* return buffer allocated by readline */

    if (mAutoRead || !datasent) {   /* read response if any from device */
      if (ibrd(devdesc,devbuf,DEVICE_BUFFER_SIZE) & ERR) {
	if (!(ThreadIbsta() & TIMO)) {
	  sprintf(errmes,"Failed during read from device at pad %d\n",pad);
	  showError(errmes);
	}
	continue;
      }
      devdatalen = ThreadIbcntl();
    } else continue;

    /* print response */
    if (!mHex) {
      printable  = true;
      for(i = 0; i < devdatalen; ++i) {
	if (!isprint(devbuf[i]) && !isspace(devbuf[i])){
	  printable = false;
	  break;
	}
      }
    } else printable = false;
    if (printable) {
      fwrite(devbuf,1,devdatalen,stdout); /* print response as ASCII string */
      /* but don't print an extra newline if it ends with LF or LF CR */
      if ((devdatalen > 2 &&
	   devbuf[devdatalen-2] != '\n' ||  devbuf[devdatalen-1] != '\r') &&
	  devbuf[devdatalen-1] != '\n') putchar('\n');
    } else {
      prhex(devbuf,devdatalen); /* print contents with hex and ascii */
    }
  }

  /* Finish up */
  ibonl(devdesc, 0);   /* free up gpib library resources */
  write_history(mHist);
  puts("\nibterm: Done.");
  exit(0);
}
