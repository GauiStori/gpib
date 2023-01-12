/*
-*- linux-c -*-

   A PHP extension for GPIB commands

written by
	Michel Billaud, billaud@labri.fr,
	Laboratoire Bordelais de Recherche en Informatique
	Universit� Bordeaux 1 - France
 	billaud@labri.fr
for
	the Minerva "Emerge" project, in February 2003
	(C) Michel Billaud, 2003

php5 modifications suggested Richard Klingler, richard@klingler.net
Updated for PHP 5.6 by Vince Vielhaber, vev@michvhf.com

 ***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************

STATUS: alpha code

LIMITS: Only ib* command are considered for the moment.

CREDITS:
The list of variable and function names was borrowed from
the linux-gpib-3.1.92 files. http://linux-gpib.sourceforge.net

MODIFICATIONS
 * 2017/08/18 : updated to current PHP library
              : added more error checking in ibrd
 * 2016/07/06 : removed checks for pass by ref for php5
                removed support for old NI_GPIB library
                added ibvers
 * 2003/02/25 : in ibrd, string terminated after ibcnt chars
 * 2003/03/14 : more comments

TODO:
- support for $r = ibrd($ud, $buffer)
- integrate more constants (for ibconf)

NOTES :
 * Installation under Debian requires the php dev package.
 * For convenience, install the gpib.so where dynamic extensions live
 * and add the following line to your php.ini files:
    extension=gpib.so


BIBLIOGRAPHY:

[php4] PHP Developpement d'applications WEB, Tobias Ratschiller et
 Till Gerken, Campus Press, pp. 327-...
 file://localhost/opt/doc.php.nexen.html/zend.structure.exporting-functions.html

*/


#include "gpib/ib.h"
#include "php.h"
#include "ext/standard/info.h"

/* ---- macros for declarations -------------------------- */

#if ( PHP_MAJOR_VERSION >= 8)
#define TSRMLS_CC
#endif

#define FUN_ACCESSOR(functionName) \
ZEND_FUNCTION(functionName)\
{	\
  if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC, "") == FAILURE) \
    {	\
      RETURN_NULL();	\
    }	\
   RETURN_LONG(functionName);   \
}

#define FUN_STRING(functionName) \
ZEND_FUNCTION(functionName)	\
{	\
  char *s;	\
  int s_len;	\
  int r;	\
  if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC, 	\
			    "s", &s , &s_len	\
			    ) == FAILURE) {	\
     RETURN_NULL();	\
  }	\
   r = functionName(s);	\
   RETURN_LONG(r);	\
}


#define FUN_INT_STRING(functionName) \
ZEND_FUNCTION(functionName) \
{ \
  long n; \
  char *s; \
  int s_len; \
  int r; \
  if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
			    "ls", &n , &s , &s_len \
			    ) == FAILURE) { \
     RETURN_NULL(); \
  } \
   r = functionName(n,s); \
   RETURN_LONG(r); \
}

#define FUN_INT_STRING_INT(functionName) \
ZEND_FUNCTION(functionName) \
{ \
  long n,nn; \
  char *s; \
  int s_len; \
  int r; \
  if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
			    "lsl", &n , &s , &s_len ,&nn\
			    ) == FAILURE) { \
     RETURN_NULL(); \
  } \
   r = functionName(n,s,nn); \
   RETURN_LONG(r); \
}



#define FUN_INT(functionName) \
ZEND_FUNCTION(functionName) \
{ \
  long n; \
   int r; \
  if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
			    "l", &n  \
			    ) == FAILURE) { \
     RETURN_NULL(); \
  } \
   r = functionName(n); \
   RETURN_LONG(r); \
}

#define FUN_INT_INT(functionName) \
ZEND_FUNCTION(functionName) \
{ \
  long n,nn; \
   int r; \
  if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
			    "ll", &n, &nn  \
			    ) == FAILURE) { \
     RETURN_NULL(); \
  } \
   r = functionName(n,nn); \
   RETURN_LONG(r); \
}

#define FUN_6INT(functionName) \
ZEND_FUNCTION(functionName) \
{ \
  long n1,n2,n3,n4,n5,n6; \
   int r; \
  if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
			    "llllll", &n1,&n2,&n3,&n4,&n5,&n6 \
			    ) == FAILURE) { \
     RETURN_NULL(); \
  } \
   r = functionName(n1,n2,n3,n4,n5,n6); \
   RETURN_LONG(r); \
}

#define FUN_INT_PINT(functionName) \
ZEND_FUNCTION(functionName) { \
	long n; \
	zval *z; \
	char result; \
	long r; \
	if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
				  "lz", &n, &z) \
	    == FAILURE) { \
		return; \
	} \
	r=functionName(n,&result); \
	ZVAL_LONG(z,result); \
	RETURN_LONG(r); \
}

#define FUN_INT_PSHORT(functionName) \
ZEND_FUNCTION(functionName) { \
	long n; \
	zval *z; \
	short result; \
	long r; \
	if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
				  "lz", &n, &z) \
	    == FAILURE) { \
		return; \
	} \
	r=functionName(n,&result); \
	ZVAL_LONG(z,result); \
	RETURN_LONG(r); \
}

#define FUN_INT_INT_PINT(functionName) \
ZEND_FUNCTION(functionName) { \
	long n,nn; \
	zval *z; \
	int result; \
	long r; \
	if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,  \
				  "llz", &n, &nn, &z) \
	    == FAILURE) { \
		return; \
	} \
	r=functionName(n,nn,&result); \
	ZVAL_LONG(z,result); \
	RETURN_LONG(r); \
}


#define NOT_IMPLEMENTED_YET(functionName) \
ZEND_FUNCTION(functionName) \
{      \
    zend_error(E_WARNING, \
	       "GPIB-PHP: Function not implemented yet"); \
}

/*
 * predefinition of all functions
 */

ZEND_FUNCTION(ibfind);
ZEND_FUNCTION(ibbna);
ZEND_FUNCTION(ibclr);
ZEND_FUNCTION(ibcac);
ZEND_FUNCTION(ibloc);
ZEND_FUNCTION(ibonl);
ZEND_FUNCTION(ibpad);
ZEND_FUNCTION(ibpct);
ZEND_FUNCTION(ibppc);
ZEND_FUNCTION(ibrdf);
ZEND_FUNCTION(ibrsc);
ZEND_FUNCTION(ibrsv);
ZEND_FUNCTION(ibsad);
ZEND_FUNCTION(ibsic);
ZEND_FUNCTION(ibsre);
ZEND_FUNCTION(ibtmo);
ZEND_FUNCTION(ibtrg);
ZEND_FUNCTION(ibvers);
ZEND_FUNCTION(ibwait);
ZEND_FUNCTION(ibwrtf);

ZEND_FUNCTION(gpib_error_string);

ZEND_FUNCTION(ibsta);
ZEND_FUNCTION(ibcnt);
ZEND_FUNCTION(iberr);

ZEND_FUNCTION(ibwrt);
ZEND_FUNCTION(ibrd);

ZEND_FUNCTION (ibask);
ZEND_FUNCTION (ibcmd);
ZEND_FUNCTION (ibcmda);
ZEND_FUNCTION (ibdev);
ZEND_FUNCTION (ibevent);
ZEND_FUNCTION (iblines);
ZEND_FUNCTION (ibrpp);
ZEND_FUNCTION (ibrsp);

ZEND_MINIT_FUNCTION(start_module);

PHP_MINFO_FUNCTION(gpib);

/*
 * table of all PHP  GPIB functions
 */

zend_function_entry php_gpib_functions[] =
{
	ZEND_FE(ibfind,	NULL)
	ZEND_FE(ibbna,	NULL)
	ZEND_FE(ibclr,	NULL)
	ZEND_FE(ibcac,	NULL)
	ZEND_FE(ibloc,	NULL)
	ZEND_FE(ibonl,	NULL)
	ZEND_FE(ibpad,	NULL)
	ZEND_FE(ibpct,	NULL)
	ZEND_FE(ibppc,	NULL)
	ZEND_FE(ibrdf,	NULL)
	ZEND_FE(ibrsc,	NULL)
	ZEND_FE(ibrsv,	NULL)
	ZEND_FE(ibsad,	NULL)
	ZEND_FE(ibsic,	NULL)
	ZEND_FE(ibsre,	NULL)
	ZEND_FE(ibtmo,	NULL)
	ZEND_FE(ibtrg,	NULL)
	ZEND_FE(ibvers,	NULL)
	ZEND_FE(ibwait,	NULL)
	ZEND_FE(ibwrtf,	NULL)

	ZEND_FE(gpib_error_string, NULL)

	ZEND_FE(ibsta,		NULL)
	ZEND_FE(ibcnt,		NULL)
	ZEND_FE(iberr,		NULL)

	ZEND_FE(ibwrt,		NULL)
	ZEND_FE(ibrd,		NULL)

	ZEND_FE (ibask,		NULL)
	ZEND_FE (ibcmd,		NULL)
	ZEND_FE (ibcmda,	NULL)
	ZEND_FE (ibdev,		NULL)
	ZEND_FE (ibevent,	NULL)
	ZEND_FE (iblines,	NULL)
	ZEND_FE (ibrpp,		NULL)
	ZEND_FE (ibrsp,		NULL)

	{ NULL, NULL, NULL}
};


zend_module_entry php_gpib_module_entry =
{
	STANDARD_MODULE_HEADER,
	"gpib",
	php_gpib_functions,
	ZEND_MINIT(start_module), 	/* module startup */
	NULL, 				/* module shutdown */
	NULL,  				/* request startup */
	NULL,  				/* request shutdown */
	PHP_MINFO(gpib), 		/* info function */
	NO_VERSION_YET,
	STANDARD_MODULE_PROPERTIES
};

PHP_MINFO_FUNCTION(gpib)
{
   php_info_print_table_start();
   php_info_print_table_row(2,"GPIB Support","Enabled");
   php_info_print_table_row(2,"Supported library",
			    "Linux GPIB "
			    "(http://linux-gpib/sourceforge.net)");
   php_info_print_table_end();
}


/*
 * Definition of PHP constants
 */

ZEND_MINIT_FUNCTION(start_module)
{
	REGISTER_LONG_CONSTANT("DCAS",	DCAS, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("DTAS",	DTAS, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("LACS",	LACS, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("TACS",	TACS, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ATN",	ATN, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("CIC",	CIC, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("REM",	REM, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("LOK",	LOK, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("CMPL",	CMPL, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EVENT",	EVENT, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("SPOLL",	SPOLL, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("RQS",	RQS, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("SRQI",	SRQI, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("END",	END, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("TIMO",	TIMO, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ERR",	ERR, CONST_CS | CONST_PERSISTENT);

	REGISTER_LONG_CONSTANT("EDVR", 0, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ECIC", 1, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ENOL", 2, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EADR", 3, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EARG", 4, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ESAC", 5, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EABO", 6, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ENEB", 7, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EDMA", 8, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EOIP", 10, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ECAP", 11, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EFSO", 12, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("EBUS", 14, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ESTB", 15, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ESRQ", 16, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("ETAB", 20,  CONST_CS | CONST_PERSISTENT);

	REGISTER_LONG_CONSTANT("TNONE", 0, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T10us", 1, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T30us", 2, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T100us", 3, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T300us", 4, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T1ms", 5, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T3ms", 6, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T10ms", 7, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T30ms", 8, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T100ms", 9, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T300ms", 10, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T1s", 11, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T3s", 12, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T10s", 13, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T30s", 14, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T100s", 15, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T300s", 16, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("T1000s", 17,  CONST_CS | CONST_PERSISTENT);

	REGISTER_LONG_CONSTANT("EOS_ASK",0x1c00, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("REOS", 0x0400, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("XEOS", 0x800, CONST_CS | CONST_PERSISTENT);
	REGISTER_LONG_CONSTANT("BIN", 0x1000,	 CONST_CS | CONST_PERSISTENT);


	return(SUCCESS);
}

#if COMPILE_DL
DLEXPORT zend_module_entry *get_module(void)
{
	return &php_gpib_module_entry;
}
#endif


/*
 * the accessors for the status variables and
 * most functions are defined by macros
 */

     FUN_ACCESSOR(iberr)
     FUN_ACCESSOR(ibcnt)
     FUN_ACCESSOR(ibsta)

     FUN_STRING(ibfind)
     FUN_INT_STRING(ibbna)
     FUN_INT(ibclr)
     FUN_INT_INT(ibcac)
     FUN_INT(ibloc)
     FUN_INT_INT(ibonl)
     FUN_INT_INT(ibpad)
     FUN_INT(ibpct)
     FUN_INT_INT(ibppc)
     FUN_INT_STRING(ibrdf)
     FUN_INT_INT(ibrsc)
     FUN_INT_INT(ibrsv)
     FUN_INT_INT(ibsad)
     FUN_INT(ibsic)
     FUN_INT_INT(ibsre)
     FUN_INT_INT(ibtmo)
     FUN_INT(ibtrg)
     FUN_INT_INT(ibwait)
     FUN_INT_STRING(ibwrtf)
     FUN_INT_PINT(ibrpp)
     FUN_INT_PINT(ibrsp)
     FUN_INT_PSHORT(ibevent)

     FUN_INT_PSHORT(iblines)
     FUN_INT_INT_PINT(ibask)

     FUN_INT_STRING_INT(ibcmd)
     FUN_INT_STRING_INT(ibcmda)

     FUN_6INT(ibdev)


/*
 * a few functions need a special treatment
 */


ZEND_FUNCTION(gpib_error_string)
{
  long n;
	if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,
				  "l", &n
		) == FAILURE) {
		return;
	}
	RETURN_STRING((char*)gpib_error_string(n));
}

ZEND_FUNCTION(ibwrt)
{
	long n,llen;
	char *s;
	int s_len;
	int len;
	if (ZEND_NUM_ARGS() == 2) {
		if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,
					  "ls", &n, &s, &s_len)
		    == FAILURE) {
			return;
		}
		len = s_len;
	}
	else {
		if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,
					  "lsl", &n, &s, &s_len, &llen)
		    == FAILURE) {
			return;
		}
        len = (int)llen;
    }
	RETURN_LONG(ibwrt(n,s,len));
}

ZEND_FUNCTION(ibvers)
{
        zval *z;
        char *version;
	if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,
				  "z", &z)
	    == FAILURE) {
		return;
	}
	ibvers(&version);
	ZVAL_STRING(z,version);
	RETURN_STRING(version);
}

ZEND_FUNCTION(ibrd)
{
	/* at the moment only the traditional form is supported
	   $r = ibrd($ud,$string,$count)
	   (read at most $count bytes into $string)
	   but
	   $r = ibrd($ud,$string)
	   (read unlimited number of bytes) will be. Soon. Really.
	*/
	long n;
	zval *z;
	long len;
	char *p;
	long r;

	if (zend_parse_parameters(ZEND_NUM_ARGS() TSRMLS_CC,
					"lzl", &n, &z, &len)
		== FAILURE) {
		return;
	}
	p = (char *) emalloc(len + 1);
	if(p == NULL) {
		ibsta |= ERR;
		iberr = EABO;  // should be ENOMEM
		RETURN_LONG(ibsta);
	}
	memset(p,0,len+1);
	r=ibrd(n,p,len);
	p[ibcnt]='\0';
	ZVAL_STRING(z,p);
	efree(p);
	RETURN_LONG(r);
}
