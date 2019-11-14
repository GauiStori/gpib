
dnl Process this file with autoconf to produce a configure script.

AC_INIT([linux-gpib-user], [4.3.0])
# libtool version: current:revision:age
#
# If the library source code has changed at all since the last update, then
# increment revision (`c:r:a' becomes `c:r+1:a').
#
# If any interfaces have been added, removed, or changed since the last update,
# increment current, and set revision to 0.
#
# If any interfaces have been added since the last public release, then
# increment age.
#
# If any interfaces have been removed since the last public release, then set
# age to 0.
GPIB_SO_VERSION=4:0:4
AC_CONFIG_SRCDIR([lib/ibask.c])
AM_INIT_AUTOMAKE
AM_MAINTAINER_MODE
AC_CONFIG_MACRO_DIR([m4])

dnl create a config.h file (Automake will add -DHAVE_CONFIG_H)
AC_CONFIG_HEADERS([config.h])

AC_SUBST([VERSION])
AC_SUBST([GPIB_SO_VERSION])

[ISODATE=`date +%Y-%m-%d`]
AC_SUBST([ISODATE])

AC_CANONICAL_HOST

AC_ENABLE_SHARED([yes])
AC_ENABLE_STATIC([no])
AC_ARG_ENABLE([all-bindings],[  --disable-all-bindings	Disable all language binding to libgpib],
	[BINDING_DEFAULT=$enableval],[BINDING_DEFAULT="yes"])
AC_ARG_ENABLE([guile-binding],[  --disable-guile-binding	Disable Guile binding to libgpib],
	[BIND_GUILE=$enableval],[BIND_GUILE=$BINDING_DEFAULT])
AC_ARG_ENABLE([perl-binding],[  --disable-perl-binding	Disable Perl binding to libgpib],
	[BIND_PERL=$enableval],[BIND_PERL=$BINDING_DEFAULT])
AC_ARG_ENABLE([php-binding],[  --disable-php-binding		Disable PHP binding to libgpib],
	[BIND_PHP=$enableval],[BIND_PHP=$BINDING_DEFAULT])
AC_ARG_ENABLE([python-binding],[  --disable-python-binding	Disable Python binding to libgpib],
	[BIND_PYTHON=$enableval],[BIND_PYTHON=$BINDING_DEFAULT])
AC_ARG_ENABLE([tcl-binding],[  --disable-tcl-binding		Disable TCL binding to libgpib],
	[BIND_TCL=$enableval],[BIND_TCL=$BINDING_DEFAULT])
AC_ARG_ENABLE([documentation],[  --disable-documentation	Disable building documentation from SGML sources],
	[BUILD_DOCS=$enableval],[BUILD_DOCS="yes"])

dnl udev lib dir
AC_ARG_WITH([udev-libdir],
	[AS_HELP_STRING([--with-udev-libdir=DIR], [location to install udev scripts [PREFIX/lib/udev]])],
	[UDEV_LIBDIR=$withval], [UDEV_LIBDIR='${prefix}'/lib/udev])
AC_SUBST([UDEV_LIBDIR])

dnl Checks for programs.
AC_PROG_INSTALL
AC_PROG_CC
AM_PROG_CC_C_O
AM_PROG_LEX
AC_PROG_YACC
AC_PROG_LIBTOOL
AC_PATH_PROG([JW_PATH], [jw], [no])
if test "$JW_PATH" = "no" ; then
	AC_MSG_NOTICE([docbook-tools (jw) not found, disabling documentation])
	BUILD_DOCS="no"
fi

if test "$BIND_PYTHON" = "yes"; then
	AM_PATH_PYTHON()
	PYTHON_MAJOR_VERSION=$(echo $PYTHON_VERSION | cut -d. -f1 )
	PYTHON_MINOR_VERSION=$(echo $PYTHON_VERSION | cut -d. -f2 )
	if (( $PYTHON_MAJOR_VERSION < 2 )); then
		AC_MSG_WARN([python version less than 2.0, disabling python binding])
		BIND_PYTHON="no"
	fi
fi

if test "$BIND_PERL" = "yes"; then
   AC_PATH_PROG([PERL], [perl], [no], [])
   if test "$PERL" = "no"; then
      AC_MSG_NOTICE([perl not found, disabling perl binding])
      BIND_PERL="no"
    fi
fi

if test "$BIND_TCL" = "yes"; then
   SC_PATH_TCLCONFIG   
   if [[ $no_tcl ]]; then
        AC_MSG_NOTICE([TCL config not found, disabling TCL binding])
        BIND_TCL="no"
   else
	SC_LOAD_TCLCONFIG
   fi
fi

if test "$BIND_PHP" = "yes"; then
   AC_PATH_PROG([PHP_CONFIG], [php-config], [no],[])
   if test "$PHP_CONFIG" = "no"; then
      AC_MSG_NOTICE([php-config not found, disabling PHP binding])
      BIND_PHP="no"
   fi
else
   PHP_CONFIG="no"
fi

dnl Checks for libraries.
if test "$BIND_GUILE" = "yes"; then
AC_CHECK_LIB([guile],[gh_define],[echo this prevents default actions from occurring > /dev/null],
	[BIND_GUILE="no";AC_MSG_NOTICE([libguile not found, disabling guile binding])],[])
fi

AC_CHECK_LIB([readline], [readline], 
  	     [readline_lib=yes;AC_SUBST([LIB_READLINE_LDFLAGS], ["-lreadline -lncurses"])],
	     [],
	     [-lncurses])
		       
dnl Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS(unistd.h sys/param.h sys/time.h time.h sys/mkdev.h sys/sysmacros.h string.h memory.h fcntl.h dirent.h sys/ndir.h ndir.h alloca.h locale.h )

if test "$BIND_PYTHON" = "yes"; then
AM_CHECK_PYTHON_HEADERS([],[BIND_PYTHON=no;AC_MSG_NOTICE([python headers not found, disabling python binding])])
fi

if test "$BIND_GUILE" = "yes"; then
AC_CHECK_HEADER([libguile.h],[],[BIND_GUILE="no";AC_MSG_NOTICE([libguile headers not found, disabling guile binding])])
fi

AC_CHECK_HEADER(readline/readline.h,[readline_h=yes])
AC_CHECK_HEADER(readline/history.h,[history_h=yes])
if test "$readline_lib" = "yes" && test "$readline_h" = "yes" && test "$history_h" = "yes"; then
   AC_SUBST([LIB_READLINE_CFLAGS], ["-DREADLINE"])
fi

if test "$BIND_TCL" != "no"; then
SAVE_CPPFLAGS=$CPPFLAGS
CPPFLAGS="$CPPFLAGS $TCL_INCLUDE_SPEC"
AC_CHECK_HEADER([tcl.h],[],[BIND_TCL="no";AC_MSG_NOTICE([TCL headers not found, disabling TCL binding])])
CPPFLAGS=$SAVE_CPPFLAGS
fi

if test "$PHP_CONFIG" != "no"; then
SAVE_CPPFLAGS=$CPPFLAGS
CPPFLAGS="$CPPFLAGS $($PHP_CONFIG --includes)"
AC_CHECK_HEADER([php.h],[],[BIND_PHP="no";AC_MSG_NOTICE([PHP headers not found, disabling PHP binding])])
CPPFLAGS=$SAVE_CPPFLAGS
fi

AC_HEADER_MAJOR
AC_FUNC_ALLOCA
AC_STRUCT_TM
AC_STRUCT_ST_BLOCKS
AC_FUNC_CLOSEDIR_VOID
AC_CHECK_FUNCS([mkfifo])
AC_CHECK_FUNC([mknod])

dnl Checks for typedefs, structures, and compiler characteristics.

dnl Checks for library functions.

LIBGPIB_CFLAGS="-I\$(top_srcdir)/include -D_REENTRANT"
LIBGPIB_LDFLAGS="\$(top_builddir)/lib/libgpib.la -lpthread"
AC_SUBST([LIBGPIB_CFLAGS])
AC_SUBST([LIBGPIB_LDFLAGS])
IBMAJOR=160
AC_SUBST([IBMAJOR])
AC_DEFINE([IBMAJOR],[160],[Major number of device files])

echo
echo Configuration:
AM_CONDITIONAL([BUILD_DOCS], [test "$BUILD_DOCS" = "yes"])
echo "SGML Documentation: $BUILD_DOCS"
AM_CONDITIONAL([BIND_GUILE], [test "$BIND_GUILE" = "yes"])
echo "Guile binding: $BIND_GUILE"
AM_CONDITIONAL([BIND_PERL], [test "$BIND_PERL" = "yes"])
echo "Perl binding: $BIND_PERL"
AM_CONDITIONAL([BIND_PHP], [test "$BIND_PHP" = "yes"])
echo "PHP binding: $BIND_PHP"
AM_CONDITIONAL([BIND_PYTHON], [test "$BIND_PYTHON" = "yes"])
echo "Python binding: $BIND_PYTHON"
AM_CONDITIONAL([BIND_TCL], [test "$BIND_TCL" = "yes"])
echo "TCL binding: $BIND_TCL"
AX_DEFINE_DIR([SYSCONFDIR], [sysconfdir], [System configuration directory])
AS_ECHO "sysconfdir: [$SYSCONFDIR]"
AS_ECHO ""

AC_OUTPUT([\
	Makefile \
	drivers/Makefile \
	lib/Makefile \
	lib/libgpib.pc \
	lib/gpib_config/Makefile \
	examples/Makefile \
	test/Makefile \
	doc/Makefile \
	include/Makefile \
	include/gpib/Makefile \
	language/Makefile \
	language/guile/Makefile \
	language/php/Makefile \
	language/php/TESTS/Makefile \
	language/python/Makefile \
	language/tcl/Makefile \
	usb/Makefile \
	usb/ni_usb_gpib/Makefile \
	usb/agilent_82357a/Makefile \
]
)

