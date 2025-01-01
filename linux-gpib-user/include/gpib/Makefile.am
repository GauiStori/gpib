# include/Makefile.am
# copyright (C) 2003 by Frank Mori Hess
# email : fmhess@users.sourceforge.net
#
#   This Makefile.am is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.

headersdir = $(includedir)/gpib
headers_HEADERS = gpib_user.h ib.h gpib_version.h
noinst_HEADERS = gpib_ioctl.h

EXTRA_DIST = version_subst

CLEANFILES =gpib_version.h

gpib_version.h: 
	echo $(VERSION) | $(SED) -f version_subst > $@
