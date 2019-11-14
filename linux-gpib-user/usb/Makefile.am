# Makefile.am
# copyright (C) 2004 by Frank Mori Hess
# email : fmhess@users.sourceforge.net
#
#   This Makefile.am is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.

SUBDIRS = agilent_82357a ni_usb_gpib
EXTRA_DIST = 98-gpib-generic.rules gpib_udev_fxloader.in gpib_udev_config udevadm.sh
CLEANFILES = gpib_udev_fxloader

UDEV_RULES_DIR = $(sysconfdir)/udev/rules.d

do_subst = $(SED) -e 's,[@]sbindir[@],$(sbindir),g'\
	-e 's,[@]datadir[@],$(datadir),g'\
	-e 's,[@]libdir[@],$(libdir),g'

gpib_udev_fxloader: gpib_udev_fxloader.in
	$(do_subst) < $(srcdir)/gpib_udev_fxloader.in >$@

all-local: gpib_udev_fxloader

install-data-local: 98-gpib-generic.rules
	test -d $(DESTDIR)$(UDEV_RULES_DIR) && test -e $(DESTDIR)$(UDEV_RULES_DIR)/98-gpib-generic.rules ||\
		$(INSTALL_DATA) -D $(srcdir)/98-gpib-generic.rules $(DESTDIR)$(UDEV_RULES_DIR)/98-gpib-generic.rules

install-exec-local: udevadm.sh
	$(INSTALL_SCRIPT) -D $(srcdir)/udevadm.sh $(DESTDIR)$(UDEV_LIBDIR)/gpib_udevadm_wrapper
	$(INSTALL_SCRIPT) -D gpib_udev_fxloader $(DESTDIR)$(UDEV_LIBDIR)/gpib_udev_fxloader
	$(INSTALL_SCRIPT) -D $(srcdir)/gpib_udev_config $(DESTDIR)$(UDEV_LIBDIR)/gpib_udev_config
