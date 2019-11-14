# Makefile.am
# copyright (C) 2004 by Frank Mori Hess
# email : fmhess@users.sourceforge.net
#
#   This Makefile.am is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.

EXTRA_DIST = 99-ni_usb_gpib.rules.in
CLEANFILES = 99-ni_usb_gpib.rules

USB_FIRMWARE_DIR=$(datadir)/usb
UDEV_RULES_DIR=$(sysconfdir)/udev/rules.d

do_subst = $(SED) -e 's,[@]sbindir[@],$(sbindir),g'\
	-e 's,[@]datadir[@],$(datadir),g'\
	-e 's,[@]libdir[@],$(libdir),g'\
	-e 's,[@]UDEV_LIBDIR[@],$(UDEV_LIBDIR),g'

99-ni_usb_gpib.rules: 99-ni_usb_gpib.rules.in
	$(do_subst) < $(srcdir)/99-ni_usb_gpib.rules.in >99-ni_usb_gpib.rules

all-local: 99-ni_usb_gpib.rules

install-data-local:
	$(INSTALL) -d $(DESTDIR)$(USB_FIRMWARE_DIR)/ni_usb_gpib
	test -d $(DESTDIR)$(UDEV_RULES_DIR) && test -e $(DESTDIR)$(UDEV_RULES_DIR)/99-ni_usb_gpib.rules ||\
		$(INSTALL_DATA) -D 99-ni_usb_gpib.rules $(DESTDIR)$(UDEV_RULES_DIR)/99-ni_usb_gpib.rules
