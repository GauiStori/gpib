# driver/Makefile.am
# copyright (C) 2003 by Frank Mori Hess
# email : fmhess@users.sourceforge.net
#
#   This Makefile.am is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.

install-data-hook: gpib.conf

$(DESTDIR)/dev/gpib0:
	$(INSTALL) -d $(DESTDIR)/dev
	for i in `seq 0 15`; \
	do \
		mknod -m u=rw,g=rw,o= $(DESTDIR)/dev/gpib$${i} c $(IBMAJOR) $${i} || exit 1; \
		chown root:gpib $(DESTDIR)/dev/gpib$${i}; \
	done

.PHONY : device_files_install
device_files_install: $(DESTDIR)/dev/gpib0
	@if [ ! -c $(DESTDIR)/dev/gpib0 ]; then \
		if [ -a $(DESTDIR)/dev/gpib0 ]; then \
			echo "A file or directory called /dev/gpib0 exists but it is not" \
				"a character device.  Delete or move it and try again."; \
			exit 1; \
		fi; \
	fi
	@ls -l $(DESTDIR)/dev/gpib0 | grep -q "$(IBMAJOR)"; \
	if [ $$? != 0 ]; then \
		echo "$(DESTDIR)/dev/gpib0 has the wrong major number. " \
			"Delete your $(DESTDIR)/dev/gpibX files and try again."; \
		exit 1; \
	fi

#should move this to util/templates Makefile.am when it exists
.PHONY : gpib.conf
gpib.conf:
	test -e $(DESTDIR)$(sysconfdir)/gpib.conf || \
		$(INSTALL_DATA) -D $(top_srcdir)/util/templates/gpib.conf $(DESTDIR)$(sysconfdir)/gpib.conf
