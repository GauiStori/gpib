# Makefile

GPIB_SRCDIR ?= $$PWD
LINUX_SRCDIR ?= /lib/modules/`uname -r`/build
VERBOSE ?= 0
GPIB_DEBUG ?= 0

all:
	-$(MAKE) -C $(LINUX_SRCDIR) V=$(VERBOSE) modules \
		M="$(GPIB_SRCDIR)/drivers/gpib" \
		GPIB_TOP_DIR=$(GPIB_SRCDIR) \
		CONFIG_GPIB_ISA="$(ENABLE_ISA)" \
		HAVE_DEV_OF_NODE=$(HAVE_DEV_OF_NODE) \
		GPIB_CONFIG_KERNEL_DEBUG=$(GPIB_DEBUG)

clean:
	$(MAKE) -C $(LINUX_SRCDIR) V=$(VERBOSE) clean \
		M="$(GPIB_SRCDIR)/drivers/gpib" \
		GPIB_TOP_DIR=$(GPIB_SRCDIR) 

install:
	$(MAKE) -C $(LINUX_SRCDIR) V=$(VERBOSE) modules_install\
		M="$(GPIB_SRCDIR)/drivers/gpib" \
		GPIB_TOP_DIR=$(GPIB_SRCDIR) \
		INSTALL_MOD_DIR=gpib
