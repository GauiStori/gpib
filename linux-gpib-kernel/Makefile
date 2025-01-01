# Makefile

GPIB_SRCDIR ?= $(CURDIR)
VERBOSE ?= 0
LINUX_SRCDIR ?= /lib/modules/`uname -r`/build
LINUX_SSRCDIR ?= /lib/modules/`uname -r`/source
ENABLE_PCMCIA ?= 0
GPIB_DEBUG ?= 0
LINUX_SSRCDIR != if test -f $(LINUX_SRCDIR)/include/linux/device.h ; then echo $(LINUX_SRCDIR) ; else echo $(LINUX_SSRCDIR) ; fi
# because of RHEL backporting 'class_create' from kernel 6.* to 5.14.? need to
# look at the actual declaration to see which type of call is used.
# Thanks a heap, RedHat.
CLASS_CREATE1ARG != if grep -q ' class_create(owner' $(LINUX_SSRCDIR)/include/linux/device.h $(LINUX_SSRCDIR)/include/linux/device/class.h 2>&1 ; then echo 0 ; else echo 1 ; fi

all:
	$(MAKE) -C $(LINUX_SRCDIR) V=$(VERBOSE) modules \
                M="$(GPIB_SRCDIR)/drivers/gpib" \
                GPIB_TOP_DIR=$(GPIB_SRCDIR) \
                CONFIG_GPIB_ISA="$(ENABLE_ISA)" \
                GPIB_CONFIG_PCMCIA="$(ENABLE_PCMCIA)" \
                HAVE_DEV_OF_NODE=$(HAVE_DEV_OF_NODE) \
                CLASS_CREATE1ARG=$(CLASS_CREATE1ARG) \
                GPIB_CONFIG_KERNEL_DEBUG=$(GPIB_DEBUG)

clean:
	$(MAKE) -C $(LINUX_SRCDIR) V=$(VERBOSE) clean \
                M="$(GPIB_SRCDIR)/drivers/gpib" \
                GPIB_TOP_DIR=$(GPIB_SRCDIR) 

#We run depmod explicitly because the depmod.sh script run
#by modules_install fails on Debian due to it failing to find
#the System.map file.
install:
	$(MAKE) -C $(LINUX_SRCDIR) V=$(VERBOSE) modules_install\
                M="$(GPIB_SRCDIR)/drivers/gpib" \
                GPIB_TOP_DIR=$(GPIB_SRCDIR) \
                INSTALL_MOD_DIR=gpib
	/sbin/depmod -A

