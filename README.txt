The Linux GPIB Package
 -----------------------------------
  (c) 1994-1996 by C.Schroeter
  (c) 2001-2011 by Frank Mori Hess

=====================================================

This is a GPIB/IEEE-488 driver and utility package for LINUX.

This software distribution package linux-gpib-4.3.4.tar.gz contains
this README and two tarballs:

1) kernel modules in linux-gpib-kernel-4.3.4.tar.gz

2) user space software in linux-gpib-user-4.3.4.tar.gz containing the
   config program, library, device scripts, examples and documentation

Untar each file and see the respective INSTALL files for instructions
on building and installing.

Send comments, questions and suggestions to to the linux-gpib mailing
list at linux-gpib-general@lists.sourceforge.net

Release Notes for linux-gpib-4.3.4
----------------------------------

Changes since the linux-gpib-4.3.3 release

	New GPIO bitbang driver for Raspberry Pi from Marcello Carla'

	Updated lpvo_usb_gpib to use usb directly from Marcello Carla'

	Better support for detecting listeners with ibln

	New example programme findlisteners.c

	Various changes for new kernel and autoconf versions

	See ChangeLog since [r1912] for bug fixes and other changes.
	  
Note: If you have any pre 4.3.0 gpib udev rules files in
      /etc/udev/rules.d/ please remove them before installing
      linux-gpib-user-4.3.4.
      
      The files to remove are:
	   99-agilent_82357a.rules
	   99-gpib-generic.rules
	   99-ni_usb_gpib.rules


