The Linux GPIB Package
 -----------------------------------
  (c) 1994-1996 by C.Schroeter
  (c) 2001-2011 by Frank Mori Hess

=====================================================

This is a GPIB/IEEE-488 driver and utility package for LINUX.

This software distribution package linux-gpib-x.x.x.tar.gz contains
this README and two tarballs:

1) kernel modules in linux-gpib-kernel-x.x.x.tar.gz

2) user space software in linux-gpib-user-x.x.x.tar.gz containing the
   config program, library, device scripts, examples and documentation

Untar each file and see the respective INSTALL files for instructions
on building and installing.

Send comments, questions and suggestions to to the linux-gpib mailing
list at linux-gpib-general@lists.sourceforge.net

Release Notes for linux-gpib-4.3.6
----------------------------------

Changes since the linux-gpib-4.3.5 release

	Major rework of gpib_bitbang driver for RPi gpios
	with fix for lost edge interrupts on BCM2835 peripheral chip
	from Marcello Carla'.

	Fix to avoid async IO race conditions in ni_usb_gpib.

	Fix for unaddressed writes blocking in agilent_82350b with
	the accelerated interface.
	
	See ChangeLog since [r2031] for bug fixes and other changes.
	  
Note: If you have any pre 4.3.0 gpib udev rules files in
      /etc/udev/rules.d/ please remove them before installing
      linux-gpib-user-x.x.x.
      
      The files to remove are:
	   99-agilent_82357a.rules
	   99-gpib-generic.rules
	   99-ni_usb_gpib.rules


