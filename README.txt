The Linux GPIB Package
 -----------------------------------
  (c) 1994-1996 by C.Schroeter
  (c) 2001-2011 by Frank Mori Hess

=====================================================

This is a GPIB/IEEE-488 driver and utility package for LINUX.

This software distribution package linux-gpib-4.3.5.tar.gz contains
this README and two tarballs:

1) kernel modules in linux-gpib-kernel-4.3.5.tar.gz

2) user space software in linux-gpib-user-4.3.5.tar.gz containing the
   config program, library, device scripts, examples and documentation

Untar each file and see the respective INSTALL files for instructions
on building and installing.

Send comments, questions and suggestions to to the linux-gpib mailing
list at linux-gpib-general@lists.sourceforge.net

Release Notes for linux-gpib-4.3.5
----------------------------------

Changes since the linux-gpib-4.3.4 release

	Add board support to ibask for IbaBNA option

	Improvements to findlisteners.c

	Fixes for python3.10 from mika

	Optionally suppress printing error messages in ibfind()
	by setting IB_NO_ERROR environment variable.

	Add support for pci version of fmh_gpib_core

	New and improved interrupt driven version of gpib_bitbang
	driver for RPi gpios from Marcello Carla'
	
	See ChangeLog since [r1962] for bug fixes and other changes.
	  
Note: If you have any pre 4.3.0 gpib udev rules files in
      /etc/udev/rules.d/ please remove them before installing
      linux-gpib-user-4.3.5.
      
      The files to remove are:
	   99-agilent_82357a.rules
	   99-gpib-generic.rules
	   99-ni_usb_gpib.rules


