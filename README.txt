The Linux GPIB Package
 -----------------------------------
  (c) 1994-1996 by C.Schroeter
  (c) 2001-2011 by Frank Mori Hess

=====================================================

This is a GPIB/IEEE-488 driver and utility package for LINUX.

This software distribution package linux-gpib-4.3.0.tar.gz contains
this README and two tarballs:

1) kernel modules in linux-gpib-kernel-4.3.0.tar.gz

2) user space software in linux-gpib-use-4.3.0.tar.gz containing the
   config program, library, device scripts, examples and documentation

Untar each file and see the respective INSTALL files for instructions
on building and installing.

Send comments, questions and suggestions to to the linux-gpib mailing
list at linux-gpib-general@lists.sourceforge.net

Release Notes for linux-gpib-4.3.0
----------------------------------

Changes since the linux-gpib-4.2.0 release

	- Removed kernel package build dependency on autoconf.  It now
	  only uses a Makefile and the standard linux kernel build
	  tool chain. See the installation instructions in
	  linux-gpib-kernel-4.3.0/INSTALL

	- Noted in docs that gpib.conf will be in the sysconfdir,
          which isn't always /etc/ depending on configuration.
          The configured directory is now printed in configure output. 

	- Added NI PCIe-GPIB to supported hardware matrix.

	- Added ibrsv2, which can be used to avoid spurious service
          requests that ibrsv is prone to. Currently only
          implemented for fmh_gpib and tnt4882.

	- Added support for IbcHSCableLength in ibconfig which is needed
          to enable high speed noninterlocked handshaking (a.k.a.  HS488).

	- Added support for selecting hardware by sysfs device path
          or serial number.

	- Removed obsolete hotplug usermaps.

	- Fixed and refactored udev rules and related scripts.

	- Improved multi-board initialisation scripts.
	  
	- Made ibln and FindLstn address the board as talker,
          to work around problems caused by transceivers preventing
          the NDAC line from being read accurately.

	- A number of other fixes as well as patches from the
          community. See Changelog entries from r1764 for more
          details.
	  
Note: As the udev rules were changed and refactored in this release it
      necessary to remove any pre 4.3.0 gpib udev rules files in
      /etc/udev/rules.d/ before installing linux-gpib-user-4.3.0.
      The files to remove are:
      	   99-agilent_82357a.rules
	   99-gpib-generic.rules
	   99-ni_usb_gpib.rules


