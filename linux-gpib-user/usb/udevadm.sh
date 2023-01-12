#!/bin/sh
# Wrapper script for udevadm
UDEVADM_PATH=""
for i in /bin /sbin /usr/bin /usr/sbin; do
	 if [ -x $i/udevadm ]; then
	     UDEVADM_PATH=$i/udevadm
	     break;
	 fi
done
if [ -n "$UDEVADM_PATH" ]; then
    $UDEVADM_PATH $@
else
    echo executable for udevadm not found
fi
