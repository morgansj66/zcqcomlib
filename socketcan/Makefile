# SPDX-License-Identifier: GPL-2.0
#
# GNU Makefile for building the Zeno CANquatro Linux SocketCAN driver
#
# Copyright(C) 2021 Zuragon LTd - www.zuragon.com
#


KERNEL_VERSION ?= `uname -r`
KDIR ?= /lib/modules/$(KERNEL_VERSION)/build

define print_versions
	echo 'Building Zeno CANquatro Socket CAN driver'
	echo '  User    : '$(USER)
	echo '  System  : '`uname -a`
	echo '  CC      : '$(CC)
	echo '  CC ver. : '`$(CC) -dumpfullversion -dumpversion`
	echo '  KDIR    : '$(KDIR)
	echo '  Kernel  : '$(KERNEL_VERSION)
	echo
endef

all: print_versions_start
	$(MAKE) -C $(KDIR) M=$$PWD

install: print_versions_start
	install -D -m 644 zeno_usb.ko /lib/modules/`uname -r`/kernel/drivers/usb/misc/zeno_usb.ko
	depmod -a

print_versions_start:
	@$(call print_versions, Start)
