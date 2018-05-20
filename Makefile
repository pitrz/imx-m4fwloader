#20.05.2018, D.RY
# - Build the app using Yocto generated toolchain and installed to a set path.
#
# ** * READ HERE * **
# Please create your own local file named "makefile.local" , defining the following variables:
#  TOOLCHAIN_CROSS_PATH   :  path to the cross compiler
#  TOOLCHAIN_CROSS_TARGET :  the target for which this is compiled for (a Linux one .. )
#  TOOLCHAIN_SYSROOT      :  the target sysroot path, passed to the cross compiler
#
# The above variables will be combined to invoke your local comiler, matching how the toolchain was
# installed on your system (Hopefully .. )
#
# Example below is for the toolchain generated by Yocto's NXP distro, for the iMX7D.
#
#TOOLCHAIN_DISTRO=poky-linux
#TOOLCHAIN_INSTALL_DIR=/opt/fsl-imx-fb/
#TOOLCHAIN_VER=4.9.11-1.0.0
#TOOLCHAIN_DIR=$(TOOLCHAIN_INSTALL_DIR)/$(TOOLCHAIN_VER)/sysroots/
#TOOLCHAIN_CROSS_TARGET=arm-$(TOOLCHAIN_DISTRO)
#TOOLCHAIN_TARGET_ARCH=cortexa7hf-neon
#TOOLCHAIN_CROSS_HOST=x86_64-pokysdk-linux
#TOOLCHAIN_CROSS_PATH=$(TOOLCHAIN_DIR)/$(TOOLCHAIN_CROSS_HOST)/usr/bin/$(TOOLCHAIN_CROSS_TARGET)/
#TOOLCHAIN_SYSROOT=$(TOOLCHAIN_DIR)/$(TOOLCHAIN_TARGET_ARCH)-$(TOOLCHAIN_DISTRO)-gnueabi
#
# Please DO NOT COMMIT that file - makefile.local - to the Git repo, thanks.

include makefile.local

CC=gcc
CROSS_PREFIX=$(TOOLCHAIN_CROSS_TARGET)-
CCFLAGS=--sysroot=$(TOOLCHAIN_SYSROOT)/ -mfloat-abi=hard -Wall -std=gnu11

export PATH := $(TOOLCHAIN_CROSS_PATH):$(PATH)

all: m4fwloader

m4fwloader:	m4fwloader.c
	$(CROSS_PREFIX)$(CC) $(CCFLAGS) -o $@.out $@.c

clean:
	rm -vf *.o *.out
