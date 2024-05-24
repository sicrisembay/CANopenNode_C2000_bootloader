PHONY := menuconfig defconfig savedefconfig help

ROOT_LOC := $(CURDIR)
ROOT_KCONFIG := $(ROOT_LOC)/Kconfig
KCONFIG_CONFIG_LOC := $(ROOT_LOC)/configs

DEFCONFIG_NAME := bootloader_defconfig
ifeq ("$(origin CONF)", "command line")
DEFCONFIG_FILE := $(KCONFIG_CONFIG_LOC)/$(CONF)
else
DEFCONFIG_FILE := $(KCONFIG_CONFIG_LOC)/$(DEFCONFIG_NAME)
endif

KCONFIG_CONFIG := $(KCONFIG_CONFIG_LOC)/.config
KCONFIG_CONFIG_OUTPUT_LOC := $(KCONFIG_CONFIG_LOC)/generated

export KCONFIG_CONFIG ROOT_KCONFIG KCONFIG_CONFIG_OUTPUT_LOC
export DEFCONFIG_FILE

menuconfig:
	cd configs && make menuconfig

defconfig:
	cd configs && make defconfig

savedefconfig:
	cd configs && make savedefconfig

help:
	@echo 'help'
	@echo 'make menuconfig - run menu configuration'
	@echo 'make defconfig - generate autoconf.h using default file $(DEFCONFIG_NAME)'
	@echo 'make defconfig CONF=file - generate autoconf.h using specified defconfig file'
	@echo 'make savedefconfig - copies .config to configs/$(DEFCONFIG_NAME)'
	@echo 'make savedefconfig CONF=file - copies .config to configs/DEF_CONF'
