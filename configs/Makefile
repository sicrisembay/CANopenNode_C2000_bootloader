PHONY := create_folder

menuconfig: create_folder
	menuconfig $(ROOT_KCONFIG)
	cd $(KCONFIG_CONFIG_OUTPUT_LOC) && genconfig $(ROOT_KCONFIG) && mv config.h autoconf.h

defconfig: create_folder
	@echo 'Building $(DEFCONFIG_FILE)'
	defconfig --kconfig $(ROOT_KCONFIG) $(DEFCONFIG_FILE)
	cd $(KCONFIG_CONFIG_OUTPUT_LOC) && genconfig $(ROOT_KCONFIG) && mv config.h autoconf.h

savedefconfig:
	cp .config $(DEFCONFIG_FILE)

create_folder:
	mkdir -p generated
