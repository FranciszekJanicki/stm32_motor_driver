include make/common.mk

.PHONY: setup_scripts
setup_scripts:
	find "$(SCRIPTS_DIR)" -type f | xargs $(SUDO) chmod +x
