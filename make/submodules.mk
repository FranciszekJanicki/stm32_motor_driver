include make/common.mk

SUBMODULES_FILE := requirements/submodules.txt
SUBMODULES_SCRIPT := $(SCRIPTS_DIR)/submodules.sh

.PHONY: add_submodules
add_submodules:
	$(SUBMODULES_SCRIPT) $(SUBMODULES_FILE) add

.PHONY: remove_submodules
remove_submodules:
	$(SUBMODULES_SCRIPT) $(SUBMODULES_FILE) remove

.PHONY: update_submodules
update_submodules:
	$(SUBMODULES_SCRIPT) $(SUBMODULES_FILE) update

.PHONY: setup_submodules
setup_submodules: add_submodules update_submodules
