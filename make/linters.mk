include make/common.mk

LINTERS_DIRS := $(MAIN_DIR) $(COMPONENTS_DIR) 
LINTERS_SCRIPT := $(SCRIPTS_DIR)/linters.sh

.PHONY: clang_tidy
clang_tidy:
	"$(SCRIPTS_DIR)/linters.sh" tidy $(LINTERS_DIRS)

.PHONY: clang_format
clang_format:
	"$(SCRIPTS_DIR)/linters.sh" format $(LINTERS_DIRS)

.PHONY: lint
lint: clang_tidy clang_format

