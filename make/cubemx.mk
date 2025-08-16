include make/common.mk

CUBEMX_BINARY ?= stm32cubemx
CUBEMX_FILE ?= $(CUBEMX_DIR)/cubemx.ioc

.PHONY: cubemx
cubemx:
	@test -f "$(CUBEMX_FILE)" && "$(CUBEMX_BINARY)" "$(CUBEMX_FILE)" || "$(CUBEMX_BINARY)"
