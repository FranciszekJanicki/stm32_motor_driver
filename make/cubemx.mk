include make/common.mk

CUBEMX_BINARY ?= stm32cubemx
CUBEPROG_BINARY ?= stm32cubeprog

.PHONY: cubemx
cubemx:
	"$(CUBEMX_BINARY)"

.PHONY: cubeprog
cubeprog:
	"$(CUBEPROG_BINARY)"
