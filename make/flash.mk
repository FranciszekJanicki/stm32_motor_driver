include make/common.mk

FLASHER := STM32_Programmer_CLI
FLASH_PORT ?= SWD
FLASH_BAUD ?=

.PHONY: flash
flash: $(PROJECT_BINARY)
	$(FLASHER) -c port=$(FLASH_PORT)$(if $(FLASH_BAUD), br=$(FLASH_BAUD)) -d "$<" -rst
