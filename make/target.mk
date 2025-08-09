include make/common.mk

PROJECT_BINARY := $(BUILD_DIR)/main/main.elf

UART_BAUD_RATE ?= 115200
UART_PORT ?= /dev/ttyACM0
USB_BAUD_RATE ?= 115200
USB_PORT ?= /dev/ttyACM0

OPENOCD ?= openocd
GDB ?= arm-none-eabi-gdb
STM32_PROGRAMMER_CLI ?= STM32_Programmer_CLI

FLASH_METHOD ?= uart
MONITOR_METHOD ?= uart

OPENOCD_INTERFACE ?=
OPENOCD_TARGET ?=

.PHONY: flash_uart
flash_uart: $(PROJECT_BINARY)
	@echo "Flashing $(PROJECT_BINARY) via UART (port=SWD)..."
	$(STM32_PROGRAMMER_CLI) -c port=swd -d "$<" -rst

.PHONY: flash_usb
flash_usb: $(PROJECT_BINARY)
	@echo "Flashing $(PROJECT_BINARY) via USB on $(USB_PORT)..."
	$(STM32_PROGRAMMER_CLI) -c port="$(USB_PORT)" -d "$<" -rst

.PHONY: flash
flash: $(PROJECT_BINARY)
	@if [ "$(FLASH_METHOD)" = "usb" ]; then \
		$(MAKE) flash_usb; \
	else \
		$(MAKE) flash_uart; \
	fi

.PHONY: monitor_uart
monitor_uart:
	@echo "Opening UART serial monitor on $(UART_PORT) at $(UART_BAUD_RATE) baud..."
	minicom -D "$(UART_PORT)" -b "$(UART_BAUD_RATE)"

.PHONY: monitor_usb
monitor_usb:
	@echo "Opening USB serial monitor on $(USB_PORT) at $(USB_BAUD_RATE) baud..."
	minicom -D "$(USB_PORT)" -b "$(USB_BAUD_RATE)"

.PHONY: monitor
monitor:
	@if [ "$(MONITOR_METHOD)" = "usb" ]; then \
		$(MAKE) monitor_usb; \
	else \
		$(MAKE) monitor_uart; \
	fi
	
.PHONY: debug
debug: $(PROJECT_BINARY)
	@if [ -z "$(OPENOCD_INTERFACE)" ] || [ -z "$(OPENOCD_TARGET)" ]; then \
		echo "Error: OPENOCD_INTERFACE and OPENOCD_TARGET must be set"; exit 1; \
	fi
	@echo "Starting OpenOCD with interface: $(OPENOCD_INTERFACE), target: $(OPENOCD_TARGET)"
	@$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" & \
	OPENOCD_PID=$$!; \
	trap 'kill $$OPENOCD_PID 2>/dev/null || true' EXIT; \
	sleep 1; \
	echo "Launching GDB..."; \
	$(GDB) "$(PROJECT_BINARY)" -ex "target extended-remote :3333" -ex "load" -ex "monitor reset halt" -ex "continue"; \
	echo "Stopping OpenOCD..."; \
	kill $$OPENOCD_PID 2>/dev/null || true; \
	trap - EXIT

