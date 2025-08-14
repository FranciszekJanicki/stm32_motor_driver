include make/common.mk

OPENOCD := openocd
OPENOCD_GDB := arm-none-eabi-gdb
OPENOCD_INTERFACE ?=
OPENOCD_TARGET ?=

.PHONY: debug
debug: $(PROJECT_BINARY)
	@$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" & \
	OPENOCD_PID=$$!; \
	sleep 1; \
	$(OPENOCD_GDB) "$<" \
		-ex "target extended-remote :3333" \
		-ex "load" \
		-ex "monitor reset halt" \
		-ex "continue"; \
	kill $$OPENOCD_PID
