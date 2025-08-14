include make/common.mk

MONITOR ?= minicom
MONITOR_PORT ?= /dev/ttyACM0
MONITOR_BAUD ?= 115200

.PHONY: monitor
monitor:
	$(MONITOR) -D "$(MONITOR_PORT)" -b "$(MONITOR_BAUD)"
