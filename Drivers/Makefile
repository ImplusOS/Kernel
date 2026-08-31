# DRIVER_RESIDENT_SRCS -- kernel-resident driver-framework code, compiled
# directly into the kernel binary (unlike the loadable driver modules under
# Drivers/<Category>/<Driver>/, each built separately via module.mk and its
# own Makefile -- see driver_build/DRIVER_DIRS in the top-level Makefile).
# Explicit allowlist rather than "everything not under a driver's own
# directory": Drivers/Module/ holds the driver framework
# (DriverManager/BusRegistry/DeviceRegistry/...) plus the per-bus/per-class
# gateway code (including the filesystem VFS bridges) that used to live
# under the now-removed Drivers/Client/ and Drivers/Server/ split (see
# Docs/Others/TODO_OS_Refactor.md 5. and 6.2). It is the only kernel-resident
# code under Drivers/; the CMOS RTC (a port-IO platform HAL, not a driver
# module) now lives under Platform/rtc/ and is built by Platform/Makefile.
DRIVER_RESIDENT_DIRS := Drivers/Module
DRIVER_RESIDENT_SRCS := $(shell find $(DRIVER_RESIDENT_DIRS) -type f -name '*.c' -print 2>/dev/null | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(DRIVER_RESIDENT_SRCS))
