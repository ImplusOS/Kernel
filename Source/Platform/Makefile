# Tests/ holds host-side harnesses (built by their own run.sh with host stubs
# on the include path); they must never enter the kernel link.
PLATFORM_SRCS := $(shell find Platform -type d -name Tests -prune -o -type f -name '*.c' -print | sort)
ifeq ($(ARCH),arm64)
PLATFORM_SRCS := $(filter-out \
	Platform/interrupt/Interrupts.c \
	Platform/interrupt/IOAPIC.c \
	Platform/interrupt/LAPIC.c, \
	$(PLATFORM_SRCS))
endif
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(PLATFORM_SRCS))
