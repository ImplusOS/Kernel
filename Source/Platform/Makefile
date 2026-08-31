PLATFORM_SRCS := $(shell find Platform -type f -name '*.c' -print | sort)
ifeq ($(ARCH),arm64)
PLATFORM_SRCS := $(filter-out \
	Platform/interrupt/Interrupts.c \
	Platform/interrupt/IOAPIC.c \
	Platform/interrupt/LAPIC.c, \
	$(PLATFORM_SRCS))
endif
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(PLATFORM_SRCS))
