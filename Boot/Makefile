BOOT_SRCS := $(shell find Boot -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(BOOT_SRCS))
