COMPAT_SRCS := $(shell find Compat -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(COMPAT_SRCS))
