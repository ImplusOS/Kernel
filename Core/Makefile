CORE_SRCS := $(shell find Core -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(CORE_SRCS))
