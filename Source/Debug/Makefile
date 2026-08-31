DEBUG_SRCS := $(shell find Debug -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(DEBUG_SRCS))
