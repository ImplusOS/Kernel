NETWORK_SRCS := $(shell find Network -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(NETWORK_SRCS))
