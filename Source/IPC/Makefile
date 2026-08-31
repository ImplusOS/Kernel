IPC_SRCS := $(shell find IPC -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(IPC_SRCS))
