MEMORY_SRCS := $(shell find MemoryManagement -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(MEMORY_SRCS))
