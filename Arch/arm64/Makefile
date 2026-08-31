ARCH_C_SRCS := $(shell find Arch/arm64 -type f -name '*.c' -print | sort)
ARCH_ASM_SRCS := $(shell find Arch/arm64 -type f -name '*.S' -print | sort)

OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(ARCH_C_SRCS))
OBJS += $(patsubst %.S,$(KERNEL_BUILD)/%.o,$(ARCH_ASM_SRCS))
