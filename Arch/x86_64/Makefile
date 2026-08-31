ARCH_C_SRCS := $(shell find Arch/x86_64 -type f -name '*.c' -print | sort)
ARCH_ASM_SRCS := $(shell find Arch/x86_64 -type f -name '*.asm' ! -name 'scratch.asm' -print | sort)

OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(ARCH_C_SRCS))
OBJS += $(patsubst %.asm,$(KERNEL_BUILD)/%.o,$(ARCH_ASM_SRCS))
