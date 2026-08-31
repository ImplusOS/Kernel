.PHONY: all clean

ARCH ?= x86_64
KERNEL_ROOT := $(CURDIR)
ROOT_DIR := $(abspath ../..)
BUILD_DIR ?= $(ROOT_DIR)/Build
KERNEL_BUILD := $(BUILD_DIR)/Kernel
KERNEL_ELF := $(KERNEL_BUILD)/Kernel_Main.ELF

include config/arch.mk

OBJS :=

include Arch/$(ARCH)/Makefile
include Boot/Makefile
include Core/Makefile
include MemoryManagement/Makefile
include Platform/Makefile
include Drivers/Makefile
include Network/Makefile
include Compat/Makefile
include IPC/Makefile
include Debug/Makefile

LIBC_SRCS := \
	$(ROOT_DIR)/I_libc/Source/src/assert.c \
	$(ROOT_DIR)/I_libc/Source/src/math.c \
	$(ROOT_DIR)/I_libc/Source/src/stdlib.c \
	$(ROOT_DIR)/I_libc/Source/src/string.c \
	$(ROOT_DIR)/I_libc/Source/src/stdio.c \
	$(ROOT_DIR)/I_libc/Source/src/errno.c

OBJS += $(patsubst $(ROOT_DIR)/I_libc/Source/%.c,$(KERNEL_BUILD)/libc/I_libc/%.o,$(LIBC_SRCS))

LIBRARY_SRCS := $(shell find $(ROOT_DIR)/Library/Source -name "*.c" 2>/dev/null)

OBJS += $(patsubst $(ROOT_DIR)/Library/Source/%.c,$(KERNEL_BUILD)/Library/%.o,$(LIBRARY_SRCS))

all: $(KERNEL_ELF)

$(KERNEL_ELF): $(OBJS)
	@mkdir -p $(dir $@)
	$(LD) $(KERNEL_LDFLAGS) $^ -o $@.tmp
	$(OBJCOPY) --strip-all -R .note -R .comment $@.tmp $@
	@rm -f $@.tmp

$(KERNEL_BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

$(KERNEL_BUILD)/%.o: %.asm
	@mkdir -p $(dir $@)
	$(NASM) -f $(ARCH_ASM_FORMAT) $< -o $@

$(KERNEL_BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

$(KERNEL_BUILD)/libc/I_libc/%.o: $(ROOT_DIR)/I_libc/Source/%.c
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

$(KERNEL_BUILD)/Library/%.o: $(ROOT_DIR)/Library/Source/%.c
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

clean:
	rm -rf $(KERNEL_BUILD)

-include $(OBJS:.o=.d)
