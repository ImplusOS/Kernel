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
	$(ROOT_DIR)/libc/I_libc/Source/src/assert.c \
	$(ROOT_DIR)/libc/I_libc/Source/src/math.c \
	$(ROOT_DIR)/libc/I_libc/Source/src/stdlib.c \
	$(ROOT_DIR)/libc/I_libc/Source/src/string.c \
	$(ROOT_DIR)/libc/I_libc/Source/src/stdio.c \
	$(ROOT_DIR)/libc/I_libc/Source/src/errno.c

OBJS += $(patsubst $(ROOT_DIR)/libc/I_libc/Source/%.c,$(KERNEL_BUILD)/libc/I_libc/%.o,$(LIBC_SRCS))

LIBRARY_SRCS := $(shell find $(ROOT_DIR)/Library/Source -name "*.c" 2>/dev/null)

OBJS += $(patsubst $(ROOT_DIR)/Library/Source/%.c,$(KERNEL_BUILD)/Library/%.o,$(LIBRARY_SRCS))

all: $(KERNEL_ELF)

$(KERNEL_ELF): $(OBJS)
	@mkdir -p $(dir $@)
	$(LD) $(KERNEL_LDFLAGS) $^ -o $@.tmp
	$(OBJCOPY) --strip-all -R .note -R .comment $@.tmp $@
	@mv -f $@.tmp $@.sym

$(KERNEL_BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

# The two on-screen text renderers draw through stb_truetype, which is float
# code, so they cannot be built with the kernel-wide -mgeneral-regs-only.
# That is safe for these two and nowhere else, because neither runs while a
# userland thread is suspended holding live vector registers:
#   Debug/panic/Panic.c    kernel_panic_init() at boot; kernel_panic(), after
#                          which nothing is resumed.
#   Debug/printf/printf.c  the boot-time console mirror, which kernel_main()
#                          detaches with serial_set_screen_mirror(NULL) before
#                          handing control to userland.
# Every other kernel translation unit must stay clear of the FPU/SSE
# registers -- see Kernel/Source/config/arch.mk for why.
KERNEL_FP_SOURCES := Debug/panic/Panic.c Debug/printf/printf.c
KERNEL_FP_OBJS := $(KERNEL_FP_SOURCES:%.c=$(KERNEL_BUILD)/%.o)

$(KERNEL_FP_OBJS): $(KERNEL_BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(filter-out -mgeneral-regs-only,$(KERNEL_CFLAGS)) -c $< -o $@

$(KERNEL_BUILD)/%.o: %.asm
	@mkdir -p $(dir $@)
	$(NASM) -f $(ARCH_ASM_FORMAT) $< -o $@

$(KERNEL_BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

$(KERNEL_BUILD)/libc/I_libc/%.o: $(ROOT_DIR)/libc/I_libc/Source/%.c
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

# libm is float by definition. In the kernel it exists only to satisfy
# stb_truetype, which is reached exclusively from the two boot/panic text
# renderers above, so the same argument applies: nothing calls it while a
# userland thread's vector registers are live.
$(KERNEL_BUILD)/libc/I_libc/src/math.o: $(ROOT_DIR)/libc/I_libc/Source/src/math.c
	@mkdir -p $(dir $@)
	$(CC) $(filter-out -mgeneral-regs-only,$(KERNEL_CFLAGS)) -c $< -o $@

$(KERNEL_BUILD)/Library/%.o: $(ROOT_DIR)/Library/Source/%.c
	@mkdir -p $(dir $@)
	$(CC) $(KERNEL_CFLAGS) -c $< -o $@

clean:
	rm -rf $(KERNEL_BUILD)

-include $(OBJS:.o=.d)
