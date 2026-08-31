ARCH ?= x86_64

ifeq ($(ARCH),x86_64)
CC := x86_64-elf-gcc
LD := x86_64-elf-ld
OBJCOPY := x86_64-elf-objcopy
NASM := nasm
ARCH_DIR := Arch/x86_64
ARCH_CFLAGS := -mcmodel=small -mno-red-zone -DPLATFORM_X86_64
ARCH_ASM_FORMAT := elf64
KERNEL_LDSCRIPT := Arch/x86_64/linker/linker.ld
else ifeq ($(ARCH),arm64)
CC := aarch64-elf-gcc
LD := aarch64-elf-ld
OBJCOPY := aarch64-elf-objcopy
NASM := false
ARCH_DIR := Arch/arm64
ARCH_CFLAGS := -mstrict-align -mno-outline-atomics -DPLATFORM_ARM64
KERNEL_LDSCRIPT := Arch/arm64/linker/linker.ld
else
$(error Unsupported ARCH: $(ARCH))
endif

KERNEL_CFLAGS := \
	-I$(KERNEL_ROOT) \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/Arch/$(ARCH) \
	-I$(KERNEL_ROOT)/Core \
	-I$(KERNEL_ROOT)/Platform \
	-I$(ROOT_DIR)/Thirdparty \
	-I$(ROOT_DIR)/ThirdParty \
	-I$(ROOT_DIR)/libc/I_libc/include \
	-I$(ROOT_DIR)/Library \
	-ffreestanding -fstack-protector-strong -fPIE -fno-plt -fno-builtin \
	-nostdlib -nostartfiles -nodefaultlibs \
	-Wall -Wextra -Wtype-limits -Wconversion -Wsign-conversion -Wshadow \
	-MMD -MP -DKERNEL \
	-O2 -g0 -ffunction-sections -fdata-sections \
	$(ARCH_CFLAGS)

# Extra kernel compile flags from the command line, e.g.
#   make kernel EXTRA_KERNEL_CFLAGS=-DLINUX_SYSCALL_TRACE
# (Docs/Others/TODO_glibc_Port.md G8 / TODO_Chromium_LinuxABI.md §6). Note:
# GNU make does not track flag changes, so `rm -rf Build/$(ARCH)/Kernel`
# first to force a rebuild.
KERNEL_CFLAGS += $(EXTRA_KERNEL_CFLAGS)

KERNEL_LDFLAGS := -nostdlib --build-id=none -e kernel_main -T $(KERNEL_LDSCRIPT) -pie --no-dynamic-linker -z max-page-size=0x1000 --gc-sections

# CI=1 promotes the warnings already enabled above to errors (Docs/Others/
# TODO_OS_Refactor.md phase P8, 2.1). NOT enabled by default and NOT passed
# by .github/workflows/build.yml yet: as of the P8 pass that added this,
# the kernel tree has pre-existing warnings (-Wunused-function,
# -Wunused-variable, -Wconversion/-Wsign-conversion in a few arithmetic
# spots, one -Wunterminated-string-initialization) predating this
# refactor's phases and out of their scope to fix. Turning CI=1 on in the
# workflow is a good, contained follow-up once those are cleaned up --
# doing it before that would just make the first CI run red for reasons
# unrelated to the change that triggered it.
ifeq ($(CI),1)
KERNEL_CFLAGS += -Werror
endif
