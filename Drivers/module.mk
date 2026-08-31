ROOT_DIR ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)

ARCH ?= x86_64

ifeq ($(ARCH),x86_64)
CC := x86_64-elf-gcc
LD := x86_64-elf-ld
OBJCOPY := x86_64-elf-objcopy
DRIVER_ARCH_CFLAGS := -mno-red-zone -DPLATFORM_X86_64
endif

ifeq ($(ARCH),arm64)
CC := aarch64-elf-gcc
LD := aarch64-elf-ld
OBJCOPY := aarch64-elf-objcopy
DRIVER_ARCH_CFLAGS := -mstrict-align -mno-outline-atomics -DPLATFORM_ARM64
endif

DRIVER_BASE_CFLAGS := \
	-I$(ROOT_DIR)/Kernel \
	-I$(ROOT_DIR)/Kernel/include \
	-I$(ROOT_DIR)/Kernel/Arch/$(ARCH) \
	-I$(ROOT_DIR)/Kernel/Core \
	-I$(ROOT_DIR)/Kernel/Platform \
	-I$(ROOT_DIR)/Kernel/Drivers \
	-I$(ROOT_DIR)/Thirdparty -I$(ROOT_DIR)/ThirdParty -I$(ROOT_DIR)/libc/I_libc/include \
	-ffreestanding -fno-stack-protector -fPIC -fno-builtin \
	$(DRIVER_ARCH_CFLAGS) -nostdlib -nostartfiles -nodefaultlibs \
	-Wall -Wextra -Wtype-limits -Wconversion -Wsign-conversion -Wshadow \
	-MMD -MP \
	-Os -g0 -ffunction-sections -fdata-sections \
	-DIMPLUS_DRIVER_MODULE -DKERNEL

DRIVER_MODULE_CFLAGS += $(DRIVER_BASE_CFLAGS)

DRIVER_MODULE_LDFLAGS ?= -nostdlib -shared --build-id=none -Bsymbolic -e driver_module_init -z max-page-size=4096 --gc-sections

DRIVER_SRCS ?= $(sort $(shell find . -type f -name '*.c' -print | sed 's|^\./||'))
DRIVER_BUILD_DIR ?= $(ROOT_DIR)/Build/Modules/$(ARCH)/$(DRIVER_NAME)
DRIVER_ELF ?= $(DRIVER_BUILD_DIR)/$(DRIVER_NAME).ELF
DRIVER_OBJS := $(patsubst %.c,$(DRIVER_BUILD_DIR)/%.o,$(DRIVER_SRCS))

.PHONY: all clean

all: $(DRIVER_ELF)

$(DRIVER_BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(DRIVER_MODULE_CFLAGS) -c $< -o $@

$(DRIVER_ELF): $(DRIVER_OBJS)
	@mkdir -p $(dir $@)
	$(LD) $(DRIVER_MODULE_LDFLAGS) $^ -o $@.tmp
	$(OBJCOPY) --strip-all -R .note -R .comment $@.tmp $@
	@rm -f $@.tmp

clean:
	rm -rf $(DRIVER_BUILD_DIR)

-include $(DRIVER_OBJS:.o=.d)
