# Tests/ holds host-side harnesses (built by their own run.sh with host stubs
# on the include path); they must never enter the kernel link.
CORE_SRCS := $(shell find Core -type d -name Tests -prune -o -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(CORE_SRCS))
