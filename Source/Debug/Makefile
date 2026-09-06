# Tests/ holds host-side harnesses (built by their own run.sh with host stubs
# on the include path); they must never enter the kernel link.
DEBUG_SRCS := $(shell find Debug -type d -name Tests -prune -o -type f -name '*.c' -print | sort)
OBJS += $(patsubst %.c,$(KERNEL_BUILD)/%.o,$(DEBUG_SRCS))
