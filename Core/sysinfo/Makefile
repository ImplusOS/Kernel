SYSINFO_DIR := $(KERNEL_DIR)/Core/sysinfo
SYSINFO_OBJ := $(BUILD_KERNEL_DIR)/Core/sysinfo/SystemInfo.o

SYSINFO_SRCS := $(SYSINFO_DIR)/SystemInfo.c

$(BUILD_KERNEL_DIR)/Core/sysinfo/%.o: $(SYSINFO_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) -c $< -o $@ $(CFLAGS_KERNEL)

KERNEL_OBJS += $(SYSINFO_OBJ)
