#pragma once
#include <stdint.h>
int32_t  process_get_current_pid(void);
int32_t  process_get_parent_pid(int32_t pid);
int      process_signal_deliver_group(int32_t pid, int32_t signum);
uint64_t process_get_current_pending_signals(void);
uint64_t process_signal_get_mask(void);
int      process_sleep_current_ms(uint64_t ms);
int      process_is_alive(int32_t pid);
