#pragma once

#include "DriverBinary.h"

#include <stdbool.h>
#include <stdint.h>

bool audio_manager_init(void);
const char *audio_manager_name(void);
bool audio_manager_open(void);
bool audio_manager_get_info(driver_audio_info_t *out_info);
int64_t audio_manager_write(const void *pcm, uint64_t bytes);
bool audio_manager_drain(uint32_t timeout_ms);
void audio_manager_close(void);

/* Continuous playback (see driver_audio_t.stream_*). audio_manager_stream_ok
 * says whether the opened device has it. None of these block. */
bool audio_manager_stream_ok(void);
uint64_t audio_manager_stream_write(const void *pcm, uint64_t bytes);
uint64_t audio_manager_stream_queued(void);
uint64_t audio_manager_stream_space(void);
bool audio_manager_stream_start(void);
void audio_manager_stream_stop(void);
