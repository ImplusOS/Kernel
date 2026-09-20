#pragma once

/*
 * ALSA kernel ABI for Linux programs: /dev/snd/controlC0 and
 * /dev/snd/pcmC0D0p, on top of the audio driver's continuous-playback
 * interface (driver_audio_t.stream_*, Drivers/Module/AudioManager.h).
 *
 * Linux programs do not talk to the kernel's sound drivers directly: they
 * link alsa-lib (libasound.so.2), whose "hw" plugin speaks this ioctl
 * protocol to the device nodes. Implementing the protocol is what lets
 * unmodified binaries -- Chromium, which falls back to ALSA when no
 * PulseAudio server answers, and OpenAL/SDL -- play sound.
 *
 * One card, one playback device; up to eight opens at a time, each its own
 * substream, mixed in software. The device accepts the
 * common sample formats (U8, S16_LE, S32_LE, FLOAT_LE), 1-8 channels and any
 * rate from 8 kHz to 192 kHz, and converts to what the hardware plays
 * (48 kHz S16_LE stereo) itself, so alsa-lib needs no plugin chain.
 * Capture is not implemented.
 */

#include <stdint.h>

#include "kernel/interfaces/vfs_file.h"

/* Returned by alsa_pcm_ioctl() when the call would block: the syscall layer
 * answers EAGAIN on an O_NONBLOCK descriptor and otherwise sleeps briefly
 * and re-executes the ioctl. */
#define ALSA_WOULD_BLOCK (-512)

int alsa_card_present(void);

int alsa_ctl_open(void);
void alsa_ctl_close(void);
int64_t alsa_ctl_ioctl(uint64_t request, uint64_t arg);

/* The substream index (>= 0) the other calls take, or -errno (-16 EBUSY
 * when every substream is open). */
int alsa_pcm_open(void);
void alsa_pcm_close(int32_t index);
int64_t alsa_pcm_ioctl(int32_t index, uint64_t request, uint64_t arg);
uint32_t alsa_pcm_poll(int32_t index, uint32_t events);

/* Called from the timer interrupt: moves converted audio into the device
 * and advances the playback position. */
void alsa_timer_tick(void);
/* Called from the other CPUs' timer interrupts: pumps only when CPU0's tick
 * has not for a while (it can be held off by a long interrupts-off stretch,
 * and the device ring then replays stale audio). */
void alsa_timer_tick_backup(void);
