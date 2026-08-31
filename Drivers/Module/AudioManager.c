#include "AudioManager.h"

#include "Debug/serial/Serial.h"
#include "DeviceRegistry.h"

#include <stddef.h>

static const driver_audio_t *g_audio;
static bool g_open;

static const driver_audio_t *audio_manager_select(void)
{
    const driver_audio_t *best = NULL;
    for (uint32_t i = 0u;; ++i) {
        const device_t *device =
            device_registry_find_by_index(DEVICE_TYPE_AUDIO, i);
        if (device == NULL) {
            break;
        }
        const driver_audio_t *audio = (const driver_audio_t *)device->ops;
        if (audio == NULL || audio->init == NULL ||
            audio->is_ready == NULL) {
            continue;
        }
        if (!audio->init() || !audio->is_ready()) {
            continue;
        }
        if (best == NULL || audio->priority < best->priority) {
            best = audio;
        }
    }
    return best;
}

bool audio_manager_init(void)
{
    if (g_audio != NULL && g_audio->is_ready != NULL &&
        g_audio->is_ready()) {
        return true;
    }
    g_audio = audio_manager_select();
    return g_audio != NULL;
}

const char *audio_manager_name(void)
{
    return g_audio != NULL ? g_audio->name : NULL;
}

bool audio_manager_open(void)
{
    if (g_open) {
        return false;
    }
    if (!audio_manager_init() || g_audio->open == NULL || !g_audio->open()) {
        g_audio = NULL;
        return false;
    }
    g_open = true;
    return true;
}

bool audio_manager_get_info(driver_audio_info_t *out_info)
{
    return g_open && g_audio != NULL && g_audio->get_info != NULL &&
           g_audio->get_info(out_info);
}

int64_t audio_manager_write(const void *pcm, uint64_t bytes)
{
    if (!g_open || g_audio == NULL || g_audio->write == NULL ||
        pcm == NULL || bytes == 0u || (bytes & 3u) != 0u) {
        return -1;
    }
    return g_audio->write(pcm, bytes);
}

bool audio_manager_drain(uint32_t timeout_ms)
{
    return g_open && g_audio != NULL && g_audio->drain != NULL &&
           g_audio->drain(timeout_ms);
}

void audio_manager_close(void)
{
    if (g_open && g_audio != NULL && g_audio->close != NULL) {
        g_audio->close();
    }
    g_open = false;
    g_audio = NULL;
}
