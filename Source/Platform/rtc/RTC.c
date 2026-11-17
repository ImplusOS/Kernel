#include "RTC.h"

#ifdef IMPLUS_DRIVER_MODULE
#include "Drivers/Module/DriverBinary.h"
#else
#include "Platform/io/IO_Main.h"
#endif

#ifdef IMPLUS_DRIVER_MODULE
static const driver_binary_t *g_api = NULL;
#define outb g_api->outb
#define inb g_api->inb
#endif

#include "Core/sync/Spinlock.h"

#if !defined(__aarch64__)
static spinlock_t g_rtc_lock;

#define CMOS_ADDR 0x70
#define CMOS_DATA 0x71

static int is_update_in_progress(void) {
    outb(CMOS_ADDR, 0x0A);
    return (inb(CMOS_DATA) & 0x80);
}

static uint8_t get_rtc_register(int reg) {
    outb(CMOS_ADDR, (uint8_t)reg);
    return inb(CMOS_DATA);
}
#endif

void rtc_init(void) {
}

#if defined(__aarch64__)
/* AArch64 has no CMOS behind port 0x70/0x71 (the port-I/O HAL reads 0xFF, so
 * the update-in-progress wait below never ends). QEMU virt -- like most
 * AArch64 boards -- has an ARM PrimeCell PL031 instead, whose data register
 * is a plain count of seconds since the Unix epoch, in UTC. */
#include "interfaces/mmio.h"

#define PL031_BASE 0x09010000ULL
#define PL031_DR   0x000u

static void rtc_from_unix_seconds(uint64_t t, rtc_time_t *time)
{
    uint64_t days = t / 86400u;
    uint64_t rem = t % 86400u;
    time->hour = (uint8_t)(rem / 3600u);
    time->minute = (uint8_t)((rem % 3600u) / 60u);
    time->second = (uint8_t)(rem % 60u);

    /* Civil-from-days (H. Hinnant), proleptic Gregorian, UTC. */
    int64_t z = (int64_t)days + 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    uint64_t doe = (uint64_t)(z - era * 146097);
    uint64_t yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    int64_t y = (int64_t)yoe + era * 400;
    uint64_t doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
    uint64_t mp = (5u * doy + 2u) / 153u;
    uint64_t d = doy - (153u * mp + 2u) / 5u + 1u;
    uint64_t m = mp < 10u ? mp + 3u : mp - 9u;
    if (m <= 2u) {
        ++y;
    }
    time->day = (uint8_t)d;
    time->month = (uint8_t)m;
    time->year = (uint16_t)y;
}

void rtc_read_time(rtc_time_t *time) {
    rtc_from_unix_seconds(
        (uint64_t)mmio_read32((const volatile void *)(uintptr_t)(PL031_BASE + PL031_DR)),
        time);
}
#else
void rtc_read_time(rtc_time_t *time) {
    /* CMOS is an index/data port pair, so two CPUs reading at once can select
     * a register and read back the one the other selected -- seconds as the
     * month, and so on. Chromium's log dates wandered between January and
     * December and TLS rejected certificates as expired. Serialize the
     * access, and take two full readings until they agree, so an update that
     * lands between registers cannot mix two different times either. */
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_rtc_lock);

    uint8_t second, minute, hour, day, month, year;
    for (int attempt = 0; ; ++attempt) {
        while (is_update_in_progress());
        second = get_rtc_register(0x00);
        minute = get_rtc_register(0x02);
        hour   = get_rtc_register(0x04);
        day    = get_rtc_register(0x07);
        month  = get_rtc_register(0x08);
        year   = get_rtc_register(0x09);
        if (attempt >= 8) break;
        while (is_update_in_progress());
        if (second == get_rtc_register(0x00) && minute == get_rtc_register(0x02) &&
            hour == get_rtc_register(0x04) && day == get_rtc_register(0x07) &&
            month == get_rtc_register(0x08) && year == get_rtc_register(0x09)) {
            break;
        }
    }
    uint8_t registerB = get_rtc_register(0x0B);

    spinlock_unlock(&g_rtc_lock);
    irq_restore(irq_flags);

    if (!(registerB & 0x04)) {
        second = (uint8_t)((second & 0x0F) + ((second / 16) * 10));
        minute = (uint8_t)((minute & 0x0F) + ((minute / 16) * 10));
        hour   = (uint8_t)(((hour & 0x0F) + (((hour & 0x70) / 16) * 10)) | (hour & 0x80));
        day    = (uint8_t)((day & 0x0F) + ((day / 16) * 10));
        month  = (uint8_t)((month & 0x0F) + ((month / 16) * 10));
        year   = (uint8_t)((year & 0x0F) + ((year / 16) * 10));
    }

    if (!(registerB & 0x02) && (hour & 0x80)) {
        hour = (uint8_t)(((hour & 0x7F) + 12) % 24);
    }

    time->second = second;
    time->minute = minute;
    time->hour   = hour;
    time->day    = day;
    time->month  = month;
    time->year   = (uint16_t)(2000 + year);
}
#endif

#ifdef IMPLUS_DRIVER_MODULE
static void rtc_shutdown(void)
{
    g_api = NULL;
}

static const driver_module_descriptor_t g_rtc_module = {
    .magic = DRIVER_DESCRIPTOR_MAGIC,
    .version = DRIVER_DESCRIPTOR_VERSION,
    .kind = DEVICE_TYPE_UNKNOWN,
    .load_priority = 100u,
    .deps = { NULL },
    .driver_api = NULL,
    .shutdown = rtc_shutdown,
};

const driver_module_descriptor_t *driver_module_init(const driver_binary_t *api)
{
    if (api == NULL || api->inb == NULL || api->outb == NULL) {
        return NULL;
    }
    g_api = api;
    return &g_rtc_module;
}
#endif
