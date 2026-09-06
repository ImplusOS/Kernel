/*
 * Host harness for the kernel log ring in Debug/serial/Serial.c.
 *
 * What matters here is serial_read_log(): /dev/kmsg hands it an absolute byte
 * position and expects back exactly the bytes that reader has not seen, with
 * the ring's wraparound and the "reader fell behind" case handled. Getting
 * that wrong shows up as a terminal full of garbage or a silently truncated
 * log, neither of which a boot test would catch.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "interfaces/uart_hal.h"
#include "Serial.h"

/* The ring size Serial.c uses; the tests need to straddle it. */
#define RING 65536

/* Stand-in UART that counts bytes instead of touching hardware. */
static uint64_t g_uart_bytes;
static void h_init(uint32_t baud) { (void)baud; }
static void h_write_char(char c) { (void)c; ++g_uart_bytes; }
static void h_write_string(const char *s) { g_uart_bytes += strlen(s); }
static int  h_read_char(void) { return -1; }
static const uart_hal_t g_hal = { h_init, h_write_char, h_write_string,
                                  h_read_char };
const uart_hal_t *uart_hal_get(void) { return &g_hal; }

static int g_failures;
static void check(const char *what, int ok)
{
    printf("%-5s %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) ++g_failures;
}

/* Everything written so far, so reads can be compared against the truth.
 * Big enough for the whole run: the flood in check 5 alone is ~240 KiB. */
#define SHADOW_MAX (RING * 16)
static char g_shadow[SHADOW_MAX];
static uint64_t g_shadow_len;

static void emit(const char *s)
{
    size_t len = strlen(s);
    if (g_shadow_len + len >= (uint64_t)SHADOW_MAX) {
        fprintf(stderr, "harness bug: shadow buffer too small\n");
        exit(2);
    }
    serial_write_string(s);
    size_t n = len;
    /* serial_write_string does not add anything; the ring gets the bytes
     * verbatim (the \r for \n is a UART-only artifact). */
    memcpy(g_shadow + g_shadow_len, s, n);
    g_shadow_len += n;
}

/* Reads everything available and checks it against the shadow. */
static int drain_and_verify(uint64_t *cursor, uint64_t expect_from)
{
    char buf[512];
    uint64_t at = expect_from;
    for (;;) {
        uint32_t n = serial_read_log(cursor, buf, sizeof(buf));
        if (n == 0u) break;
        if (memcmp(buf, g_shadow + at, n) != 0) return 0;
        at += n;
    }
    return at == g_shadow_len && *cursor == g_shadow_len;
}

int main(void)
{
    serial_init();

    check("log starts empty", serial_log_total() == 0u);

    /* 1. A fresh reader sees everything written so far, byte for byte. */
    emit("[app] start pid=7 abi=linux path=/usr/bin/xterm\n");
    emit("[app] exit pid=7 status=0 lived=1234ms\n");
    check("total tracks bytes written", serial_log_total() == g_shadow_len);

    uint64_t cursor = 0;
    check("fresh reader gets the whole log", drain_and_verify(&cursor, 0));

    /* 2. A caught-up reader gets nothing, then exactly the new bytes. */
    char scratch[64];
    check("caught-up reader reads 0",
          serial_read_log(&cursor, scratch, sizeof(scratch)) == 0u);

    uint64_t before = g_shadow_len;
    emit("[app] start pid=9 abi=linux path=/usr/bin/fastfetch\n");
    check("only the new bytes arrive", drain_and_verify(&cursor, before));

    /* 3. Two readers are independent. */
    uint64_t a = serial_log_total();
    uint64_t b = serial_log_total();
    before = g_shadow_len;
    emit("shared line\n");
    check("reader A sees it", drain_and_verify(&a, before));
    check("reader B sees it too", drain_and_verify(&b, before));

    /* 4. Wrap the ring several times over, reading as we go. The cursor is
     *    absolute, so it must keep working past 64 KiB. */
    cursor = serial_log_total();
    int ok = 1;
    for (uint32_t i = 0; i < 3000u && ok; ++i) {
        char line[64];
        int n = snprintf(line, sizeof(line), "line %06u ................\n", i);
        (void)n;
        before = g_shadow_len;
        emit(line);
        ok = drain_and_verify(&cursor, before);
    }
    check("survives wrapping the ring", ok);
    check("wrapped past the ring size", serial_log_total() > (uint64_t)RING);

    /* 5. A reader that stopped reading long enough to fall off the back of
     *    the ring must be snapped forward to the oldest surviving byte, not
     *    handed stale or garbage data. */
    uint64_t stale = serial_log_total();
    for (uint32_t i = 0; i < 4000u; ++i) {
        emit("flooding the ring to push the slow reader off the back......\n");
    }
    check("the slow reader really did fall behind",
          serial_log_total() - stale > (uint64_t)RING);
    {
        char buf[512];
        uint32_t n = serial_read_log(&stale, buf, sizeof(buf));
        uint64_t oldest = serial_log_total() - (uint64_t)RING;
        check("fallen-behind cursor snaps to the oldest byte",
              n > 0u && stale == oldest + n &&
              memcmp(buf, g_shadow + oldest, n) == 0);
    }

    /* 6. Partial reads: a caller with a tiny buffer gets a prefix and the
     *    cursor advances by exactly that much. */
    {
        uint64_t c = serial_log_total();
        before = g_shadow_len;
        emit("abcdefghij");
        char small[4];
        uint32_t n = serial_read_log(&c, small, sizeof(small));
        check("short buffer returns a prefix",
              n == 4u && memcmp(small, "abcd", 4) == 0 && c == before + 4u);
        check("the rest follows", drain_and_verify(&c, before + 4u));
    }

    /* 7. Degenerate arguments. */
    {
        uint64_t c = 0;
        char one[1];
        check("max=0 reads nothing", serial_read_log(&c, one, 0u) == 0u);
        check("NULL buffer reads nothing", serial_read_log(&c, NULL, 8u) == 0u);
        check("NULL cursor reads nothing",
              serial_read_log(NULL, one, 1u) == 0u);
    }

    /* 8. The launch-log ring is a separate stream: kernel log writes must not
     *    appear in it, and it must follow the same cursor contract. This is
     *    what /dev/applog exposes, and the whole point of it being separate
     *    is that a terminal reading it cannot feed back into it. */
    {
        uint64_t before_app = serial_applog_total();
        emit("[usock] tx fd=0xC2 n=0x20\n");   /* noise on the main log */
        check("kernel log writes do not reach the launch ring",
              serial_applog_total() == before_app);

        uint64_t c = 0;
        serial_applog_write("[app] spawn pid=7 path=/usr/bin/xterm\n");
        serial_applog_write("[app] exit pid=7 status=0 lived=12ms\n");
        char buf[256];
        uint32_t n = serial_applog_read(&c, buf, sizeof(buf));
        check("launch ring reads back what was written",
              n == serial_applog_total() &&
              memcmp(buf, "[app] spawn pid=7 path=/usr/bin/xterm\n"
                          "[app] exit pid=7 status=0 lived=12ms\n", n) == 0);
        check("launch ring reader is then caught up",
              serial_applog_read(&c, buf, sizeof(buf)) == 0u);

        /* Wrap the small ring and confirm a fallen-behind reader snaps. */
        uint64_t slow = serial_applog_total();
        for (uint32_t i = 0; i < 400u; ++i) {
            serial_applog_write("[app] exit pid=9 status=0 lived=1ms......\n");
        }
        n = serial_applog_read(&slow, buf, sizeof(buf));
        check("launch ring survives wrapping",
              n > 0u && slow <= serial_applog_total());
    }

    printf("\n%llu bytes logged, %llu reached the UART\n",
           (unsigned long long)serial_log_total(),
           (unsigned long long)g_uart_bytes);
    printf("%s\n", g_failures == 0 ? "all serial-log checks passed"
                                   : "SERIAL LOG CHECKS FAILED");
    return g_failures == 0 ? 0 : 1;
}
