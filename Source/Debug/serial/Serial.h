#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

typedef void (*serial_mirror_char_t)(char c);

typedef struct {
    void (*write_char)(char c);
    void (*write_string)(const char *str);
} serial_backend_t;

void serial_init(void);
void serial_set_screen_mirror(serial_mirror_char_t writer);
void serial_register_backend(const serial_backend_t *backend);
void serial_write_char(char c);
void serial_write_string(const char *str);
/* Counted write, atomic against other CPUs for its whole length -- see the
 * definition. Used for a foreign program's stdout/stderr. */
void serial_write_buffer(const char *data, uint32_t length);
void serial_write_uint64(uint64_t value);
void serial_write_uint32(uint32_t value);
void serial_write_uint16(uint16_t value);
void serial_write_uint8(uint8_t value);
void serial_write_dec16(uint16_t v);
void serial_enable_file_logging(const char *path);
uint32_t serial_copy_log(char *buffer, uint32_t buffer_size);

/* Streaming reader behind /dev/kmsg (Core/vfs/DevFS.c).
 *
 * `*cursor` is an absolute byte position in the kernel log; pass 0 to start at
 * the oldest line still held. Copies at most `max` bytes that the reader has
 * not seen yet, advances `*cursor` past them, and returns how many were
 * copied (0 = caller is up to date). A cursor that has fallen off the back of
 * the ring is snapped forward to the oldest surviving byte, so a slow reader
 * loses old lines rather than reading garbage. */
uint32_t serial_read_log(uint64_t *cursor, char *buffer, uint32_t max);

/* Total bytes ever written to the log. */
uint64_t serial_log_total(void);

/* ---- foreign-program launch log -------------------------------------------
 *
 * A second, much smaller ring carrying only the "[app] ..." lines the kernel
 * writes when a Linux-ABI program is exec'd or exits (see
 * OS_CONFIG_FOREIGN_LAUNCH_LOG). Exposed as /dev/applog.
 *
 * It exists separately from the main log because a terminal that streams the
 * *whole* kernel log into a window feeds back on itself: drawing a line is X
 * traffic, X traffic is logged, the log is drawn, and so on. Launch lines are
 * produced by exec and exit only, so nothing a terminal does can generate
 * them and the loop cannot close. Same cursor contract as
 * serial_read_log(). */
void     serial_applog_write(const char *line);
uint32_t serial_applog_read(uint64_t *cursor, char *buffer, uint32_t max);
uint64_t serial_applog_total(void);

#endif
