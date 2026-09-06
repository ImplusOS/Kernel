/*
 * pty_test.c -- host harness for the pseudo-terminal line discipline.
 *
 * Builds the real Kernel/Source/Core/tty/Pty.c against the stubs in stub/
 * (see run.sh) and drives it directly: there is no QEMU boot in this loop, so
 * every discipline rule -- canonical assembly, erase, ^C, ^D, ONLCR, the
 * hangup that tells a terminal emulator its child died -- is checked here
 * rather than one boot at a time.
 *
 * What it deliberately does NOT cover: anything that needs a real address
 * space (copy_to_user is a memcpy here) or real scheduling (the blocking
 * paths are only exercised through their non-blocking form).
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "Pty.h"

/* ---- stubbed kernel ------------------------------------------------------ */

static int32_t g_current_pid = 10;
static int32_t g_parent_of[64];
static int32_t g_last_signal;
static int32_t g_last_signal_pid;
static int     g_signal_count;
static uint64_t g_pending_signals;
static int     g_sleep_calls;

int32_t process_get_current_pid(void) { return g_current_pid; }

int32_t process_get_parent_pid(int32_t pid)
{
    if (pid < 0 || pid >= 64) return -1;
    return g_parent_of[pid];
}

int process_signal_deliver_group(int32_t pid, int32_t signum)
{
    g_last_signal_pid = pid;
    g_last_signal = signum;
    g_signal_count++;
    return 0;
}

uint64_t process_get_current_pending_signals(void) { return g_pending_signals; }
uint64_t process_signal_get_mask(void) { return 0u; }
int process_is_alive(int32_t pid) { (void)pid; return 1; }

int process_sleep_current_ms(uint64_t ms)
{
    (void)ms;
    /* Nothing in this harness may block: a test that reaches here has asked a
     * blocking read a question the harness cannot answer. Fail loudly instead
     * of hanging the run. */
    if (++g_sleep_calls > 4) {
        fprintf(stderr, "FATAL: pty blocked in a single-threaded harness\n");
        exit(2);
    }
    return 0;
}

int32_t syscall_file_open(const char *path, uint64_t flags)
{
    (void)flags;
    /* TIOCGPTPEER: the harness has no fd table, so just prove the path the
     * master builds is the one the slave lives at. */
    return (strncmp(path, "/dev/pts/", 9) == 0) ? 42 : -2;
}

/* ---- termios/ioctl mirrors (Linux asm-generic layout) -------------------- */

typedef struct {
    uint32_t c_iflag, c_oflag, c_cflag, c_lflag;
    uint8_t  c_line;
    uint8_t  c_cc[19];
} termios_t;

typedef struct { uint16_t row, col, xpixel, ypixel; } winsize_t;

#define TCGETS     0x5401u
#define TCSETS     0x5402u
#define TIOCSCTTY  0x540Eu
#define TIOCSPGRP  0x5410u
#define TIOCGWINSZ 0x5413u
#define TIOCSWINSZ 0x5414u
#define FIONREAD   0x541Bu
#define TIOCGPTN   0x80045430u
#define TIOCSPTLCK 0x40045431u
#define TIOCGPTPEER 0x5441u

#define ICANON 0x0002u
#define ECHO   0x0008u

/* ---- tiny test scaffolding ----------------------------------------------- */

static int g_failures;
static const char *g_case;

static void check(int condition, const char *what)
{
    if (!condition) {
        printf("  FAIL  %s: %s\n", g_case, what);
        g_failures++;
    }
}

static void check_bytes(const uint8_t *got, int got_len,
                        const char *want, const char *what)
{
    int want_len = (int)strlen(want);
    if (got_len != want_len || memcmp(got, want, (size_t)want_len) != 0) {
        printf("  FAIL  %s: %s (got %d bytes \"", g_case, what, got_len);
        for (int i = 0; i < got_len; ++i) {
            unsigned char c = got[i];
            if (c >= 32 && c < 127) putchar(c);
            else printf("\\x%02x", c);
        }
        printf("\", wanted \"");
        for (int i = 0; i < want_len; ++i) {
            unsigned char c = (unsigned char)want[i];
            if (c >= 32 && c < 127) putchar(c);
            else printf("\\x%02x", c);
        }
        printf("\")\n");
        g_failures++;
    }
}

/* Open a pair the way glibc's openpty() does. */
static int32_t open_pair(void)
{
    int32_t index = pty_allocate();
    check(index >= 0, "pty_allocate");
    int32_t lock = 0;
    check(pty_ioctl(index, 1, TIOCSPTLCK, (uint64_t)(uintptr_t)&lock) == 0,
          "unlockpt");
    check(pty_slave_acquire(index) == 0, "slave open");
    return index;
}

static int64_t master_write(int32_t index, const char *s)
{
    return pty_write(index, 1, (const uint8_t *)s, strlen(s), 0u);
}
static int64_t slave_write(int32_t index, const char *s)
{
    return pty_write(index, 0, (const uint8_t *)s, strlen(s), 0u);
}
static int64_t slave_read(int32_t index, uint8_t *buf, uint64_t len)
{
    return pty_read(index, 0, buf, len, 1u); /* non-blocking */
}
static int64_t master_read(int32_t index, uint8_t *buf, uint64_t len)
{
    return pty_read(index, 1, buf, len, 1u);
}

/* ---- cases --------------------------------------------------------------- */

static void case_canonical_line(void)
{
    g_case = "canonical line";
    int32_t p = open_pair();
    uint8_t buf[64];

    /* A half-typed line is not readable: canonical mode hands over whole
     * lines only. */
    check(master_write(p, "ls -l") == 5, "master write partial");
    check(slave_read(p, buf, sizeof(buf)) == -11, "partial line is EAGAIN");

    check(master_write(p, "\n") == 1, "master write newline");
    int64_t n = slave_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "ls -l\n", "slave sees the line");

    /* ...and the echo came back out of the master, newline expanded by ONLCR
     * because the echo goes through the output post-processor. */
    n = master_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "ls -l\r\n", "echo reaches master");

    pty_slave_release(p);
    pty_master_release(p);
}

static void case_erase(void)
{
    g_case = "erase";
    int32_t p = open_pair();
    uint8_t buf[64];

    check(master_write(p, "abc\x7f\n") == 5, "type abc<DEL><CR>");
    int64_t n = slave_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "ab\n", "DEL erased one character");

    n = master_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "abc\b \b\r\n", "erase echoed as \\b \\b");

    pty_slave_release(p);
    pty_master_release(p);
}

static void case_interrupt(void)
{
    g_case = "interrupt";
    int32_t p = open_pair();
    uint8_t buf[64];

    int32_t pgrp = 77;
    check(pty_ioctl(p, 0, TIOCSPGRP, (uint64_t)(uintptr_t)&pgrp) == 0,
          "tcsetpgrp");

    g_signal_count = 0;
    check(master_write(p, "half typed") == 10, "type a partial line");
    check(master_write(p, "\x03") == 1, "type ^C");
    check(g_signal_count == 1, "one signal raised");
    check(g_last_signal == 2, "signal is SIGINT");
    check(g_last_signal_pid == 77, "signal went to the foreground group");

    /* ^C discards the partial line, so nothing is left to read. */
    check(slave_read(p, buf, sizeof(buf)) == -11, "input flushed by ^C");

    int64_t n = master_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "half typed^C", "^C echoed as ^C");

    pty_slave_release(p);
    pty_master_release(p);
}

static void case_eof(void)
{
    g_case = "eof";
    int32_t p = open_pair();
    uint8_t buf[64];

    /* ^D on a non-empty line hands over what was typed, without a newline. */
    check(master_write(p, "xy\x04") == 3, "type xy^D");
    int64_t n = slave_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "xy", "^D flushes the partial line");

    /* ...and on an empty line it is end of file. */
    check(master_write(p, "\x04") == 1, "type ^D alone");
    check(slave_read(p, buf, sizeof(buf)) == 0, "bare ^D reads as EOF");

    pty_slave_release(p);
    pty_master_release(p);
}

static void case_output_post_processing(void)
{
    g_case = "onlcr";
    int32_t p = open_pair();
    uint8_t buf[64];

    check(slave_write(p, "hi\n") == 3, "shell writes hi\\n");
    int64_t n = master_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "hi\r\n", "ONLCR expanded the newline");

    pty_slave_release(p);
    pty_master_release(p);
}

static void case_raw_mode(void)
{
    g_case = "raw mode";
    int32_t p = open_pair();
    uint8_t buf[64];
    termios_t t;

    check(pty_ioctl(p, 0, TCGETS, (uint64_t)(uintptr_t)&t) == 0, "tcgetattr");
    check((t.c_lflag & ICANON) != 0u, "starts canonical");
    check(t.c_cc[2] == 127u, "VERASE defaults to DEL");

    t.c_lflag &= ~(ICANON | ECHO);
    check(pty_ioctl(p, 0, TCSETS, (uint64_t)(uintptr_t)&t) == 0, "tcsetattr raw");

    /* Raw mode hands every byte over immediately -- this is what a full-screen
     * program (vi, less) switches to. */
    check(master_write(p, "k") == 1, "type one key");
    int64_t n = slave_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "k", "raw byte is readable at once");
    check(master_read(p, buf, sizeof(buf)) == -11, "ECHO off: nothing echoed");

    pty_slave_release(p);
    pty_master_release(p);
}

static void case_winsize(void)
{
    g_case = "winsize";
    int32_t p = open_pair();
    winsize_t ws;

    check(pty_ioctl(p, 0, TIOCGWINSZ, (uint64_t)(uintptr_t)&ws) == 0, "TIOCGWINSZ");
    check(ws.row == 24u && ws.col == 80u, "defaults to 24x80");

    int32_t pgrp = 88;
    (void)pty_ioctl(p, 0, TIOCSPGRP, (uint64_t)(uintptr_t)&pgrp);
    g_signal_count = 0;

    ws.row = 50u; ws.col = 132u;
    check(pty_ioctl(p, 1, TIOCSWINSZ, (uint64_t)(uintptr_t)&ws) == 0, "TIOCSWINSZ");
    check(g_signal_count == 1 && g_last_signal == 28, "resize raised SIGWINCH");

    memset(&ws, 0, sizeof(ws));
    check(pty_ioctl(p, 0, TIOCGWINSZ, (uint64_t)(uintptr_t)&ws) == 0, "re-read size");
    check(ws.row == 50u && ws.col == 132u, "new size is visible to the slave");

    pty_slave_release(p);
    pty_master_release(p);
}

static void case_ptmx_protocol(void)
{
    g_case = "ptmx protocol";
    int32_t index = pty_allocate();
    check(index >= 0, "allocate");

    /* A fresh pair is locked, exactly like Linux: the slave cannot be opened
     * until unlockpt(). */
    check(pty_slave_is_openable(index) == 0, "slave starts locked");
    check(pty_slave_acquire(index) < 0, "locked slave refuses to open");

    uint32_t number = 0xFFFFFFFFu;
    check(pty_ioctl(index, 1, TIOCGPTN, (uint64_t)(uintptr_t)&number) == 0,
          "TIOCGPTN");
    check(number == (uint32_t)index, "TIOCGPTN reports the pair number");

    int32_t lock = 0;
    check(pty_ioctl(index, 1, TIOCSPTLCK, (uint64_t)(uintptr_t)&lock) == 0,
          "unlockpt");
    check(pty_slave_is_openable(index) == 1, "unlocked");
    check(pty_ioctl(index, 1, TIOCGPTPEER, 2u) == 42, "TIOCGPTPEER opens /dev/pts/N");

    /* The slave end must refuse the master-only requests. */
    check(pty_slave_acquire(index) == 0, "slave open");
    check(pty_ioctl(index, 0, TIOCGPTN, (uint64_t)(uintptr_t)&number) == -25,
          "TIOCGPTN on the slave is ENOTTY");

    pty_slave_release(index);
    pty_master_release(index);
}

static void case_hangup(void)
{
    g_case = "hangup";
    uint8_t buf[64];

    /* Child exits -> last slave fd closes -> the master's read reports EIO.
     * That is the event a terminal emulator closes its window on. */
    int32_t p = open_pair();
    check(master_read(p, buf, sizeof(buf)) == -11, "no output yet: EAGAIN");
    check(slave_write(p, "bye\n") == 4, "final output");
    pty_slave_release(p);
    int64_t n = master_read(p, buf, sizeof(buf));
    check_bytes(buf, (int)(n < 0 ? 0 : n), "bye\r\n", "buffered output survives");
    check(master_read(p, buf, sizeof(buf)) == -5, "drained + no slave = EIO");
    pty_master_release(p);

    /* The other direction: the emulator dies, the shell's read sees EOF. */
    p = open_pair();
    g_signal_count = 0;
    int32_t pgrp = 99;
    (void)pty_ioctl(p, 0, TIOCSPGRP, (uint64_t)(uintptr_t)&pgrp);
    pty_master_release(p);
    check(g_signal_count == 1 && g_last_signal == 1, "master close raised SIGHUP");
    check(slave_read(p, buf, sizeof(buf)) == 0, "slave read sees EOF");
    pty_slave_release(p);
}

static void case_poll_and_fionread(void)
{
    g_case = "poll";
    enum { POLLIN = 0x1u, POLLOUT = 0x4u, POLLHUP = 0x10u };
    int32_t p = open_pair();

    check((pty_poll(p, 0, POLLIN) & POLLIN) == 0u, "slave not readable when idle");
    check((pty_poll(p, 1, POLLOUT) & POLLOUT) != 0u, "master writable");

    (void)master_write(p, "x");
    check((pty_poll(p, 0, POLLIN) & POLLIN) == 0u,
          "half a line does not make the slave readable");
    (void)master_write(p, "\n");
    check((pty_poll(p, 0, POLLIN) & POLLIN) != 0u, "complete line is readable");

    int32_t available = -1;
    check(pty_ioctl(p, 0, FIONREAD, (uint64_t)(uintptr_t)&available) == 0,
          "FIONREAD");
    check(available == 2, "FIONREAD counts the pending line");

    pty_slave_release(p);
    check((pty_poll(p, 1, POLLIN) & POLLHUP) != 0u, "master sees POLLHUP");
    pty_master_release(p);
}

static void case_ctty_inheritance(void)
{
    g_case = "controlling terminal";
    int32_t p = open_pair();

    g_current_pid = 20;                 /* the shell */
    g_parent_of[20] = 10;
    check(pty_ctty_index_for_current() == -1, "no controlling terminal yet");
    check(pty_ioctl(p, 0, TIOCSCTTY, 0u) == 0, "TIOCSCTTY");
    check(pty_ctty_index_for_current() == p, "the shell has one now");

    /* A command the shell forks resolves /dev/tty through its parent. */
    g_current_pid = 21;
    g_parent_of[21] = 20;
    check(pty_ctty_index_for_current() == p, "child inherits it");

    /* ...and it goes away with the process that owned it. */
    pty_forget_process(20);
    check(pty_ctty_index_for_current() == -1, "cleared when the shell exits");

    g_current_pid = 10;
    pty_slave_release(p);
    pty_master_release(p);
}

static void case_exhaustion(void)
{
    g_case = "exhaustion";
    int32_t held[16];
    int count = 0;
    for (int i = 0; i < 16; ++i) {
        int32_t index = pty_allocate();
        if (index < 0) break;
        held[count++] = index;
    }
    check(count > 0, "at least one pair available");
    check(pty_allocate() < 0, "allocation fails once the table is full");
    for (int i = 0; i < count; ++i) {
        pty_master_release(held[i]);
    }
    check(pty_allocate() >= 0, "and recovers after they are released");
    pty_master_release(0);
}

int main(void)
{
    struct { const char *name; void (*run)(void); } cases[] = {
        { "ptmx protocol",        case_ptmx_protocol },
        { "canonical line",       case_canonical_line },
        { "erase",                case_erase },
        { "interrupt",            case_interrupt },
        { "eof",                  case_eof },
        { "onlcr",                case_output_post_processing },
        { "raw mode",             case_raw_mode },
        { "winsize",              case_winsize },
        { "hangup",               case_hangup },
        { "poll",                 case_poll_and_fionread },
        { "controlling terminal", case_ctty_inheritance },
        { "exhaustion",           case_exhaustion },
    };

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        pty_init();
        memset(g_parent_of, -1, sizeof(g_parent_of));
        g_current_pid = 10;
        g_sleep_calls = 0;
        g_pending_signals = 0u;
        int before = g_failures;
        cases[i].run();
        printf("%s  %s\n", g_failures == before ? "ok  " : "FAIL", cases[i].name);
    }

    if (g_failures != 0) {
        printf("\n%d check(s) failed\n", g_failures);
        return 1;
    }
    printf("\nall pty checks passed\n");
    return 0;
}
