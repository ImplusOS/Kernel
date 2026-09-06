#include "Pty.h"

#include <string.h>

#include "kernel/config.h"
#include "Core/process/ProcessManager.h"
#include "Core/syscall/Poll_Wait.h"
#include "Core/sync/Spinlock.h"
#include "Core/syscall/Syscall_File.h"
#include "Core/usercopy/Usercopy.h"
#include "Debug/serial/Serial.h"

/* ---- sizing ------------------------------------------------------------- */

#define PTY_MAX_COUNT    8u
#define PTY_INPUT_SIZE   4096u
#define PTY_OUTPUT_SIZE  8192u
#define PTY_LINE_MAX     1024u
#define PTY_NCCS         19u   /* Linux asm-generic NCCS, NOT glibc's 32 */

/* ---- termios bits (Linux asm-generic/termbits.h) ------------------------- */

#define TTY_ISTRIP  0x0020u
#define TTY_INLCR   0x0040u
#define TTY_IGNCR   0x0080u
#define TTY_ICRNL   0x0100u
#define TTY_IXON    0x0400u
#define TTY_IXANY   0x0800u

#define TTY_OPOST   0x0001u
#define TTY_ONLCR   0x0004u
#define TTY_OCRNL   0x0008u

#define TTY_CS8     0x0030u
#define TTY_CREAD   0x0080u
#define TTY_HUPCL   0x0400u
#define TTY_B38400  0x000Fu

#define TTY_ISIG    0x0001u
#define TTY_ICANON  0x0002u
#define TTY_ECHO    0x0008u
#define TTY_ECHOE   0x0010u
#define TTY_ECHOK   0x0020u
#define TTY_ECHONL  0x0040u
#define TTY_NOFLSH  0x0080u
#define TTY_ECHOCTL 0x0200u
#define TTY_ECHOKE  0x0800u
#define TTY_IEXTEN  0x8000u

/* c_cc indices */
#define VINTR     0u
#define VQUIT     1u
#define VERASE    2u
#define VKILL     3u
#define VEOF      4u
#define VTIME     5u
#define VMIN      6u
#define VSTART    8u
#define VSTOP     9u
#define VSUSP    10u
#define VEOL     11u
#define VREPRINT 12u
#define VWERASE  14u
#define VLNEXT   15u
#define VEOL2    16u

/* ---- ioctls -------------------------------------------------------------- */

#define TTY_TCGETS      0x5401u
#define TTY_TCSETS      0x5402u
#define TTY_TCSETSW     0x5403u
#define TTY_TCSETSF     0x5404u
#define TTY_TCSBRK      0x5409u
#define TTY_TCXONC      0x540Au
#define TTY_TCFLSH      0x540Bu
#define TTY_TIOCSCTTY   0x540Eu
#define TTY_TIOCGPGRP   0x540Fu
#define TTY_TIOCSPGRP   0x5410u
#define TTY_TIOCOUTQ    0x5411u
#define TTY_TIOCGWINSZ  0x5413u
#define TTY_TIOCSWINSZ  0x5414u
#define TTY_FIONREAD    0x541Bu   /* == TIOCINQ */
#define TTY_TIOCPKT     0x5420u
#define TTY_TIOCNOTTY   0x5422u
#define TTY_TIOCGSID    0x5429u
#define TTY_TIOCGPTN    0x80045430u
#define TTY_TIOCSPTLCK  0x40045431u
#define TTY_TIOCGPTPEER 0x5441u
#define TTY_FIONBIO     0x5421u

/* Signals raised by the line discipline. */
#define TTY_SIGHUP    1
#define TTY_SIGINT    2
#define TTY_SIGQUIT   3
#define TTY_SIGTSTP  20
#define TTY_SIGWINCH 28

/* O_NOCTTY, in the same numeric space open(2) hands the file layer. */
#define TTY_O_NOCTTY 0x0100u

typedef struct {
    uint32_t c_iflag;
    uint32_t c_oflag;
    uint32_t c_cflag;
    uint32_t c_lflag;
    uint8_t  c_line;
    uint8_t  c_cc[PTY_NCCS];
} linux_termios_t;

typedef struct {
    uint16_t ws_row;
    uint16_t ws_col;
    uint16_t ws_xpixel;
    uint16_t ws_ypixel;
} linux_winsize_t;

typedef struct {
    uint8_t  used;
    uint8_t  locked;            /* TIOCSPTLCK; a fresh pair starts locked */
    uint8_t  master_open;
    uint8_t  slave_ever_opened;
    uint8_t  eof_pending;       /* ^D pressed: next drained read reports EOF */
    uint8_t  packet_mode;
    uint8_t  output_stopped;    /* ^S until ^Q (IXON) */
    uint16_t slave_open;        /* one per open file description */

    uint8_t  in_buf[PTY_INPUT_SIZE];
    uint32_t in_head;
    uint32_t in_count;

    uint8_t  out_buf[PTY_OUTPUT_SIZE];
    uint32_t out_head;
    uint32_t out_count;

    uint8_t  line[PTY_LINE_MAX];
    uint32_t line_len;

    uint32_t c_iflag;
    uint32_t c_oflag;
    uint32_t c_cflag;
    uint32_t c_lflag;
    uint8_t  c_line;
    uint8_t  c_cc[PTY_NCCS];

    linux_winsize_t win;

    int32_t foreground_pgid;
    int32_t session_pid;
} pty_t;

static pty_t g_ptys[PTY_MAX_COUNT];
static spinlock_t g_pty_table_lock;

/* Controlling terminal per pid. -1 = none. Set by TIOCSCTTY (and by opening a
 * slave without O_NOCTTY when the process has none), cleared on process exit
 * and by TIOCNOTTY. Children inherit through the parent walk in
 * pty_ctty_index_for_current() rather than by copying, so a fork that happens
 * after the parent bound its terminal still resolves /dev/tty. */
static int8_t g_ctty[OS_CONFIG_PROCESS_MAX_COUNT];

/* ---- small helpers ------------------------------------------------------- */

static pty_t *pty_at(int32_t index)
{
    if (index < 0 || (uint32_t)index >= PTY_MAX_COUNT) {
        return NULL;
    }
    pty_t *pty = &g_ptys[index];
    return pty->used != 0u ? pty : NULL;
}

static void pty_ring_push(uint8_t *buf, uint32_t capacity, uint32_t head,
                          uint32_t *count, uint8_t value)
{
    if (*count >= capacity) {
        return; /* caller checked; belt and braces */
    }
    buf[(head + *count) % capacity] = value;
    *count += 1u;
}

static void pty_input_push(pty_t *pty, uint8_t value)
{
    if (pty->in_count >= PTY_INPUT_SIZE) {
        return; /* input overrun: drop, like a real UART */
    }
    pty_ring_push(pty->in_buf, PTY_INPUT_SIZE, pty->in_head,
                  &pty->in_count, value);
}

/* Raw byte onto the master-visible stream (no post-processing). */
static void pty_output_push_raw(pty_t *pty, uint8_t value)
{
    if (pty->out_count >= PTY_OUTPUT_SIZE) {
        return;
    }
    pty_ring_push(pty->out_buf, PTY_OUTPUT_SIZE, pty->out_head,
                  &pty->out_count, value);
}

/* One byte through the output post-processor. ONLCR is what keeps a shell's
 * output from stair-stepping down the screen: the slave writes "\n" and the
 * terminal needs "\r\n". */
static void pty_output_push_cooked(pty_t *pty, uint8_t value)
{
    if ((pty->c_oflag & TTY_OPOST) != 0u) {
        if (value == (uint8_t)'\n' && (pty->c_oflag & TTY_ONLCR) != 0u) {
            pty_output_push_raw(pty, (uint8_t)'\r');
        } else if (value == (uint8_t)'\r' && (pty->c_oflag & TTY_OCRNL) != 0u) {
            value = (uint8_t)'\n';
        }
    }
    pty_output_push_raw(pty, value);
}

/* Echo of a character the user typed. Control characters come back as ^X when
 * ECHOCTL is set, which is what makes ^C visible at the prompt. */
static void pty_echo(pty_t *pty, uint8_t value)
{
    if ((pty->c_lflag & TTY_ECHO) == 0u) {
        return;
    }
    if (value < 32u && value != (uint8_t)'\n' && value != (uint8_t)'\t' &&
        value != (uint8_t)'\r' && (pty->c_lflag & TTY_ECHOCTL) != 0u) {
        pty_output_push_cooked(pty, (uint8_t)'^');
        pty_output_push_cooked(pty, (uint8_t)(value + 64u));
        return;
    }
    pty_output_push_cooked(pty, value);
}

static void pty_echo_erase(pty_t *pty)
{
    if ((pty->c_lflag & (TTY_ECHO | TTY_ECHOE)) == (TTY_ECHO | TTY_ECHOE)) {
        pty_output_push_raw(pty, (uint8_t)'\b');
        pty_output_push_raw(pty, (uint8_t)' ');
        pty_output_push_raw(pty, (uint8_t)'\b');
    }
}

static void pty_flush_input(pty_t *pty)
{
    pty->in_head = 0u;
    pty->in_count = 0u;
    pty->line_len = 0u;
    pty->eof_pending = 0u;
}

static void pty_commit_line(pty_t *pty)
{
    for (uint32_t i = 0; i < pty->line_len; ++i) {
        pty_input_push(pty, pty->line[i]);
    }
    pty->line_len = 0u;
}

static void pty_set_default_termios(pty_t *pty)
{
    pty->c_iflag = TTY_ICRNL | TTY_IXON;
    pty->c_oflag = TTY_OPOST | TTY_ONLCR;
    pty->c_cflag = TTY_B38400 | TTY_CS8 | TTY_CREAD | TTY_HUPCL;
    pty->c_lflag = TTY_ISIG | TTY_ICANON | TTY_ECHO | TTY_ECHOE | TTY_ECHOK |
                   TTY_ECHOCTL | TTY_ECHOKE | TTY_IEXTEN;
    pty->c_line = 0u;
    memset(pty->c_cc, 0, sizeof(pty->c_cc));
    pty->c_cc[VINTR]    = 3u;    /* ^C */
    pty->c_cc[VQUIT]    = 28u;   /* ^\ */
    pty->c_cc[VERASE]   = 127u;  /* DEL */
    pty->c_cc[VKILL]    = 21u;   /* ^U */
    pty->c_cc[VEOF]     = 4u;    /* ^D */
    pty->c_cc[VTIME]    = 0u;
    pty->c_cc[VMIN]     = 1u;
    pty->c_cc[VSTART]   = 17u;   /* ^Q */
    pty->c_cc[VSTOP]    = 19u;   /* ^S */
    pty->c_cc[VSUSP]    = 26u;   /* ^Z */
    pty->c_cc[VEOL]     = 0u;
    pty->c_cc[VREPRINT] = 18u;   /* ^R */
    pty->c_cc[VWERASE]  = 23u;   /* ^W */
    pty->c_cc[VLNEXT]   = 22u;   /* ^V */
    pty->c_cc[VEOL2]    = 0u;
}

void pty_init(void)
{
    memset(g_ptys, 0, sizeof(g_ptys));
    spinlock_init(&g_pty_table_lock);
    for (uint32_t i = 0; i < (uint32_t)OS_CONFIG_PROCESS_MAX_COUNT; ++i) {
        g_ctty[i] = -1;
    }
}

/* ---- lifetime ------------------------------------------------------------ */

int32_t pty_allocate(void)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    for (uint32_t i = 0; i < PTY_MAX_COUNT; ++i) {
        if (g_ptys[i].used == 0u) {
            pty_t *pty = &g_ptys[i];
            memset(pty, 0, sizeof(*pty));
            pty->used = 1u;
            /* Linux hands out a locked slave; unlockpt() clears it. Enforcing
             * it is what keeps a second process from opening the slave in the
             * window before the owner has configured the pair. */
            pty->locked = 1u;
            pty->master_open = 1u;
            pty->foreground_pgid = -1;
            pty->session_pid = -1;
            pty->win.ws_row = 24u;
            pty->win.ws_col = 80u;
            pty_set_default_termios(pty);
            spinlock_unlock(&g_pty_table_lock);
            irq_restore(irq_flags);
            return (int32_t)i;
        }
    }
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
    return PTY_ERR_NOMEM;
}

static void pty_free_if_idle(pty_t *pty)
{
    if (pty->master_open == 0u && pty->slave_open == 0u) {
        pty->used = 0u;
    }
}

void pty_master_release(int32_t index)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    pty_t *pty = pty_at(index);
    if (pty != NULL) {
        pty->master_open = 0u;
        /* Closing the master hangs up the session, exactly like a modem
         * dropping carrier: the shell's next read sees EOF and it exits. */
        int32_t target = pty->foreground_pgid;
        pty_free_if_idle(pty);
        spinlock_unlock(&g_pty_table_lock);
        irq_restore(irq_flags);
        if (target > 0) {
            (void)process_signal_deliver_group(target, TTY_SIGHUP);
        }
        return;
    }
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
}

int32_t pty_slave_acquire(int32_t index)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    pty_t *pty = pty_at(index);
    if (pty == NULL || pty->locked != 0u) {
        spinlock_unlock(&g_pty_table_lock);
        irq_restore(irq_flags);
        return PTY_ERR_IO;
    }
    pty->slave_open = (uint16_t)(pty->slave_open + 1u);
    pty->slave_ever_opened = 1u;
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
    return 0;
}

void pty_slave_release(int32_t index)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    pty_t *pty = pty_at(index);
    if (pty != NULL) {
        if (pty->slave_open > 0u) {
            pty->slave_open = (uint16_t)(pty->slave_open - 1u);
        }
        pty_free_if_idle(pty);
    }
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
}

int pty_is_valid_index(int32_t index)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    int valid = pty_at(index) != NULL ? 1 : 0;
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
    return valid;
}

int pty_slave_is_openable(int32_t index)
{
    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    pty_t *pty = pty_at(index);
    int ok = (pty != NULL && pty->locked == 0u) ? 1 : 0;
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
    return ok;
}

int32_t pty_index_from_path(const char *path)
{
    static const char prefix[] = "/dev/pts/";
    if (path == NULL || strncmp(path, prefix, sizeof(prefix) - 1u) != 0) {
        return -1;
    }
    const char *digits = path + (sizeof(prefix) - 1u);
    if (*digits == '\0') {
        return -1;
    }
    uint32_t value = 0u;
    for (const char *p = digits; *p != '\0'; ++p) {
        if (*p < '0' || *p > '9') {
            return -1;
        }
        value = value * 10u + (uint32_t)(*p - '0');
        if (value >= PTY_MAX_COUNT) {
            return -1;
        }
    }
    return (int32_t)value;
}

/* ---- controlling terminal ------------------------------------------------ */

static int ctty_pid_is_valid(int32_t pid)
{
    return pid >= 0 && pid < OS_CONFIG_PROCESS_MAX_COUNT;
}

void pty_ctty_bind_current(int32_t index)
{
    int32_t pid = process_get_current_pid();
    if (!ctty_pid_is_valid(pid) || !pty_is_valid_index(index)) {
        return;
    }
    g_ctty[pid] = (int8_t)index;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    pty_t *pty = pty_at(index);
    if (pty != NULL) {
        if (pty->session_pid < 0) {
            pty->session_pid = pid;
        }
        if (pty->foreground_pgid < 0) {
            pty->foreground_pgid = pid;
        }
    }
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
}

int32_t pty_ctty_index_for_current(void)
{
    int32_t pid = process_get_current_pid();
    /* Bounded walk: a process inherits its parent's controlling terminal, and
     * the chain is short (xterm -> shell -> command). The bound is what keeps
     * a corrupt parent link from spinning here. */
    for (uint32_t depth = 0; depth < 16u; ++depth) {
        if (!ctty_pid_is_valid(pid)) {
            break;
        }
        int32_t index = g_ctty[pid];
        if (index >= 0 && pty_is_valid_index(index)) {
            return index;
        }
        int32_t parent = process_get_parent_pid(pid);
        if (parent < 0 || parent == pid) {
            break;
        }
        pid = parent;
    }
    return -1;
}

void pty_forget_process(int32_t pid)
{
    if (!ctty_pid_is_valid(pid)) {
        return;
    }
    g_ctty[pid] = -1;

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    for (uint32_t i = 0; i < PTY_MAX_COUNT; ++i) {
        pty_t *pty = &g_ptys[i];
        if (pty->used == 0u) {
            continue;
        }
        if (pty->foreground_pgid == pid) {
            pty->foreground_pgid = -1;
        }
        if (pty->session_pid == pid) {
            pty->session_pid = -1;
        }
    }
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);
}

/* ---- line discipline ----------------------------------------------------- */

/* Feeds one byte typed on the master into the slave's input. Returns a signal
 * number to raise once the lock is dropped, or 0. Must be called with
 * the pty table lock held. */
static int pty_input_byte(pty_t *pty, uint8_t value)
{
    int pending_signal = 0;

    if ((pty->c_iflag & TTY_ISTRIP) != 0u) {
        value &= 0x7Fu;
    }
    if (value == (uint8_t)'\r') {
        if ((pty->c_iflag & TTY_IGNCR) != 0u) {
            return 0;
        }
        if ((pty->c_iflag & TTY_ICRNL) != 0u) {
            value = (uint8_t)'\n';
        }
    } else if (value == (uint8_t)'\n' && (pty->c_iflag & TTY_INLCR) != 0u) {
        value = (uint8_t)'\r';
    }

    if ((pty->c_iflag & TTY_IXON) != 0u) {
        if (value == pty->c_cc[VSTOP] && pty->c_cc[VSTOP] != 0u) {
            pty->output_stopped = 1u;
            return 0;
        }
        if (value == pty->c_cc[VSTART] && pty->c_cc[VSTART] != 0u) {
            pty->output_stopped = 0u;
            return 0;
        }
    }

    if ((pty->c_lflag & TTY_ISIG) != 0u) {
        int signal_number = 0;
        if (value == pty->c_cc[VINTR] && pty->c_cc[VINTR] != 0u) {
            signal_number = TTY_SIGINT;
        } else if (value == pty->c_cc[VQUIT] && pty->c_cc[VQUIT] != 0u) {
            signal_number = TTY_SIGQUIT;
        } else if (value == pty->c_cc[VSUSP] && pty->c_cc[VSUSP] != 0u) {
            signal_number = TTY_SIGTSTP;
        }
        if (signal_number != 0) {
            pty_echo(pty, value);
            if ((pty->c_lflag & TTY_NOFLSH) == 0u) {
                pty_flush_input(pty);
            }
            return signal_number;
        }
    }

    if ((pty->c_lflag & TTY_ICANON) == 0u) {
        pty_input_push(pty, value);
        pty_echo(pty, value);
        return pending_signal;
    }

    /* Canonical mode: assemble a line, and only hand it over once it is
     * complete. Everything a reader can see is therefore whole lines, which
     * is what makes the "read up to and including \n" rule in pty_read()
     * correct. */
    if (value == pty->c_cc[VERASE] || value == 8u) {
        if (pty->line_len > 0u) {
            pty->line_len -= 1u;
            pty_echo_erase(pty);
        }
        return 0;
    }
    if (value == pty->c_cc[VKILL] && pty->c_cc[VKILL] != 0u) {
        while (pty->line_len > 0u) {
            pty->line_len -= 1u;
            pty_echo_erase(pty);
        }
        return 0;
    }
    if ((pty->c_lflag & TTY_IEXTEN) != 0u &&
        value == pty->c_cc[VWERASE] && pty->c_cc[VWERASE] != 0u) {
        while (pty->line_len > 0u && pty->line[pty->line_len - 1u] == (uint8_t)' ') {
            pty->line_len -= 1u;
            pty_echo_erase(pty);
        }
        while (pty->line_len > 0u && pty->line[pty->line_len - 1u] != (uint8_t)' ') {
            pty->line_len -= 1u;
            pty_echo_erase(pty);
        }
        return 0;
    }
    if (value == pty->c_cc[VEOF] && pty->c_cc[VEOF] != 0u) {
        /* ^D sends what has been typed so far; on an empty line that is a
         * zero-byte handover, which is end-of-file for the reader. */
        pty_commit_line(pty);
        pty->eof_pending = 1u;
        return 0;
    }
    if (value == (uint8_t)'\n' ||
        (pty->c_cc[VEOL] != 0u && value == pty->c_cc[VEOL]) ||
        (pty->c_cc[VEOL2] != 0u && value == pty->c_cc[VEOL2])) {
        if (pty->line_len < PTY_LINE_MAX) {
            pty->line[pty->line_len++] = (uint8_t)'\n';
        }
        pty_echo(pty, (uint8_t)'\n');
        pty_commit_line(pty);
        return 0;
    }

    if (pty->line_len < PTY_LINE_MAX) {
        pty->line[pty->line_len++] = value;
        pty_echo(pty, value);
    }
    return pending_signal;
}

/* ---- readability --------------------------------------------------------- */

/* Bytes a slave read can take right now. In canonical mode that is 0 until a
 * whole line (or a ^D) has landed. Called with the pty table lock held. */
static uint32_t pty_slave_readable_locked(const pty_t *pty)
{
    if (pty->in_count == 0u) {
        return 0u;
    }
    if ((pty->c_lflag & TTY_ICANON) == 0u) {
        return pty->in_count;
    }
    for (uint32_t i = 0; i < pty->in_count; ++i) {
        if (pty->in_buf[(pty->in_head + i) % PTY_INPUT_SIZE] == (uint8_t)'\n') {
            return i + 1u;
        }
    }
    return pty->eof_pending != 0u ? pty->in_count : 0u;
}

/* ---- bring-up trace ------------------------------------------------------ */

/* The first few transfers in each direction, with the bytes shown. A terminal
 * that comes up blank has exactly two possible causes -- the shell never wrote
 * a prompt, or the emulator never read it -- and nothing outside the pty can
 * tell them apart. Capped hard: COM1 is driven one character per syscall from
 * inside the syscall path, so an uncapped trace changes the timing it is
 * measuring. */
#define PTY_TRACE_MAX 6u

static void pty_trace_bytes(const char *what, int32_t index,
                            const uint8_t *data, uint32_t length)
{
    static uint32_t emitted;
    if (!OS_CONFIG_FOREIGN_TRACE || emitted >= PTY_TRACE_MAX) {
        return;
    }
    emitted += 1u;
    serial_write_string("[pty] ");
    serial_write_string(what);
    serial_write_string(" pair=");
    serial_write_uint32((uint32_t)index);
    serial_write_string(" n=");
    serial_write_uint32(length);
    serial_write_string(" \"");
    uint32_t show = length < 24u ? length : 24u;
    for (uint32_t i = 0; i < show; ++i) {
        uint8_t c = data[i];
        if (c >= 32u && c < 127u) {
            serial_write_char((char)c);
        } else {
            serial_write_char('.');
        }
    }
    serial_write_string("\"\n");
}

/* ---- read / write -------------------------------------------------------- */

static int pty_signal_is_pending(void)
{
    uint64_t pending = process_get_current_pending_signals();
    uint64_t masked = process_signal_get_mask();
    return (pending & ~masked) != 0u ? 1 : 0;
}

/* Drains up to `length` bytes out of one end. Returns the count, 0 for EOF, or
 * a negative errno. `user_buffer` is a user pointer (dev_read contract). */
int64_t pty_read(int32_t index, int is_master, uint8_t *user_buffer,
                 uint64_t length, uint32_t nonblock)
{
    if (user_buffer == NULL) {
        return PTY_ERR_FAULT;
    }
    if (length == 0u) {
        return 0;
    }

    uint32_t idle_iterations = 0u;
    for (;;) {
        /* Before looking at the ring, so a write that lands while this
         * iteration is running cancels the sleep below (Poll_Wait.h). */
        uint64_t generation = poll_wait_generation();
        uint8_t staged[256];
        uint32_t staged_count = 0u;
        int64_t immediate = 0;
        int done = 0;

        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_pty_table_lock);
        pty_t *pty = pty_at(index);
        if (pty == NULL) {
            spinlock_unlock(&g_pty_table_lock);
            irq_restore(irq_flags);
            return PTY_ERR_IO;
        }

        uint32_t want = (length < sizeof(staged)) ? (uint32_t)length
                                                  : (uint32_t)sizeof(staged);
        if (is_master) {
            uint32_t available = pty->out_count;
            if (available > 0u) {
                uint32_t take = available < want ? available : want;
                for (uint32_t i = 0; i < take; ++i) {
                    staged[i] = pty->out_buf[(pty->out_head + i) % PTY_OUTPUT_SIZE];
                }
                pty->out_head = (pty->out_head + take) % PTY_OUTPUT_SIZE;
                pty->out_count -= take;
                staged_count = take;
                done = 1;
            } else if (pty->slave_ever_opened != 0u && pty->slave_open == 0u) {
                /* Every slave fd is gone and the buffer is drained: the child
                 * has exited. Linux reports EIO here, and that is the signal
                 * a terminal emulator waits for to close its window. */
                immediate = PTY_ERR_IO;
                done = 1;
            }
        } else {
            uint32_t readable = pty_slave_readable_locked(pty);
            if (readable > 0u) {
                uint32_t take = readable < want ? readable : want;
                for (uint32_t i = 0; i < take; ++i) {
                    staged[i] = pty->in_buf[(pty->in_head + i) % PTY_INPUT_SIZE];
                }
                pty->in_head = (pty->in_head + take) % PTY_INPUT_SIZE;
                pty->in_count -= take;
                staged_count = take;
                done = 1;
            } else if (pty->eof_pending != 0u) {
                pty->eof_pending = 0u;
                immediate = 0;
                done = 1;
            } else if (pty->master_open == 0u) {
                immediate = 0; /* hangup: EOF */
                done = 1;
            } else if ((pty->c_lflag & TTY_ICANON) == 0u &&
                       pty->c_cc[VMIN] == 0u) {
                immediate = 0; /* polling read, nothing to say */
                done = 1;
            }
        }
        spinlock_unlock(&g_pty_table_lock);
        irq_restore(irq_flags);

        if (done) {
            if (staged_count == 0u) {
                return immediate;
            }
            pty_trace_bytes(is_master ? "master read" : "slave read",
                            index, staged, staged_count);
            if (copy_to_user(user_buffer, staged, staged_count) != 0u) {
                return PTY_ERR_FAULT;
            }
            /* Draining the ring makes the other end writable again. */
            poll_wait_notify();
            return (int64_t)staged_count;
        }

        if (nonblock != 0u) {
            return PTY_ERR_AGAIN;
        }
        if (pty_signal_is_pending()) {
            return PTY_ERR_INTR;
        }
        /* A terminal is idle most of its life, so back the poll off once it is
         * clear nothing is coming; 1 ms per iteration forever would burn a
         * timeslice per millisecond in every shell sitting at a prompt. The
         * back-off is only a ceiling -- pty_write() cuts it short the moment
         * a byte arrives, so keystroke latency does not grow with idle time. */
        (void)poll_wait_park(generation, idle_iterations < 64u ? 1u : 10u);
        if (idle_iterations < 64u) {
            idle_iterations += 1u;
        }
    }
}

/* `kernel_buffer` is kernel memory (the write_at contract: syscall_file_write
 * stages the user bytes for us). */
int64_t pty_write(int32_t index, int is_master, const uint8_t *kernel_buffer,
                  uint64_t length, uint32_t nonblock)
{
    if (kernel_buffer == NULL) {
        return PTY_ERR_FAULT;
    }
    if (length == 0u) {
        return 0;
    }

    uint64_t written = 0u;
    uint32_t idle_iterations = 0u;
    for (;;) {
        uint64_t generation = poll_wait_generation();
        int pending_signal = 0;
        int32_t signal_target = -1;
        int done = 0;
        int64_t failure = 0;

        uint64_t irq_flags = irq_save_disable();
        spinlock_lock(&g_pty_table_lock);
        pty_t *pty = pty_at(index);
        if (pty == NULL) {
            spinlock_unlock(&g_pty_table_lock);
            irq_restore(irq_flags);
            return written != 0u ? (int64_t)written : PTY_ERR_IO;
        }

        if (is_master) {
            /* Master -> slave: keyboard input, through the line discipline. */
            while (written < length && pty->in_count < PTY_INPUT_SIZE &&
                   pending_signal == 0) {
                pending_signal = pty_input_byte(pty, kernel_buffer[written]);
                written += 1u;
            }
            signal_target = pty->foreground_pgid;
            if (written >= length || pending_signal != 0) {
                done = 1;
            }
        } else {
            if (pty->master_open == 0u) {
                failure = written != 0u ? (int64_t)written : PTY_ERR_IO;
                done = 1;
            } else if (pty->output_stopped == 0u) {
                /* Slave -> master: program output, post-processed. Two output
                 * slots per input byte because ONLCR can double a newline. */
                while (written < length &&
                       pty->out_count + 2u <= PTY_OUTPUT_SIZE) {
                    pty_output_push_cooked(pty, kernel_buffer[written]);
                    written += 1u;
                }
                if (written >= length) {
                    done = 1;
                }
            }
        }
        spinlock_unlock(&g_pty_table_lock);
        irq_restore(irq_flags);

        if (written > 0u) {
            pty_trace_bytes(is_master ? "master write" : "slave write",
                            index, kernel_buffer, (uint32_t)written);
            /* The other end of the pty is now readable. */
            poll_wait_notify();
        }
        if (pending_signal != 0 && signal_target > 0) {
            (void)process_signal_deliver_group(signal_target, pending_signal);
        }
        if (done) {
            return failure != 0 ? failure : (int64_t)written;
        }
        if (written > 0u && nonblock != 0u) {
            return (int64_t)written;
        }
        if (nonblock != 0u) {
            return PTY_ERR_AGAIN;
        }
        if (pty_signal_is_pending()) {
            return written != 0u ? (int64_t)written : PTY_ERR_INTR;
        }
        (void)poll_wait_park(generation, idle_iterations < 64u ? 1u : 10u);
        if (idle_iterations < 64u) {
            idle_iterations += 1u;
        }
    }
}

uint32_t pty_poll(int32_t index, int is_master, uint32_t events)
{
    enum { POLL_IN = 0x0001u, POLL_OUT = 0x0004u, POLL_HUP = 0x0010u };

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    pty_t *pty = pty_at(index);
    if (pty == NULL) {
        spinlock_unlock(&g_pty_table_lock);
        irq_restore(irq_flags);
        return POLL_HUP;
    }

    uint32_t ready = 0u;
    if (is_master) {
        if (pty->out_count > 0u) {
            ready |= POLL_IN;
        }
        if (pty->slave_ever_opened != 0u && pty->slave_open == 0u) {
            /* Hung up. Readable as well as POLLHUP, and while buffered output
             * is still draining: a poll loop that only asks for POLLIN
             * (xterm's does) has to be woken either way, first to collect the
             * last of the output and then to collect the EIO -- otherwise the
             * window never notices the shell died. */
            ready |= POLL_IN | POLL_HUP;
        }
        if (pty->in_count < PTY_INPUT_SIZE) {
            ready |= POLL_OUT;
        }
    } else {
        if (pty_slave_readable_locked(pty) > 0u || pty->eof_pending != 0u) {
            ready |= POLL_IN;
        }
        if (pty->master_open == 0u) {
            ready |= POLL_IN | POLL_HUP;
        }
        if (pty->out_count + 2u <= PTY_OUTPUT_SIZE) {
            ready |= POLL_OUT;
        }
    }
    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);

    return ready & (events | POLL_HUP);
}

/* ---- ioctl --------------------------------------------------------------- */

static int64_t pty_ioctl_termios_get(pty_t *pty, uint64_t arg)
{
    linux_termios_t termios;
    memset(&termios, 0, sizeof(termios));
    termios.c_iflag = pty->c_iflag;
    termios.c_oflag = pty->c_oflag;
    termios.c_cflag = pty->c_cflag;
    termios.c_lflag = pty->c_lflag;
    termios.c_line = pty->c_line;
    memcpy(termios.c_cc, pty->c_cc, sizeof(termios.c_cc));
    if (copy_to_user((void *)(uintptr_t)arg, &termios, sizeof(termios)) != 0u) {
        return PTY_ERR_FAULT;
    }
    return 0;
}

static int64_t pty_ioctl_termios_set(pty_t *pty, uint64_t arg, int flush_input)
{
    linux_termios_t termios;
    if (copy_from_user(&termios, (const void *)(uintptr_t)arg,
                       sizeof(termios)) != 0u) {
        return PTY_ERR_FAULT;
    }
    pty->c_iflag = termios.c_iflag;
    pty->c_oflag = termios.c_oflag;
    pty->c_cflag = termios.c_cflag;
    pty->c_lflag = termios.c_lflag;
    pty->c_line = termios.c_line;
    memcpy(pty->c_cc, termios.c_cc, sizeof(pty->c_cc));
    if (flush_input) {
        pty_flush_input(pty);
    }
    /* A mode switch abandons whatever half-typed line was being assembled:
     * the bytes belong to the old discipline. */
    pty->line_len = 0u;
    return 0;
}

int64_t pty_ioctl(int32_t index, int is_master, uint64_t request, uint64_t arg)
{
    /* TIOCGPTPEER opens a file, which cannot happen under the table lock. */
    if (is_master && (uint32_t)request == TTY_TIOCGPTPEER) {
        char path[32];
        char digits[8];
        uint32_t value = (uint32_t)index;
        uint32_t digit_count = 0u;
        do {
            digits[digit_count++] = (char)('0' + (value % 10u));
            value /= 10u;
        } while (value != 0u && digit_count < sizeof(digits));
        memcpy(path, "/dev/pts/", 9u);
        for (uint32_t i = 0; i < digit_count; ++i) {
            path[9u + i] = digits[digit_count - 1u - i];
        }
        path[9u + digit_count] = '\0';
        return (int64_t)syscall_file_open(path, arg);
    }

    uint64_t irq_flags = irq_save_disable();
    spinlock_lock(&g_pty_table_lock);
    pty_t *pty = pty_at(index);
    if (pty == NULL) {
        spinlock_unlock(&g_pty_table_lock);
        irq_restore(irq_flags);
        return PTY_ERR_IO;
    }

    int64_t result;
    int32_t bind_ctty = -1;
    int32_t unbind_ctty = 0;

    switch ((uint32_t)request) {
        case TTY_TCGETS:
            result = pty_ioctl_termios_get(pty, arg);
            break;
        case TTY_TCSETS:
        case TTY_TCSETSW:
            result = pty_ioctl_termios_set(pty, arg, 0);
            break;
        case TTY_TCSETSF:
            result = pty_ioctl_termios_set(pty, arg, 1);
            break;
        case TTY_TCFLSH:
            /* arg: 0 = input, 1 = output, 2 = both. */
            if (arg == 0u || arg == 2u) {
                pty_flush_input(pty);
            }
            if (arg == 1u || arg == 2u) {
                pty->out_head = 0u;
                pty->out_count = 0u;
            }
            result = 0;
            break;
        case TTY_TCSBRK:
        case TTY_TCXONC:
            result = 0;
            break;
        case TTY_TIOCGWINSZ:
            result = copy_to_user((void *)(uintptr_t)arg, &pty->win,
                                  sizeof(pty->win)) == 0u ? 0 : PTY_ERR_FAULT;
            break;
        case TTY_TIOCSWINSZ: {
            linux_winsize_t win;
            if (copy_from_user(&win, (const void *)(uintptr_t)arg,
                               sizeof(win)) != 0u) {
                result = PTY_ERR_FAULT;
                break;
            }
            int changed = (win.ws_row != pty->win.ws_row ||
                           win.ws_col != pty->win.ws_col);
            pty->win = win;
            /* A resized window has to reach the program that is drawing in it;
             * that is the whole reason SIGWINCH exists. */
            if (changed && pty->foreground_pgid > 0) {
                int32_t target = pty->foreground_pgid;
                spinlock_unlock(&g_pty_table_lock);
                irq_restore(irq_flags);
                (void)process_signal_deliver_group(target, TTY_SIGWINCH);
                return 0;
            }
            result = 0;
            break;
        }
        case TTY_TIOCGPGRP: {
            int32_t pgrp = pty->foreground_pgid > 0 ? pty->foreground_pgid
                                                    : process_get_current_pid();
            result = copy_to_user((void *)(uintptr_t)arg, &pgrp,
                                  sizeof(pgrp)) == 0u ? 0 : PTY_ERR_FAULT;
            break;
        }
        case TTY_TIOCSPGRP: {
            int32_t pgrp = 0;
            if (copy_from_user(&pgrp, (const void *)(uintptr_t)arg,
                               sizeof(pgrp)) != 0u) {
                result = PTY_ERR_FAULT;
                break;
            }
            if (pgrp <= 0) {
                result = PTY_ERR_INVAL;
                break;
            }
            pty->foreground_pgid = pgrp;
            result = 0;
            break;
        }
        case TTY_TIOCGSID: {
            int32_t sid = pty->session_pid > 0 ? pty->session_pid
                                               : process_get_current_pid();
            result = copy_to_user((void *)(uintptr_t)arg, &sid,
                                  sizeof(sid)) == 0u ? 0 : PTY_ERR_FAULT;
            break;
        }
        case TTY_TIOCSCTTY:
            if (is_master) {
                result = PTY_ERR_NOTTY;
                break;
            }
            bind_ctty = index;
            result = 0;
            break;
        case TTY_TIOCNOTTY:
            unbind_ctty = 1;
            result = 0;
            break;
        case TTY_TIOCGPTN: {
            if (!is_master) {
                result = PTY_ERR_NOTTY;
                break;
            }
            uint32_t number = (uint32_t)index;
            result = copy_to_user((void *)(uintptr_t)arg, &number,
                                  sizeof(number)) == 0u ? 0 : PTY_ERR_FAULT;
            break;
        }
        case TTY_TIOCSPTLCK: {
            if (!is_master) {
                result = PTY_ERR_NOTTY;
                break;
            }
            int32_t lock = 0;
            if (copy_from_user(&lock, (const void *)(uintptr_t)arg,
                               sizeof(lock)) != 0u) {
                result = PTY_ERR_FAULT;
                break;
            }
            pty->locked = (lock != 0) ? 1u : 0u;
            result = 0;
            break;
        }
        case TTY_TIOCPKT: {
            int32_t enable = 0;
            if (copy_from_user(&enable, (const void *)(uintptr_t)arg,
                               sizeof(enable)) != 0u) {
                result = PTY_ERR_FAULT;
                break;
            }
            pty->packet_mode = (enable != 0) ? 1u : 0u;
            result = 0;
            break;
        }
        case TTY_FIONREAD: {
            int32_t available = is_master
                ? (int32_t)pty->out_count
                : (int32_t)pty_slave_readable_locked(pty);
            result = copy_to_user((void *)(uintptr_t)arg, &available,
                                  sizeof(available)) == 0u ? 0 : PTY_ERR_FAULT;
            break;
        }
        case TTY_TIOCOUTQ: {
            int32_t queued = is_master ? 0 : (int32_t)pty->out_count;
            result = copy_to_user((void *)(uintptr_t)arg, &queued,
                                  sizeof(queued)) == 0u ? 0 : PTY_ERR_FAULT;
            break;
        }
        default:
            /* Logged because an unimplemented terminal ioctl is invisible
             * otherwise: the caller just gets ENOTTY and decides the fd is not
             * a terminal, which is how a whole terminal emulator gives up. */
            if (OS_CONFIG_FOREIGN_TRACE) {
                serial_write_string("[pty] unhandled ioctl req=");
                serial_write_uint64(request);
                serial_write_string(" pair=");
                serial_write_uint32((uint32_t)index);
                serial_write_string(is_master ? " master\n" : " slave\n");
            }
            result = PTY_ERR_NOTTY;
            break;
    }

    spinlock_unlock(&g_pty_table_lock);
    irq_restore(irq_flags);

    if (bind_ctty >= 0) {
        pty_ctty_bind_current(bind_ctty);
    }
    if (unbind_ctty) {
        int32_t pid = process_get_current_pid();
        if (ctty_pid_is_valid(pid)) {
            g_ctty[pid] = -1;
        }
    }
    return result;
}
