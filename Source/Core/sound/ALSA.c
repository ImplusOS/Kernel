#include "ALSA.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "Core/sync/Spinlock.h"
#include "Core/timer/Timer.h"
#include "Core/usercopy/Usercopy.h"
#include "Debug/serial/Serial.h"
#include "Drivers/Module/AudioManager.h"
#include "Core/process/ProcessManager.h"
#include "kernel/config.h"
#include "smp/SMP_Main.h"

/* ------------------------------------------------------------------------
 * ABI (include/uapi/sound/asound.h, x86_64 layouts)
 * ------------------------------------------------------------------------ */

#define SND_VER(a, b, c) (((a) << 16) | ((b) << 8) | (c))
#define ALSA_PCM_VERSION SND_VER(2u, 0u, 15u)
#define ALSA_CTL_VERSION SND_VER(2u, 0u, 8u)

#define IOC(dir, type, nr, size) \
    (((uint64_t)(dir) << 30) | ((uint64_t)(size) << 16) | \
     ((uint64_t)(type) << 8) | (uint64_t)(nr))
#define IOC_NONE 0u
#define IOC_W    1u
#define IOC_R    2u
#define IOC_RW   3u

/* The request's size field is checked against these struct sizes; they are
 * the x86_64 ones. */
#define PCM_INFO_SIZE       288u
#define PCM_HW_PARAMS_SIZE  608u
#define PCM_SW_PARAMS_SIZE  136u
#define PCM_STATUS_SIZE     152u
#define PCM_SYNC_PTR_SIZE   136u
#define PCM_XFERI_SIZE      24u
#define CTL_CARD_INFO_SIZE  376u
#define CTL_ELEM_LIST_SIZE  72u

#define PCM_IOCTL_PVERSION      IOC(IOC_R,  'A', 0x00, 4)
#define PCM_IOCTL_INFO          IOC(IOC_R,  'A', 0x01, PCM_INFO_SIZE)
#define PCM_IOCTL_TSTAMP        IOC(IOC_W,  'A', 0x02, 4)
#define PCM_IOCTL_TTSTAMP       IOC(IOC_W,  'A', 0x03, 4)
#define PCM_IOCTL_USER_PVERSION IOC(IOC_W,  'A', 0x04, 4)
#define PCM_IOCTL_HW_REFINE     IOC(IOC_RW, 'A', 0x10, PCM_HW_PARAMS_SIZE)
#define PCM_IOCTL_HW_PARAMS     IOC(IOC_RW, 'A', 0x11, PCM_HW_PARAMS_SIZE)
#define PCM_IOCTL_HW_FREE       IOC(IOC_NONE, 'A', 0x12, 0)
#define PCM_IOCTL_SW_PARAMS     IOC(IOC_RW, 'A', 0x13, PCM_SW_PARAMS_SIZE)
#define PCM_IOCTL_STATUS        IOC(IOC_R,  'A', 0x20, PCM_STATUS_SIZE)
#define PCM_IOCTL_DELAY         IOC(IOC_R,  'A', 0x21, 8)
#define PCM_IOCTL_HWSYNC        IOC(IOC_NONE, 'A', 0x22, 0)
#define PCM_IOCTL_SYNC_PTR      IOC(IOC_RW, 'A', 0x23, PCM_SYNC_PTR_SIZE)
#define PCM_IOCTL_STATUS_EXT    IOC(IOC_RW, 'A', 0x24, PCM_STATUS_SIZE)
#define PCM_IOCTL_PREPARE       IOC(IOC_NONE, 'A', 0x40, 0)
#define PCM_IOCTL_RESET         IOC(IOC_NONE, 'A', 0x41, 0)
#define PCM_IOCTL_START         IOC(IOC_NONE, 'A', 0x42, 0)
#define PCM_IOCTL_DROP          IOC(IOC_NONE, 'A', 0x43, 0)
#define PCM_IOCTL_DRAIN         IOC(IOC_NONE, 'A', 0x44, 0)
#define PCM_IOCTL_PAUSE         IOC(IOC_W,  'A', 0x45, 4)
#define PCM_IOCTL_REWIND        IOC(IOC_W,  'A', 0x46, 8)
#define PCM_IOCTL_RESUME        IOC(IOC_NONE, 'A', 0x47, 0)
#define PCM_IOCTL_XRUN          IOC(IOC_NONE, 'A', 0x48, 0)
#define PCM_IOCTL_FORWARD       IOC(IOC_W,  'A', 0x49, 8)
#define PCM_IOCTL_WRITEI_FRAMES IOC(IOC_W,  'A', 0x50, PCM_XFERI_SIZE)

#define CTL_IOCTL_PVERSION        IOC(IOC_R,  'U', 0x00, 4)
#define CTL_IOCTL_CARD_INFO       IOC(IOC_R,  'U', 0x01, CTL_CARD_INFO_SIZE)
#define CTL_IOCTL_ELEM_LIST       IOC(IOC_RW, 'U', 0x10, CTL_ELEM_LIST_SIZE)
#define CTL_IOCTL_SUBSCRIBE       IOC(IOC_RW, 'U', 0x16, 4)
#define CTL_IOCTL_PCM_NEXT_DEVICE IOC(IOC_R,  'U', 0x30, 4)
#define CTL_IOCTL_PCM_INFO        IOC(IOC_RW, 'U', 0x31, PCM_INFO_SIZE)
#define CTL_IOCTL_PCM_PREFER_SUB  IOC(IOC_W,  'U', 0x32, 4)
#define CTL_IOCTL_RAWMIDI_NEXT    IOC(IOC_RW, 'U', 0x40, 4)
#define CTL_IOCTL_POWER_STATE     IOC(IOC_R,  'U', 0xD1, 4)

#define E_AGAIN   (-11)
#define E_FAULT   (-14)
#define E_BUSY    (-16)
#define E_NODEV   (-19)
#define E_INVAL   (-22)
#define E_NOTTY   (-25)
#define E_PIPE    (-32)
#define E_NOSYS   (-38)
#define E_BADFD   (-77)
#define E_NOMEM   (-12)

enum {
    ST_OPEN = 0, ST_SETUP, ST_PREPARED, ST_RUNNING, ST_XRUN, ST_DRAINING,
    ST_PAUSED, ST_SUSPENDED, ST_DISCONNECTED
};

/* hw_params parameter numbering. */
enum {
    P_ACCESS = 0, P_FORMAT = 1, P_SUBFORMAT = 2,
    P_FIRST_INTERVAL = 8,
    P_SAMPLE_BITS = 8, P_FRAME_BITS, P_CHANNELS, P_RATE, P_PERIOD_TIME,
    P_PERIOD_SIZE, P_PERIOD_BYTES, P_PERIODS, P_BUFFER_TIME, P_BUFFER_SIZE,
    P_BUFFER_BYTES, P_TICK_TIME,
    P_LAST_INTERVAL = P_TICK_TIME
};
#define N_IV (P_LAST_INTERVAL - P_FIRST_INTERVAL + 1)

#define ACCESS_RW_INTERLEAVED 3u

#define FMT_U8       1u
#define FMT_S16_LE   2u
#define FMT_S32_LE   10u
#define FMT_FLOAT_LE 14u

#define INFO_INTERLEAVED     0x00000100u
#define INFO_BLOCK_TRANSFER  0x00010000u
#define INFO_PAUSE           0x00080000u

#define IV_OPENMIN 1u
#define IV_OPENMAX 2u
#define IV_INTEGER 4u
#define IV_EMPTY   8u

typedef struct {
    uint32_t min, max, flags;
} iv_t;

/* struct snd_pcm_hw_params, x86_64 (naturally aligned, no padding). */
typedef struct {
    uint32_t flags;
    uint32_t masks[3][8];
    uint32_t mres[5][8];
    iv_t     intervals[N_IV];
    iv_t     ires[9];
    uint32_t rmask;
    uint32_t cmask;
    uint32_t info;
    uint32_t msbits;
    uint32_t rate_num;
    uint32_t rate_den;
    uint64_t fifo_size;
    uint8_t  reserved[64];
} hw_params_t;
_Static_assert(sizeof(hw_params_t) == PCM_HW_PARAMS_SIZE, "hw_params layout");

typedef struct __attribute__((packed)) {
    int32_t  tstamp_mode;
    uint32_t period_step;
    uint32_t sleep_min;
    uint32_t pad0;
    uint64_t avail_min;
    uint64_t xfer_align;
    uint64_t start_threshold;
    uint64_t stop_threshold;
    uint64_t silence_threshold;
    uint64_t silence_size;
    uint64_t boundary;
    uint32_t proto;
    uint32_t tstamp_type;
    uint8_t  reserved[56];
} sw_params_t;
_Static_assert(sizeof(sw_params_t) == PCM_SW_PARAMS_SIZE, "sw_params layout");

typedef struct __attribute__((packed)) {
    int64_t tv_sec;
    int64_t tv_nsec;
} ts_t;

typedef struct __attribute__((packed)) {
    int32_t  state;
    uint32_t pad0;
    ts_t     trigger_tstamp;
    ts_t     tstamp;
    uint64_t appl_ptr;
    uint64_t hw_ptr;
    int64_t  delay;
    uint64_t avail;
    uint64_t avail_max;
    uint64_t overrange;
    int32_t  suspended_state;
    uint32_t audio_tstamp_data;
    ts_t     audio_tstamp;
    ts_t     driver_tstamp;
    uint32_t audio_tstamp_accuracy;
    uint8_t  reserved[20];
} status_t;
_Static_assert(sizeof(status_t) == PCM_STATUS_SIZE, "status layout");

typedef struct __attribute__((packed)) {
    uint32_t flags;
    uint32_t pad0;
    union {
        struct __attribute__((packed)) {
            int32_t  state;
            int32_t  pad1;
            uint64_t hw_ptr;
            ts_t     tstamp;
            int32_t  suspended_state;
            int32_t  pad2;
            ts_t     audio_tstamp;
        } status;
        uint8_t reserved[64];
    } s;
    union {
        struct __attribute__((packed)) {
            uint64_t appl_ptr;
            uint64_t avail_min;
        } control;
        uint8_t reserved[64];
    } c;
} sync_ptr_t;
_Static_assert(sizeof(sync_ptr_t) == PCM_SYNC_PTR_SIZE, "sync_ptr layout");

#define SYNC_PTR_HWSYNC    1u
#define SYNC_PTR_APPL      2u
#define SYNC_PTR_AVAIL_MIN 4u

typedef struct __attribute__((packed)) {
    int64_t  result;
    uint64_t buf;
    uint64_t frames;
} xferi_t;

typedef struct __attribute__((packed)) {
    uint32_t device;
    uint32_t subdevice;
    int32_t  stream;
    int32_t  card;
    uint8_t  id[64];
    uint8_t  name[80];
    uint8_t  subname[32];
    int32_t  dev_class;
    int32_t  dev_subclass;
    uint32_t subdevices_count;
    uint32_t subdevices_avail;
    uint8_t  sync[16];
    uint8_t  reserved[64];
} pcm_info_t;
_Static_assert(sizeof(pcm_info_t) == PCM_INFO_SIZE, "pcm_info layout");

typedef struct __attribute__((packed)) {
    int32_t card;
    int32_t pad;
    uint8_t id[16];
    uint8_t driver[16];
    uint8_t name[32];
    uint8_t longname[80];
    uint8_t reserved_[16];
    uint8_t mixername[80];
    uint8_t components[128];
} card_info_t;
_Static_assert(sizeof(card_info_t) == CTL_CARD_INFO_SIZE, "card_info layout");

/* ------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------ */

/* The device plays 48 kHz S16_LE stereo; hda.c's native format. */
#define DEV_RATE        48000u
#define DEV_FRAME_BYTES 4u
/* How far ahead of the hardware the converted audio is kept: enough to ride
 * out a late timer tick and to stay ahead of the controller's DMA prefetch,
 * small enough that the position reported to the application stays close to
 * what is actually heard. The prefetch is the binding limit: QEMU's HDA codec
 * pulls up to 8 KiB (~43 ms) from the ring ahead of playback, so a 40 ms
 * target let the DMA read past the last written byte on every lap -- ~10 ms
 * of sound, then ~20 ms of the ring's silence, over and over. 80 ms keeps a
 * full prefetch plus margin queued and still fits the 160 ms ring. */
#define DEV_TARGET_BYTES (DEV_RATE * DEV_FRAME_BYTES * 80u / 1000u)

typedef struct {
    int open;
    int32_t state;
    int32_t prev_state; /* before PAUSED */

    /* hw_params */
    uint32_t format;
    uint32_t channels;
    uint32_t rate;
    uint32_t sample_bytes;
    uint32_t frame_bytes;
    uint64_t period_size;
    uint64_t buffer_size;

    /* sw_params */
    uint64_t avail_min;
    uint64_t start_threshold;
    uint64_t stop_threshold;
    uint64_t boundary;
    int32_t  tstamp_mode;
    uint32_t tstamp_type;

    uint8_t *buf;          /* buffer_size frames of client-format audio */

    /* Absolute frame counters (never wrap); the ABI's appl_ptr/hw_ptr are
     * these modulo `boundary`. */
    uint64_t appl;         /* written by the application */
    uint64_t hw;           /* reported as played */
    uint64_t pump;         /* whole client frames handed to the converter */
    uint64_t pump_frac;    /* sub-frame position, 32.32 */
    uint64_t dev_bytes;    /* converted bytes handed to the device */
    uint64_t dev_end;      /* device byte position just past this stream's
                            * last mixed-in frame (g_dev_written scale) */
    uint64_t avail_max;

    ts_t trigger_tstamp;
} alsa_pcm_t;

/* Several playback substreams, mixed into the one hardware stream. Linux
 * programs open the "hw" device directly (there is no dmix/PulseAudio here),
 * and Chromium opens a new stream for every page that makes sound -- often
 * before the previous page's stream is closed. With a single substream that
 * second open failed (EBUSY), Chromium fell back to a silent fake output, and
 * a tab played nothing after a navigation. */
#define ALSA_SUBSTREAMS 8

static spinlock_t g_alsa_lock;
static alsa_pcm_t g_pcms[ALSA_SUBSTREAMS];
/* The substream the code below works on: g_pcm is *g_cur. Valid while
 * g_alsa_lock is held; lock_irq() loads it from the calling CPU's selection
 * (set by the entry point that owns the file), the mixer walks all of them. */
static alsa_pcm_t *g_cur = &g_pcms[0];
static int32_t g_sel[OS_CONFIG_SMP_MAX_CPUS];
#define g_pcm (*g_cur)
static int g_ctl_open_count;
static int g_dev_open;
static int g_dev_running;
static uint64_t g_dev_written;  /* bytes handed to the device, ever */
static spinlock_t g_open_lock;  /* serialises device open/close */

static uint32_t alsa_cpu(void)
{
    uint32_t c = smp_get_current_cpu_id();
    return c < OS_CONFIG_SMP_MAX_CPUS ? c : 0u;
}

static void alsa_select(int32_t index)
{
    g_sel[alsa_cpu()] = (index >= 0 && index < ALSA_SUBSTREAMS) ? index : 0;
}

/* Periodic playback statistics on the serial console (bring-up aid). */
#ifndef ALSA_STATS
#define ALSA_STATS 0
#endif

/* Bring-up statistics, printed every couple of seconds while open. */
static uint64_t g_stat_writei_frames;
static int32_t g_stat_peak;
static uint32_t g_stat_xruns;
static uint32_t g_stat_quiet, g_stat_loud;
static uint32_t g_stat_dry, g_stat_short;
static uint64_t g_stat_last_ns, g_stat_max_gap_ns;



static uint64_t lock_irq(void)
{
    uint64_t f = irq_save_disable();
    spinlock_lock(&g_alsa_lock);
    g_cur = &g_pcms[g_sel[alsa_cpu()]];
    return f;
}

static void unlock_irq(uint64_t f)
{
    spinlock_unlock(&g_alsa_lock);
    irq_restore(f);
}

static ts_t now_ts(void)
{
    uint64_t ns = timer_monotonic_ns();
    ts_t t;
    t.tv_sec = (int64_t)(ns / 1000000000ull);
    t.tv_nsec = (int64_t)(ns % 1000000000ull);
    return t;
}

int alsa_card_present(void)
{
    return audio_manager_init() ? 1 : 0;
}

/* ------------------------------------------------------------------------
 * Interval arithmetic (the kernel's snd_interval rules)
 * ------------------------------------------------------------------------ */

static int iv_is_empty(const iv_t *i) { return (i->flags & IV_EMPTY) != 0u; }

static int iv_check_empty(const iv_t *i)
{
    return i->min > i->max ||
           (i->min == i->max &&
            (i->flags & (IV_OPENMIN | IV_OPENMAX)) != 0u);
}

static int iv_single(const iv_t *i)
{
    return !iv_is_empty(i) &&
           (i->min == i->max ||
            (i->min + 1u == i->max && (i->flags & IV_OPENMIN) != 0u) ||
            (i->min + 1u == i->max && (i->flags & IV_OPENMAX) != 0u));
}

static uint32_t iv_value(const iv_t *i)
{
    if ((i->flags & IV_OPENMIN) != 0u && i->min != i->max) {
        return i->min + 1u;
    }
    return i->min;
}

/* Intersect `i` with `v`. Returns 1 if `i` changed, 0 if not, <0 if the
 * result is empty. */
static int iv_refine(iv_t *i, const iv_t *v)
{
    int changed = 0;
    if (iv_is_empty(i)) {
        return E_INVAL;
    }
    if (i->min < v->min) {
        i->min = v->min;
        i->flags = (i->flags & ~IV_OPENMIN) | (v->flags & IV_OPENMIN);
        changed = 1;
    } else if (i->min == v->min && (i->flags & IV_OPENMIN) == 0u &&
               (v->flags & IV_OPENMIN) != 0u) {
        i->flags |= IV_OPENMIN;
        changed = 1;
    }
    if (i->max > v->max) {
        i->max = v->max;
        i->flags = (i->flags & ~IV_OPENMAX) | (v->flags & IV_OPENMAX);
        changed = 1;
    } else if (i->max == v->max && (i->flags & IV_OPENMAX) == 0u &&
               (v->flags & IV_OPENMAX) != 0u) {
        i->flags |= IV_OPENMAX;
        changed = 1;
    }
    if ((i->flags & IV_INTEGER) == 0u && (v->flags & IV_INTEGER) != 0u) {
        i->flags |= IV_INTEGER;
        changed = 1;
    }
    if ((i->flags & IV_INTEGER) != 0u) {
        if ((i->flags & IV_OPENMIN) != 0u) {
            i->min++;
            i->flags &= ~IV_OPENMIN;
        }
        if ((i->flags & IV_OPENMAX) != 0u) {
            i->max--;
            i->flags &= ~IV_OPENMAX;
        }
    } else if ((i->flags & (IV_OPENMIN | IV_OPENMAX)) == 0u &&
               i->min == i->max) {
        i->flags |= IV_INTEGER;
    }
    if (iv_check_empty(i)) {
        i->flags |= IV_EMPTY;
        return E_INVAL;
    }
    return changed;
}

static uint32_t sat32(uint64_t v) { return v > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)v; }

static uint32_t div32(uint32_t a, uint32_t b, uint32_t *rem)
{
    if (b == 0u) {
        *rem = 0u;
        return 0xFFFFFFFFu;
    }
    *rem = a % b;
    return a / b;
}

static uint32_t muldiv32(uint32_t a, uint32_t b, uint32_t c, uint32_t *rem)
{
    uint64_t n = (uint64_t)a * b;
    if (c == 0u) {
        *rem = 0u;
        return 0xFFFFFFFFu;
    }
    uint64_t q = n / c;
    if (q > 0xFFFFFFFFull) {
        *rem = 0u;
        return 0xFFFFFFFFu;
    }
    *rem = (uint32_t)(n % c);
    return (uint32_t)q;
}

static uint32_t flg(const iv_t *i, uint32_t f) { return (i->flags & f) != 0u; }

static void iv_mul(const iv_t *a, const iv_t *b, iv_t *c)
{
    c->flags = 0u;
    c->min = sat32((uint64_t)a->min * b->min);
    if (flg(a, IV_OPENMIN) || flg(b, IV_OPENMIN)) c->flags |= IV_OPENMIN;
    c->max = sat32((uint64_t)a->max * b->max);
    if (flg(a, IV_OPENMAX) || flg(b, IV_OPENMAX)) c->flags |= IV_OPENMAX;
    if (flg(a, IV_INTEGER) && flg(b, IV_INTEGER)) c->flags |= IV_INTEGER;
}

static void iv_div(const iv_t *a, const iv_t *b, iv_t *c)
{
    uint32_t r;
    c->flags = 0u;
    c->min = div32(a->min, b->max, &r);
    if (r != 0u || flg(a, IV_OPENMIN) || flg(b, IV_OPENMAX)) c->flags |= IV_OPENMIN;
    if (b->min > 0u) {
        c->max = div32(a->max, b->min, &r);
        if (r != 0u) {
            c->max++;
            c->flags |= IV_OPENMAX;
        }
        if (flg(a, IV_OPENMAX) || flg(b, IV_OPENMIN)) c->flags |= IV_OPENMAX;
    } else {
        c->max = 0xFFFFFFFFu;
    }
}

/* c = a * b / k */
static void iv_muldivk(const iv_t *a, const iv_t *b, uint32_t k, iv_t *c)
{
    uint32_t r;
    c->flags = 0u;
    c->min = muldiv32(a->min, b->min, k, &r);
    if (r != 0u || flg(a, IV_OPENMIN) || flg(b, IV_OPENMIN)) c->flags |= IV_OPENMIN;
    c->max = muldiv32(a->max, b->max, k, &r);
    if (r != 0u) {
        c->max++;
        c->flags |= IV_OPENMAX;
    }
    if (flg(a, IV_OPENMAX) || flg(b, IV_OPENMAX)) c->flags |= IV_OPENMAX;
}

/* c = a * k / b */
static void iv_mulkdiv(const iv_t *a, uint32_t k, const iv_t *b, iv_t *c)
{
    uint32_t r;
    c->flags = 0u;
    c->min = muldiv32(a->min, k, b->max, &r);
    if (r != 0u || flg(a, IV_OPENMIN) || flg(b, IV_OPENMAX)) c->flags |= IV_OPENMIN;
    if (b->min > 0u) {
        c->max = muldiv32(a->max, k, b->min, &r);
        if (r != 0u) {
            c->max++;
            c->flags |= IV_OPENMAX;
        }
        if (flg(a, IV_OPENMAX) || flg(b, IV_OPENMIN)) c->flags |= IV_OPENMAX;
    } else {
        c->max = 0xFFFFFFFFu;
    }
}

/* ------------------------------------------------------------------------
 * hw_params refinement
 * ------------------------------------------------------------------------ */

static uint32_t format_width(uint32_t fmt)
{
    switch (fmt) {
    case FMT_U8:       return 8u;
    case FMT_S16_LE:   return 16u;
    case FMT_S32_LE:   return 32u;
    case FMT_FLOAT_LE: return 32u;
    default:           return 0u;
    }
}

static const uint32_t g_formats[] = { FMT_U8, FMT_S16_LE, FMT_S32_LE, FMT_FLOAT_LE };

#define IVP(p, n) (&(p)->intervals[(n) - P_FIRST_INTERVAL])

static int mask_test(const uint32_t *m, uint32_t bit)
{
    return (m[bit >> 5] & (1u << (bit & 31u))) != 0u;
}

static int mask_empty(const uint32_t *m)
{
    for (uint32_t i = 0; i < 8u; ++i) {
        if (m[i] != 0u) return 0;
    }
    return 1;
}

static void hw_constraint_iv(iv_t *out, uint32_t min, uint32_t max, int integer)
{
    out->min = min;
    out->max = max;
    out->flags = integer ? IV_INTEGER : 0u;
}

/* Apply one rule result to `target`; tracks whether anything changed. */
static int apply(hw_params_t *p, uint32_t target, const iv_t *v, int *changed)
{
    int r = iv_refine(IVP(p, target), v);
    if (r < 0) return r;
    if (r > 0) {
        *changed = 1;
        p->cmask |= 1u << target;
    }
    return 0;
}

static int hw_refine(hw_params_t *p)
{
    uint32_t hw_masks[3][8];
    memset(hw_masks, 0, sizeof(hw_masks));
    hw_masks[P_ACCESS][0] = 1u << ACCESS_RW_INTERLEAVED;
    for (uint32_t i = 0; i < sizeof(g_formats) / sizeof(g_formats[0]); ++i) {
        hw_masks[P_FORMAT][g_formats[i] >> 5] |= 1u << (g_formats[i] & 31u);
    }
    hw_masks[P_SUBFORMAT][0] = 1u; /* STD */

    p->cmask = 0u;
    for (uint32_t m = 0; m < 3u; ++m) {
        int changed = 0;
        for (uint32_t w = 0; w < 8u; ++w) {
            uint32_t nv = p->masks[m][w] & hw_masks[m][w];
            if (nv != p->masks[m][w]) changed = 1;
            p->masks[m][w] = nv;
        }
        if (mask_empty(p->masks[m])) return E_INVAL;
        if (changed) p->cmask |= 1u << m;
    }

    /* The hardware's own limits. */
    iv_t c;
    int dummy = 0;
    hw_constraint_iv(&c, 8u, 32u, 1);        if (apply(p, P_SAMPLE_BITS, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 8u, 256u, 1);       if (apply(p, P_FRAME_BITS, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 1u, 8u, 1);         if (apply(p, P_CHANNELS, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 8000u, 192000u, 0); if (apply(p, P_RATE, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 1u, 0xFFFFFFFFu, 1); if (apply(p, P_PERIOD_SIZE, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 64u, 131072u, 1);   if (apply(p, P_PERIOD_BYTES, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 2u, 128u, 1);       if (apply(p, P_PERIODS, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 1u, 0xFFFFFFFFu, 1); if (apply(p, P_BUFFER_SIZE, &c, &dummy) < 0) return E_INVAL;
    hw_constraint_iv(&c, 128u, 1048576u, 1); if (apply(p, P_BUFFER_BYTES, &c, &dummy) < 0) return E_INVAL;
    /* At least 160 ms of buffer. Chromium asks for ~85 ms, which leaves its
     * audio thread under 60 ms of slack; one late wakeup while a page is busy
     * then empties the buffer and the stream underruns (an audible gap per
     * xrun, dozens a minute). Clients read the delay back, so A/V sync keeps. */
    hw_constraint_iv(&c, 160000u, 0xFFFFFFFFu, 0); if (apply(p, P_BUFFER_TIME, &c, &dummy) < 0) return E_INVAL;

    for (int iter = 0; iter < 64; ++iter) {
        int changed = 0;
        iv_t t;

        /* FORMAT <-> SAMPLE_BITS */
        {
            const iv_t *sb = IVP(p, P_SAMPLE_BITS);
            uint32_t lo = 0xFFFFFFFFu, hi = 0u;
            for (uint32_t i = 0; i < sizeof(g_formats) / sizeof(g_formats[0]); ++i) {
                uint32_t f = g_formats[i];
                if (!mask_test(p->masks[P_FORMAT], f)) continue;
                uint32_t w = format_width(f);
                int ok = w >= sb->min && w <= sb->max &&
                         !(w == sb->min && flg(sb, IV_OPENMIN)) &&
                         !(w == sb->max && flg(sb, IV_OPENMAX));
                if (!ok) {
                    p->masks[P_FORMAT][f >> 5] &= ~(1u << (f & 31u));
                    p->cmask |= 1u << P_FORMAT;
                    changed = 1;
                    continue;
                }
                if (w < lo) lo = w;
                if (w > hi) hi = w;
            }
            if (mask_empty(p->masks[P_FORMAT])) return E_INVAL;
            hw_constraint_iv(&t, lo, hi, 1);
            if (apply(p, P_SAMPLE_BITS, &t, &changed) < 0) return E_INVAL;
        }

        iv_div(IVP(p, P_FRAME_BITS), IVP(p, P_CHANNELS), &t);
        if (apply(p, P_SAMPLE_BITS, &t, &changed) < 0) return E_INVAL;
        iv_mul(IVP(p, P_SAMPLE_BITS), IVP(p, P_CHANNELS), &t);
        if (apply(p, P_FRAME_BITS, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_PERIOD_BYTES), 8u, IVP(p, P_PERIOD_SIZE), &t);
        if (apply(p, P_FRAME_BITS, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_BUFFER_BYTES), 8u, IVP(p, P_BUFFER_SIZE), &t);
        if (apply(p, P_FRAME_BITS, &t, &changed) < 0) return E_INVAL;
        iv_div(IVP(p, P_FRAME_BITS), IVP(p, P_SAMPLE_BITS), &t);
        if (apply(p, P_CHANNELS, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_PERIOD_SIZE), 1000000u, IVP(p, P_PERIOD_TIME), &t);
        if (apply(p, P_RATE, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_BUFFER_SIZE), 1000000u, IVP(p, P_BUFFER_TIME), &t);
        if (apply(p, P_RATE, &t, &changed) < 0) return E_INVAL;
        iv_div(IVP(p, P_BUFFER_SIZE), IVP(p, P_PERIOD_SIZE), &t);
        if (apply(p, P_PERIODS, &t, &changed) < 0) return E_INVAL;
        iv_div(IVP(p, P_BUFFER_SIZE), IVP(p, P_PERIODS), &t);
        if (apply(p, P_PERIOD_SIZE, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_PERIOD_BYTES), 8u, IVP(p, P_FRAME_BITS), &t);
        if (apply(p, P_PERIOD_SIZE, &t, &changed) < 0) return E_INVAL;
        iv_muldivk(IVP(p, P_PERIOD_TIME), IVP(p, P_RATE), 1000000u, &t);
        if (apply(p, P_PERIOD_SIZE, &t, &changed) < 0) return E_INVAL;
        iv_mul(IVP(p, P_PERIOD_SIZE), IVP(p, P_PERIODS), &t);
        if (apply(p, P_BUFFER_SIZE, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_BUFFER_BYTES), 8u, IVP(p, P_FRAME_BITS), &t);
        if (apply(p, P_BUFFER_SIZE, &t, &changed) < 0) return E_INVAL;
        iv_muldivk(IVP(p, P_BUFFER_TIME), IVP(p, P_RATE), 1000000u, &t);
        if (apply(p, P_BUFFER_SIZE, &t, &changed) < 0) return E_INVAL;
        iv_muldivk(IVP(p, P_PERIOD_SIZE), IVP(p, P_FRAME_BITS), 8u, &t);
        if (apply(p, P_PERIOD_BYTES, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_PERIOD_SIZE), 1000000u, IVP(p, P_RATE), &t);
        if (apply(p, P_PERIOD_TIME, &t, &changed) < 0) return E_INVAL;
        iv_muldivk(IVP(p, P_BUFFER_SIZE), IVP(p, P_FRAME_BITS), 8u, &t);
        if (apply(p, P_BUFFER_BYTES, &t, &changed) < 0) return E_INVAL;
        iv_mulkdiv(IVP(p, P_BUFFER_SIZE), 1000000u, IVP(p, P_RATE), &t);
        if (apply(p, P_BUFFER_TIME, &t, &changed) < 0) return E_INVAL;

        if (!changed) break;
    }

    p->info = INFO_INTERLEAVED | INFO_BLOCK_TRANSFER | INFO_PAUSE;
    if (iv_single(IVP(p, P_SAMPLE_BITS))) {
        p->msbits = iv_value(IVP(p, P_SAMPLE_BITS));
    }
    if (iv_single(IVP(p, P_RATE))) {
        p->rate_num = iv_value(IVP(p, P_RATE));
        p->rate_den = 1u;
    }
    p->fifo_size = 0u;
    return 0;
}

/* ------------------------------------------------------------------------
 * Conversion to the device format
 * ------------------------------------------------------------------------ */

/* IEEE-754 single to S16 with integer operations only (the kernel does not
 * use the FPU). Clamps to [-1, 1]. */
static int32_t float_bits_to_s16(uint32_t bits)
{
    uint32_t exp = (bits >> 23) & 0xFFu;
    uint32_t mant = (bits & 0x7FFFFFu) | 0x800000u;
    int neg = (bits >> 31) != 0u;
    if (exp == 0xFFu) return 0;               /* NaN/Inf -> silence */
    if (exp >= 127u) return neg ? -32768 : 32767; /* |x| >= 1 */
    if (exp < 127u - 16u) return 0;
    /* value = mant * 2^(exp-127-23); scaled by 2^15 */
    int32_t shift = (int32_t)(127u + 23u - 15u) - (int32_t)exp;
    int32_t v = (int32_t)(mant >> (uint32_t)shift);
    if (v > 32767) v = 32767;
    return neg ? -v : v;
}

static int32_t sample_s16(const uint8_t *p, uint32_t fmt)
{
    switch (fmt) {
    case FMT_U8:
        return ((int32_t)p[0] - 128) << 8;
    case FMT_S16_LE:
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    case FMT_S32_LE: {
        uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        return (int32_t)v >> 16;
    }
    case FMT_FLOAT_LE: {
        uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        return float_bits_to_s16(v);
    }
    default:
        return 0;
    }
}

/* Stereo pair of client frame `abs` (absolute index into the ring). */
static void frame_lr(uint64_t abs, int32_t *l, int32_t *r)
{
    const uint8_t *f = g_pcm.buf + (abs % g_pcm.buffer_size) * g_pcm.frame_bytes;
    *l = sample_s16(f, g_pcm.format);
    *r = g_pcm.channels >= 2u ? sample_s16(f + g_pcm.sample_bytes, g_pcm.format)
                              : *l;
}

/* Converts client frames [pump, appl) into at most `max_out` device frames
 * at 48 kHz, linear interpolation. Returns the device frames produced. */
static uint32_t convert(int16_t *out, uint32_t max_out)
{
    uint32_t produced = 0u;
    uint64_t step = ((uint64_t)g_pcm.rate << 32) / DEV_RATE;
    while (produced < max_out) {
        uint64_t idx = g_pcm.pump;
        /* Interpolation needs the next frame too, unless the rate matches. */
        if (idx >= g_pcm.appl ||
            (g_pcm.pump_frac != 0u && idx + 1u >= g_pcm.appl)) {
            break;
        }
        int32_t l0, r0;
        frame_lr(idx, &l0, &r0);
        if (g_pcm.pump_frac != 0u) {
            int32_t l1, r1;
            frame_lr(idx + 1u, &l1, &r1);
            int64_t fr = (int64_t)(g_pcm.pump_frac >> 16); /* 16-bit fraction */
            l0 = (int32_t)(l0 + (((int64_t)(l1 - l0) * fr) >> 16));
            r0 = (int32_t)(r0 + (((int64_t)(r1 - r0) * fr) >> 16));
        }
        out[produced * 2u] = (int16_t)l0;
        out[produced * 2u + 1u] = (int16_t)r0;
        produced++;
        uint64_t pos = g_pcm.pump_frac + step;
        g_pcm.pump += pos >> 32;
        g_pcm.pump_frac = pos & 0xFFFFFFFFull;
    }
    return produced;
}

/* ------------------------------------------------------------------------
 * Playback engine (called with g_alsa_lock held)
 * ------------------------------------------------------------------------ */

static uint64_t pcm_avail(void)
{
    uint64_t queued = g_pcm.appl - g_pcm.hw;
    return queued >= g_pcm.buffer_size ? 0u : g_pcm.buffer_size - queued;
}

static int stream_active(const alsa_pcm_t *p)
{
    return p->open && (p->state == ST_RUNNING || p->state == ST_DRAINING);
}

/* The current substream stops feeding the device. The hardware stream itself
 * stops only once no other substream is playing: stopping it discards what
 * is queued, which would cut the others off. */
static void dev_stop_locked(void)
{
    for (int i = 0; i < ALSA_SUBSTREAMS; ++i) {
        if (&g_pcms[i] != g_cur && stream_active(&g_pcms[i])) {
            return;
        }
    }
    if (g_dev_running) {
        audio_manager_stream_stop();
        g_dev_running = 0;
    }
}

/* Work out how far the current substream has been played, and whether it
 * drained or ran dry. */
static void stream_update_locked(uint64_t dev_queued)
{
    if (g_pcm.state != ST_RUNNING && g_pcm.state != ST_DRAINING) {
        return;
    }
    /* Played position in client frames: what was handed over, less what the
     * device still holds of this stream (scaled back to the client rate). */
    uint64_t dev_played = g_dev_written - dev_queued;
    uint64_t held_bytes = g_pcm.dev_end > dev_played ? g_pcm.dev_end - dev_played : 0u;
    uint64_t held = (held_bytes / DEV_FRAME_BYTES) * g_pcm.rate / DEV_RATE;
    uint64_t hw = g_pcm.pump > held ? g_pcm.pump - held : 0u;
    if (hw > g_pcm.appl) hw = g_pcm.appl;
    if (hw > g_pcm.hw) g_pcm.hw = hw;

    uint64_t avail = pcm_avail();
    if (avail > g_pcm.avail_max) g_pcm.avail_max = avail;

    if (g_pcm.state == ST_DRAINING) {
        if (g_pcm.hw >= g_pcm.appl && held_bytes == 0u) {
            g_pcm.state = ST_SETUP;
        }
        return;
    }
    /* Underrun: everything written has been played. */
    if (avail >= g_pcm.stop_threshold && held_bytes == 0u) {
        g_pcm.state = ST_XRUN;
        g_pcm.trigger_tstamp = now_ts();
        g_stat_xruns++;
    }
}

/* Mix every playing substream into the device, then update each one's
 * position. Leaves g_cur as it found it. */
static void pump_locked(void)
{
    alsa_pcm_t *self = g_cur;
    uint64_t dev_queued = audio_manager_stream_queued();
    {
        uint64_t ns = timer_monotonic_ns();
        if (g_stat_last_ns != 0u && ns - g_stat_last_ns > g_stat_max_gap_ns)
            g_stat_max_gap_ns = ns - g_stat_last_ns;
        g_stat_last_ns = ns;
        if (g_dev_running && dev_queued == 0u) g_stat_dry++;
    }
    int16_t tmp[256 * 2];
    int16_t mix[256 * 2];
    int32_t acc[256 * 2];
    while (dev_queued < DEV_TARGET_BYTES) {
        uint64_t space = audio_manager_stream_space();
        uint64_t want = DEV_TARGET_BYTES - dev_queued;
        if (want > space) want = space;
        uint32_t frames = (uint32_t)(want / DEV_FRAME_BYTES);
        if (frames > 256u) frames = 256u;
        if (frames == 0u) break;
        uint32_t produced = 0u;
        for (int i = 0; i < ALSA_SUBSTREAMS; ++i) {
            alsa_pcm_t *p = &g_pcms[i];
            if (!stream_active(p)) continue;
            g_cur = p;
            uint32_t got = convert(tmp, frames);
            if (got == 0u) continue;
#ifdef ALSA_TONE
            { static uint32_t ph; for (uint32_t k = 0; k < got; ++k, ++ph) {
                int16_t v = ((ph / 55u) & 1u) ? 8000 : -8000; tmp[2*k] = v; tmp[2*k+1] = v; } }
#endif
            if (produced == 0u) {
                memset(acc, 0, sizeof(acc));
            } else if (got > produced) {
                memset(acc + produced * 2u, 0, (size_t)(got - produced) * 2u * sizeof(int32_t));
            }
            int32_t cpk = 0;
            for (uint32_t k = 0; k < got * 2u; ++k) {
                acc[k] += tmp[k];
                int32_t a = tmp[k] < 0 ? -(int32_t)tmp[k] : tmp[k];
                if (a > cpk) cpk = a;
            }
            if (cpk > g_stat_peak) g_stat_peak = cpk;
            if (cpk < 30) g_stat_quiet++; else g_stat_loud++;
            p->dev_end = g_dev_written + (uint64_t)got * DEV_FRAME_BYTES;
            p->dev_bytes += (uint64_t)got * DEV_FRAME_BYTES;
            if (got > produced) produced = got;
        }
        if (produced == 0u) {
            if (dev_queued < DEV_TARGET_BYTES / 2u) g_stat_short++;
            break;
        }
        for (uint32_t k = 0; k < produced * 2u; ++k) {
            int32_t v = acc[k];
            mix[k] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
        }
        uint64_t n = audio_manager_stream_write(mix, (uint64_t)produced * DEV_FRAME_BYTES);
        g_dev_written += n;
        dev_queued += n;
        if (n < (uint64_t)produced * DEV_FRAME_BYTES) break;
    }
    if (!g_dev_running && dev_queued != 0u) {
        g_dev_running = audio_manager_stream_start() ? 1 : 0;
    }

    int any = 0;
    for (int i = 0; i < ALSA_SUBSTREAMS; ++i) {
        alsa_pcm_t *p = &g_pcms[i];
        if (!p->open) continue;
        g_cur = p;
        stream_update_locked(dev_queued);
        if (stream_active(p)) any = 1;
    }
    g_cur = self;
    /* Nothing plays any more and the device ran out: stop the hardware. */
    if (!any && dev_queued == 0u && g_dev_running) {
        audio_manager_stream_stop();
        g_dev_running = 0;
    }
}

static void start_locked(void)
{
    g_pcm.dev_end = 0u;
    g_pcm.state = ST_RUNNING;
    g_pcm.trigger_tstamp = now_ts();
    pump_locked();
}

static uint64_t g_pump_tick_ns;

void alsa_timer_tick_backup(void)
{
    if (!g_dev_open) {
        return;
    }
    uint64_t last = __atomic_load_n(&g_pump_tick_ns, __ATOMIC_RELAXED);
    if (timer_monotonic_ns() - last < 20000000ull) {
        return;
    }
    alsa_timer_tick();
}

void alsa_timer_tick(void)
{
    if (!g_dev_open) {
        return;
    }
    __atomic_store_n(&g_pump_tick_ns, timer_monotonic_ns(), __ATOMIC_RELAXED);
    uint64_t f = lock_irq();
    pump_locked();
    static uint32_t ticks;
    int report = ALSA_STATS && (++ticks % 500u) == 0u;
    uint64_t appl = 0, hw = 0, dev = g_dev_written;
    int32_t state = -1, peak = g_stat_peak;
    for (int i = 0; i < ALSA_SUBSTREAMS; ++i) {
        if (g_pcms[i].open && g_pcms[i].state >= ST_PREPARED) {
            appl = g_pcms[i].appl; hw = g_pcms[i].hw; state = g_pcms[i].state;
        }
    }
    if (state < 0) report = 0;
    uint32_t xruns = g_stat_xruns;
    uint32_t quiet = g_stat_quiet, loud = g_stat_loud;
    uint64_t wf = g_stat_writei_frames;
    if (report) { g_stat_peak = 0; g_stat_quiet = g_stat_loud = 0; }
    unlock_irq(f);
    if (report) {
        serial_write_string("[alsa] st=");
        serial_write_uint32((uint32_t)state);
        serial_write_string(" appl=");
        serial_write_uint64(appl);
        serial_write_string(" hw=");
        serial_write_uint64(hw);
        serial_write_string(" dev_bytes=");
        serial_write_uint64(dev);
        serial_write_string(" written=");
        serial_write_uint64(wf);
        serial_write_string(" peak=");
        serial_write_uint32((uint32_t)peak);
        serial_write_string(" xruns=");
        serial_write_uint32(xruns);
        serial_write_string(" quiet=");
        serial_write_uint32(quiet);
        serial_write_string(" loud=");
        serial_write_uint32(loud);
        serial_write_string(" dry=");
        serial_write_uint32(g_stat_dry);
        serial_write_string(" short=");
        serial_write_uint32(g_stat_short);
        serial_write_string(" maxgap_us=");
        serial_write_uint32((uint32_t)(g_stat_max_gap_ns / 1000u));
        g_stat_dry = g_stat_short = 0; g_stat_max_gap_ns = 0;
        serial_write_string("\n");
    }
}

/* ------------------------------------------------------------------------
 * PCM device
 * ------------------------------------------------------------------------ */

static void free_buffer_locked(void)
{
    if (g_pcm.buf != NULL) {
        free(g_pcm.buf);
        g_pcm.buf = NULL;
    }
}

int alsa_pcm_open(void)
{
    if (!alsa_card_present()) {
        return E_NODEV;
    }
    spinlock_lock(&g_open_lock);
    uint64_t f = lock_irq();
    int index = -1;
    for (int i = 0; i < ALSA_SUBSTREAMS; ++i) {
        if (!g_pcms[i].open) {
            index = i;
            break;
        }
    }
    unlock_irq(f);
    if (index < 0) {
        spinlock_unlock(&g_open_lock);
        return E_BUSY;
    }

    if (!g_dev_open) {
        if (!audio_manager_open()) {
            spinlock_unlock(&g_open_lock);
            return E_BUSY;
        }
        if (!audio_manager_stream_ok()) {
            audio_manager_close();
            spinlock_unlock(&g_open_lock);
            return E_NODEV;
        }
        g_dev_running = 0;
        g_dev_written = 0u;
        g_dev_open = 1;
    }

    f = lock_irq();
    memset(&g_pcms[index], 0, sizeof(g_pcms[index]));
    g_pcms[index].open = 1;
    g_pcms[index].state = ST_OPEN;
    unlock_irq(f);
    spinlock_unlock(&g_open_lock);
    serial_write_string("[alsa] pcm open pid=");
    serial_write_uint32((uint32_t)process_get_current_pid());
    serial_write_string(" substream=");
    serial_write_uint32((uint32_t)index);
    serial_write_string("\n");
    return index;
}

void alsa_pcm_close(int32_t index)
{
    if (index < 0 || index >= ALSA_SUBSTREAMS) {
        return;
    }
    alsa_select(index);
    spinlock_lock(&g_open_lock);
    uint64_t f = lock_irq();
    dev_stop_locked();
    free_buffer_locked();
    memset(&g_pcm, 0, sizeof(g_pcm));
    int any_open = 0;
    for (int i = 0; i < ALSA_SUBSTREAMS; ++i) {
        if (g_pcms[i].open) any_open = 1;
    }
    unlock_irq(f);
    if (!any_open && g_dev_open) {
        f = lock_irq();
        g_dev_open = 0;
        if (g_dev_running) {
            audio_manager_stream_stop();
            g_dev_running = 0;
        }
        unlock_irq(f);
        audio_manager_close();
    }
    spinlock_unlock(&g_open_lock);
}

static uint64_t ptr_mod(uint64_t abs)
{
    return g_pcm.boundary != 0u ? abs % g_pcm.boundary : abs;
}

static void fill_status(status_t *st)
{
    memset(st, 0, sizeof(*st));
    st->state = g_pcm.state;
    st->trigger_tstamp = g_pcm.trigger_tstamp;
    st->tstamp = now_ts();
    st->appl_ptr = ptr_mod(g_pcm.appl);
    st->hw_ptr = ptr_mod(g_pcm.hw);
    st->delay = (g_pcm.state == ST_RUNNING || g_pcm.state == ST_DRAINING ||
                 g_pcm.state == ST_PAUSED)
                    ? (int64_t)(g_pcm.appl - g_pcm.hw) : 0;
    st->avail = pcm_avail();
    st->avail_max = g_pcm.avail_max;
    st->suspended_state = 0;
    st->audio_tstamp = st->tstamp;
    st->driver_tstamp = st->tstamp;
    g_pcm.avail_max = 0u;
}

static int64_t do_hw_params(uint64_t arg, int commit)
{
    hw_params_t p;
    if (copy_from_user(&p, (const void *)(uintptr_t)arg, sizeof(p)) != 0u) {
        return E_FAULT;
    }
    int rc = hw_refine(&p);
    if (rc < 0) {
        return rc;
    }
    if (commit) {
        /* alsa-lib has narrowed everything to one value by now; settle any
         * leftover range on its minimum the way the kernel does. */
        uint32_t fmt = 0xFFFFFFFFu;
        for (uint32_t i = 0; i < sizeof(g_formats) / sizeof(g_formats[0]); ++i) {
            if (mask_test(p.masks[P_FORMAT], g_formats[i])) {
                fmt = g_formats[i];
                break;
            }
        }
        uint32_t channels = iv_value(IVP(&p, P_CHANNELS));
        uint32_t rate = iv_value(IVP(&p, P_RATE));
        uint32_t period = iv_value(IVP(&p, P_PERIOD_SIZE));
        uint32_t buffer = IVP(&p, P_BUFFER_SIZE)->max;
        if (fmt == 0xFFFFFFFFu || channels == 0u || rate == 0u ||
            period == 0u || buffer < period) {
            return E_INVAL;
        }
        uint32_t sb = format_width(fmt) / 8u;
        uint8_t *nbuf = malloc((size_t)buffer * sb * channels);
        if (nbuf == NULL) {
            return E_NOMEM;
        }
        memset(nbuf, 0, (size_t)buffer * sb * channels);

        uint64_t f = lock_irq();
        if (g_pcm.state == ST_RUNNING || g_pcm.state == ST_DRAINING ||
            g_pcm.state == ST_PAUSED) {
            unlock_irq(f);
            free(nbuf);
            return E_BADFD;
        }
        dev_stop_locked();
        free_buffer_locked();
        g_pcm.buf = nbuf;
        g_pcm.format = fmt;
        g_pcm.channels = channels;
        g_pcm.rate = rate;
        g_pcm.sample_bytes = sb;
        g_pcm.frame_bytes = sb * channels;
        g_pcm.period_size = period;
        g_pcm.buffer_size = buffer;
        g_pcm.avail_min = period;
        g_pcm.start_threshold = 1u;
        g_pcm.stop_threshold = buffer;
        g_pcm.boundary = buffer;
        while (g_pcm.boundary * 2u <= 0x7FFFFFFFFFFFFFFFull - buffer) {
            g_pcm.boundary *= 2u;
        }
        g_pcm.appl = g_pcm.hw = g_pcm.pump = 0u;
        g_pcm.pump_frac = 0u;
        g_pcm.state = ST_SETUP;
        unlock_irq(f);

        serial_write_string("[alsa] hw_params rate=");
        serial_write_uint32(rate);
        serial_write_string(" ch=");
        serial_write_uint32(channels);
        serial_write_string(" fmt=");
        serial_write_uint32(fmt);
        serial_write_string(" period=");
        serial_write_uint32(period);
        serial_write_string(" buffer=");
        serial_write_uint32(buffer);
        serial_write_string("\n");
    }
    if (copy_to_user((void *)(uintptr_t)arg, &p, sizeof(p)) != 0u) {
        return E_FAULT;
    }
    return 0;
}

static int64_t do_sw_params(uint64_t arg)
{
    sw_params_t sw;
    if (copy_from_user(&sw, (const void *)(uintptr_t)arg, sizeof(sw)) != 0u) {
        return E_FAULT;
    }
    uint64_t f = lock_irq();
    if (g_pcm.state == ST_OPEN) {
        unlock_irq(f);
        return E_BADFD;
    }
    if (sw.avail_min == 0u) sw.avail_min = 1u;
    g_pcm.avail_min = sw.avail_min;
    g_pcm.start_threshold = sw.start_threshold;
    g_pcm.stop_threshold = sw.stop_threshold;
    if (sw.boundary != 0u && sw.boundary % g_pcm.buffer_size == 0u) {
        g_pcm.boundary = sw.boundary;
    } else {
        sw.boundary = g_pcm.boundary;
    }
    g_pcm.tstamp_mode = sw.tstamp_mode;
    g_pcm.tstamp_type = sw.tstamp_type;
    sw.proto = ALSA_PCM_VERSION;
    unlock_irq(f);
    if (copy_to_user((void *)(uintptr_t)arg, &sw, sizeof(sw)) != 0u) {
        return E_FAULT;
    }
    return 0;
}

static int64_t do_writei(uint64_t arg)
{
    xferi_t x;
    if (copy_from_user(&x, (const void *)(uintptr_t)arg, sizeof(x)) != 0u) {
        return E_FAULT;
    }
    uint64_t f = lock_irq();
    switch (g_pcm.state) {
    case ST_XRUN:
        unlock_irq(f);
        return E_PIPE;
    case ST_PREPARED:
    case ST_RUNNING:
    case ST_PAUSED:
        break;
    default:
        unlock_irq(f);
        return E_BADFD;
    }
    pump_locked();
    if (g_pcm.state == ST_XRUN) {
        unlock_irq(f);
        return E_PIPE;
    }
    uint64_t avail = pcm_avail();
    uint64_t frames = x.frames < avail ? x.frames : avail;
    uint64_t start = g_pcm.appl;
    uint64_t bsize = g_pcm.buffer_size;
    uint32_t fb = g_pcm.frame_bytes;
    uint8_t *buf = g_pcm.buf;
    unlock_irq(f);

    if (x.frames == 0u) {
        x.result = 0;
        return copy_to_user((void *)(uintptr_t)arg, &x, sizeof(x)) != 0u ? E_FAULT : 0;
    }
    if (frames == 0u) {
        return ALSA_WOULD_BLOCK;
    }

    /* Copy outside the lock: user memory may fault. [appl, hw + buffer) is
     * the writer's alone, the converter only reads below appl. */
    uint64_t done = 0u;
    while (done < frames) {
        uint64_t off = (start + done) % bsize;
        uint64_t n = bsize - off;
        if (n > frames - done) n = frames - done;
        if (copy_from_user(buf + off * fb,
                           (const void *)(uintptr_t)(x.buf + done * fb),
                           n * fb) != 0u) {
            break;
        }
        done += n;
    }
    if (done == 0u) {
        return E_FAULT;
    }

    f = lock_irq();
    if (g_pcm.buf == buf && g_pcm.appl == start) {
        g_pcm.appl += done;
        g_stat_writei_frames += done;
        if (g_pcm.state == ST_PREPARED &&
            g_pcm.appl - g_pcm.hw >= g_pcm.start_threshold) {
            start_locked();
        } else {
            pump_locked();
        }
    }
    unlock_irq(f);

    x.result = (int64_t)done;
    if (copy_to_user((void *)(uintptr_t)arg, &x, sizeof(x)) != 0u) {
        return E_FAULT;
    }
    return 0;
}

static int64_t do_sync_ptr(uint64_t arg)
{
    sync_ptr_t sp;
    if (copy_from_user(&sp, (const void *)(uintptr_t)arg, sizeof(sp)) != 0u) {
        return E_FAULT;
    }
    uint64_t f = lock_irq();
    if ((sp.flags & SYNC_PTR_HWSYNC) != 0u) {
        pump_locked();
    }
    if ((sp.flags & SYNC_PTR_APPL) == 0u && g_pcm.boundary != 0u) {
        /* The application moved appl_ptr itself (mmap-style transfers or a
         * rewind); accept a forward move within the buffer. */
        uint64_t cur = ptr_mod(g_pcm.appl);
        uint64_t want = sp.c.control.appl_ptr % g_pcm.boundary;
        uint64_t fwd = (want + g_pcm.boundary - cur) % g_pcm.boundary;
        if (fwd != 0u && fwd <= pcm_avail()) {
            g_pcm.appl += fwd;
        }
    }
    if ((sp.flags & SYNC_PTR_AVAIL_MIN) == 0u) {
        if (sp.c.control.avail_min != 0u) {
            g_pcm.avail_min = sp.c.control.avail_min;
        }
    }
    memset(&sp.s, 0, sizeof(sp.s));
    sp.s.status.state = g_pcm.state;
    sp.s.status.hw_ptr = ptr_mod(g_pcm.hw);
    sp.s.status.tstamp = now_ts();
    sp.s.status.audio_tstamp = sp.s.status.tstamp;
    sp.c.control.appl_ptr = ptr_mod(g_pcm.appl);
    sp.c.control.avail_min = g_pcm.avail_min;
    unlock_irq(f);
    if (copy_to_user((void *)(uintptr_t)arg, &sp, sizeof(sp)) != 0u) {
        return E_FAULT;
    }
    return 0;
}

static void fill_pcm_info(pcm_info_t *info)
{
    memset(info, 0, sizeof(*info));
    info->device = 0u;
    info->subdevice = 0u;
    info->stream = 0; /* playback */
    info->card = 0;
    memcpy(info->id, "ImplusOS PCM", 13);
    memcpy(info->name, "ImplusOS Audio", 15);
    memcpy(info->subname, "subdevice #0", 13);
    info->subdevices_count = 1u;
    info->subdevices_avail = 1u;
}

static int64_t alsa_pcm_ioctl_impl(uint64_t request, uint64_t arg);

/* Bring-up trace: every PCM ioctl that fails, and the first few of each
 * run, with the request and result. */
int64_t alsa_pcm_ioctl(int32_t index, uint64_t request, uint64_t arg)
{
    alsa_select(index);
    int64_t rc = alsa_pcm_ioctl_impl(request, arg);
    static volatile uint32_t traced;
    if ((rc < 0 && rc != ALSA_WOULD_BLOCK && rc != E_AGAIN &&
         __atomic_fetch_add(&traced, 1u, __ATOMIC_RELAXED) < 64u)) {
        serial_write_string("[alsa] pcm ioctl ");
        serial_write_uint64(request);
        serial_write_string(" -> ");
        serial_write_uint64((uint64_t)rc);
        serial_write_string("\n");
    }
    return rc;
}

static int64_t alsa_pcm_ioctl_impl(uint64_t request, uint64_t arg)
{
    int32_t ival = 0;
    uint64_t f;

    switch (request) {
    case PCM_IOCTL_PVERSION: {
        int32_t v = (int32_t)ALSA_PCM_VERSION;
        return copy_to_user((void *)(uintptr_t)arg, &v, sizeof(v)) != 0u ? E_FAULT : 0;
    }
    case PCM_IOCTL_INFO: {
        pcm_info_t info;
        fill_pcm_info(&info);
        info.subdevices_avail = 1u;
        return copy_to_user((void *)(uintptr_t)arg, &info, sizeof(info)) != 0u ? E_FAULT : 0;
    }
    case PCM_IOCTL_TSTAMP:
    case PCM_IOCTL_TTSTAMP:
    case PCM_IOCTL_USER_PVERSION:
        return 0;
    case PCM_IOCTL_HW_REFINE:
        return do_hw_params(arg, 0);
    case PCM_IOCTL_HW_PARAMS:
        return do_hw_params(arg, 1);
    case PCM_IOCTL_HW_FREE:
        f = lock_irq();
        if (g_pcm.state == ST_RUNNING || g_pcm.state == ST_DRAINING) {
            unlock_irq(f);
            return E_BADFD;
        }
        dev_stop_locked();
        free_buffer_locked();
        g_pcm.state = ST_OPEN;
        unlock_irq(f);
        return 0;
    case PCM_IOCTL_SW_PARAMS:
        return do_sw_params(arg);
    case PCM_IOCTL_STATUS:
    case PCM_IOCTL_STATUS_EXT: {
        status_t st;
        f = lock_irq();
        pump_locked();
        fill_status(&st);
        unlock_irq(f);
        return copy_to_user((void *)(uintptr_t)arg, &st, sizeof(st)) != 0u ? E_FAULT : 0;
    }
    case PCM_IOCTL_DELAY: {
        f = lock_irq();
        pump_locked();
        int32_t state = g_pcm.state;
        int64_t d = (int64_t)(g_pcm.appl - g_pcm.hw);
        unlock_irq(f);
        if (state == ST_XRUN) return E_PIPE;
        if (state == ST_OPEN || state == ST_SETUP) return E_BADFD;
        return copy_to_user((void *)(uintptr_t)arg, &d, sizeof(d)) != 0u ? E_FAULT : 0;
    }
    case PCM_IOCTL_HWSYNC: {
        f = lock_irq();
        pump_locked();
        int32_t state = g_pcm.state;
        unlock_irq(f);
        if (state == ST_XRUN) return E_PIPE;
        return (state == ST_OPEN || state == ST_SETUP) ? E_BADFD : 0;
    }
    case PCM_IOCTL_SYNC_PTR:
        return do_sync_ptr(arg);
    case PCM_IOCTL_PREPARE:
        f = lock_irq();
        if (g_pcm.state == ST_OPEN || g_pcm.state == ST_DISCONNECTED) {
            unlock_irq(f);
            return E_BADFD;
        }
        dev_stop_locked();
        g_pcm.appl = g_pcm.hw;
        g_pcm.pump = g_pcm.hw;
        g_pcm.pump_frac = 0u;
        g_pcm.state = ST_PREPARED;
        unlock_irq(f);
        return 0;
    case PCM_IOCTL_RESET:
        f = lock_irq();
        dev_stop_locked();
        g_pcm.appl = g_pcm.hw;
        g_pcm.pump = g_pcm.hw;
        g_pcm.pump_frac = 0u;
        if (g_pcm.state == ST_RUNNING) {
            start_locked();
        }
        unlock_irq(f);
        return 0;
    case PCM_IOCTL_START:
        f = lock_irq();
        if (g_pcm.state != ST_PREPARED) {
            unlock_irq(f);
            return E_BADFD;
        }
        start_locked();
        unlock_irq(f);
        return 0;
    case PCM_IOCTL_DROP:
        f = lock_irq();
        if (g_pcm.state == ST_OPEN) {
            unlock_irq(f);
            return E_BADFD;
        }
        dev_stop_locked();
        g_pcm.appl = g_pcm.hw;
        g_pcm.pump = g_pcm.hw;
        g_pcm.pump_frac = 0u;
        g_pcm.state = ST_SETUP;
        unlock_irq(f);
        return 0;
    case PCM_IOCTL_DRAIN: {
        f = lock_irq();
        int64_t rc = 0;
        switch (g_pcm.state) {
        case ST_OPEN:
        case ST_SETUP:
            rc = E_BADFD;
            break;
        case ST_XRUN:
            g_pcm.state = ST_SETUP;
            break;
        case ST_PREPARED:
            if (g_pcm.appl == g_pcm.hw) {
                g_pcm.state = ST_SETUP;
                break;
            }
            start_locked();
            g_pcm.state = ST_DRAINING;
            rc = ALSA_WOULD_BLOCK;
            break;
        case ST_RUNNING:
        case ST_PAUSED:
            if (g_pcm.state == ST_PAUSED) {
                start_locked();
            }
            g_pcm.state = ST_DRAINING;
            pump_locked();
            rc = g_pcm.state == ST_SETUP ? 0 : ALSA_WOULD_BLOCK;
            break;
        case ST_DRAINING:
            pump_locked();
            rc = g_pcm.state == ST_SETUP ? 0 : ALSA_WOULD_BLOCK;
            break;
        default:
            rc = E_BADFD;
            break;
        }
        unlock_irq(f);
        return rc;
    }
    case PCM_IOCTL_PAUSE:
        ival = (int32_t)arg; /* declared _IOW, but the int is passed by value */
        f = lock_irq();
        if (ival != 0) {
            if (g_pcm.state != ST_RUNNING) {
                unlock_irq(f);
                return E_BADFD;
            }
            pump_locked();
            dev_stop_locked();
            /* Whatever was converted but not played is re-converted on
             * resume. */
            g_pcm.pump = g_pcm.hw;
            g_pcm.pump_frac = 0u;
            g_pcm.state = ST_PAUSED;
        } else {
            if (g_pcm.state != ST_PAUSED) {
                unlock_irq(f);
                return E_BADFD;
            }
            start_locked();
        }
        unlock_irq(f);
        return 0;
    case PCM_IOCTL_REWIND:
    case PCM_IOCTL_FORWARD: {
        uint64_t frames = 0;
        if (copy_from_user(&frames, (const void *)(uintptr_t)arg, sizeof(frames)) != 0u) {
            return E_FAULT;
        }
        f = lock_irq();
        uint64_t moved;
        if (request == PCM_IOCTL_REWIND) {
            uint64_t can = g_pcm.appl - (g_pcm.pump > g_pcm.hw ? g_pcm.pump : g_pcm.hw);
            moved = frames < can ? frames : can;
            g_pcm.appl -= moved;
        } else {
            uint64_t can = pcm_avail();
            moved = frames < can ? frames : can;
            g_pcm.appl += moved;
        }
        unlock_irq(f);
        /* The count moved goes back through the same pointer. */
        return copy_to_user((void *)(uintptr_t)arg, &moved, sizeof(moved)) != 0u
                   ? E_FAULT : 0;
    }
    case PCM_IOCTL_XRUN:
        f = lock_irq();
        if (g_pcm.state == ST_RUNNING || g_pcm.state == ST_PREPARED ||
            g_pcm.state == ST_PAUSED || g_pcm.state == ST_DRAINING) {
            dev_stop_locked();
            g_pcm.state = ST_XRUN;
        }
        unlock_irq(f);
        return 0;
    case PCM_IOCTL_RESUME:
        return E_NOSYS;
    case PCM_IOCTL_WRITEI_FRAMES:
        return do_writei(arg);
    default:
        return E_NOTTY;
    }
}

uint32_t alsa_pcm_poll(int32_t index, uint32_t events)
{
    alsa_select(index);
    uint64_t f = lock_irq();
    pump_locked();
    uint32_t ready = 0u;
    switch (g_pcm.state) {
    case ST_PREPARED:
    case ST_RUNNING:
    case ST_PAUSED:
        if (pcm_avail() >= g_pcm.avail_min) ready = 0x4u; /* POLLOUT */
        break;
    case ST_DRAINING:
        break;
    default:
        ready = 0x4u | 0x8u; /* POLLOUT | POLLERR: let the caller see the state */
        break;
    }
    unlock_irq(f);
    return ready & (events | 0x8u | 0x10u);
}

/* ------------------------------------------------------------------------
 * Control device
 * ------------------------------------------------------------------------ */

int alsa_ctl_open(void)
{
    serial_write_string("[alsa] ctl open\n");
    if (!alsa_card_present()) {
        return E_NODEV;
    }
    uint64_t f = lock_irq();
    g_ctl_open_count++;
    unlock_irq(f);
    return 0;
}

void alsa_ctl_close(void)
{
    uint64_t f = lock_irq();
    if (g_ctl_open_count > 0) g_ctl_open_count--;
    unlock_irq(f);
}

int64_t alsa_ctl_ioctl(uint64_t request, uint64_t arg)
{
    switch (request) {
    case CTL_IOCTL_PVERSION: {
        int32_t v = (int32_t)ALSA_CTL_VERSION;
        return copy_to_user((void *)(uintptr_t)arg, &v, sizeof(v)) != 0u ? E_FAULT : 0;
    }
    case CTL_IOCTL_CARD_INFO: {
        card_info_t ci;
        memset(&ci, 0, sizeof(ci));
        ci.card = 0;
        memcpy(ci.id, "ImplusOS", 9);
        const char *drv = audio_manager_name();
        if (drv == NULL) drv = "audio";
        size_t dl = strlen(drv);
        if (dl > 15u) dl = 15u;
        memcpy(ci.driver, drv, dl);
        memcpy(ci.name, "ImplusOS Audio", 15);
        memcpy(ci.longname, "ImplusOS Audio (kernel mixer)", 30);
        memcpy(ci.mixername, "ImplusOS", 9);
        return copy_to_user((void *)(uintptr_t)arg, &ci, sizeof(ci)) != 0u ? E_FAULT : 0;
    }
    case CTL_IOCTL_ELEM_LIST: {
        /* struct snd_ctl_elem_list: offset, space, used, count, pids, ...
         * No mixer controls: report zero elements. */
        uint8_t list[CTL_ELEM_LIST_SIZE];
        if (copy_from_user(list, (const void *)(uintptr_t)arg, sizeof(list)) != 0u) {
            return E_FAULT;
        }
        memset(list + 8, 0, 8); /* used = 0, count = 0 */
        return copy_to_user((void *)(uintptr_t)arg, list, sizeof(list)) != 0u ? E_FAULT : 0;
    }
    case CTL_IOCTL_SUBSCRIBE: {
        int32_t v = 0;
        return copy_to_user((void *)(uintptr_t)arg, &v, sizeof(v)) != 0u ? E_FAULT : 0;
    }
    case CTL_IOCTL_PCM_NEXT_DEVICE: {
        int32_t dev = 0;
        if (copy_from_user(&dev, (const void *)(uintptr_t)arg, sizeof(dev)) != 0u) {
            return E_FAULT;
        }
        dev = dev < 0 ? 0 : -1;
        return copy_to_user((void *)(uintptr_t)arg, &dev, sizeof(dev)) != 0u ? E_FAULT : 0;
    }
    case CTL_IOCTL_PCM_INFO: {
        pcm_info_t req;
        if (copy_from_user(&req, (const void *)(uintptr_t)arg, sizeof(req)) != 0u) {
            return E_FAULT;
        }
        if (req.device != 0u || req.stream != 0 ||
            (req.subdevice != 0u && req.subdevice != 0xFFFFFFFFu)) {
            return -2; /* ENOENT: no such device/stream (capture) */
        }
        pcm_info_t info;
        fill_pcm_info(&info);
        return copy_to_user((void *)(uintptr_t)arg, &info, sizeof(info)) != 0u ? E_FAULT : 0;
    }
    case CTL_IOCTL_PCM_PREFER_SUB:
        return 0;
    case CTL_IOCTL_RAWMIDI_NEXT: {
        int32_t dev = -1;
        return copy_to_user((void *)(uintptr_t)arg, &dev, sizeof(dev)) != 0u ? E_FAULT : 0;
    }
    case CTL_IOCTL_POWER_STATE: {
        int32_t st = 0; /* D0 */
        return copy_to_user((void *)(uintptr_t)arg, &st, sizeof(st)) != 0u ? E_FAULT : 0;
    }
    default:
        return E_NOTTY;
    }
}
