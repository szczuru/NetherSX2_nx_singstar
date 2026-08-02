/* usb_singstar_nx.c - Emulated SingStar USB-mic device for NetherSX2
 *
 * How the PS2 USB mic stack works (abridged)
 * --------------------------------------------
 * The PS2 USB IOP module enumerates the SingStar adapter, then reads PCM via
 * isochronous IN transfers.  AetherSX2/PCSX2 emulates this in the USBMic
 * plugin (pcsx2/USB/usb-mic/usb-mic.cpp).
 *
 * Critical path in the core:
 *   MicrophoneDevice::CreateDevice()
 *     -> AudioDevice::CreateDevice(AUDIODIR_SOURCE, ..., playerN_device_name)
 *     -> if BOTH players fail to open -> device creation FAILS entirely
 *       (OSD: "USB-Mic: Neither player 1 nor 2 is connected")
 *       -> game sees no USB mic at all.
 *
 * On Android, AudioDevice is backed by AAudio DIRECTION_INPUT.  On Switch we
 * have no native AAudio input, so we:
 *
 *  a) Force the right prefs keys so the core creates a SingStar device on
 *     Port 1 (and Port 2 as backup).
 *  b) Export usb_singstar_openStream_hook() which aaudio.c calls for every
 *     AAUDIO_DIRECTION_INPUT open.  We always succeed and return a fake
 *     AAudioStream, so AudioDevice::CreateDevice never fails.
 *  c) Drive PCM either via the data callback (if the core registered one)
 *     or via AAudioStream_read (if the core pulls).
 *
 * MIT License - see LICENSE in the repository root.
 */

#include <switch.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdint.h>

#include "usb_singstar_nx.h"
#include "usb_mic_nx.h"
#include "aaudio.h"
#include "prefs.h"

/* Diagnostics: how many times core opened INPUT */
static int g_open_count = 0;
static int g_start_count = 0;
int usb_singstar_open_count(void) { return g_open_count; }
int usb_singstar_start_count(void) { return g_start_count; }


/* =========================================================================
 * Fake AAudioStream for INPUT direction
 * ========================================================================= */

#define MAX_INPUT_STREAMS 4

typedef struct {
    AAudioStream *stream;        /* the fake handle returned to the core    */
    int           player;        /* 0 or 1 -> which mic ring buffer to drain */
    AAudioStream_dataCallback  data_cb;
    void                      *data_user;
    int32_t       frames_per_cb;
    int32_t       sample_rate;
    int32_t       channel_count;
    aaudio_format_t format;
    int           running;
    pthread_t     thread;
    volatile int  thread_stop;
    volatile int  thread_exited;
    int64_t       frames_delivered;
} InputStreamEntry;

static InputStreamEntry  g_input[MAX_INPUT_STREAMS];
static int               g_input_count = 0;
static Mutex             g_input_lock;
static int               g_lock_ready = 0;

/* Minimal fake AAudioStream.  The core never dereferences the struct
 * directly (it only passes the pointer back into AAudio* functions). */
struct AAudioStreamStruct {
    int _singstar_marker;   /* 0xCAFE5135 identifies our streams */
};

#define SINGSTAR_MARKER 0xCAFE5135

static InputStreamEntry *find_entry(AAudioStream *s)
{
    for (int i = 0; i < g_input_count; i++)
        if (g_input[i].stream == s) return &g_input[i];
    return NULL;
}

int usb_singstar_owns_stream(AAudioStream *s)
{
    if (!s) return 0;
    return s->_singstar_marker == SINGSTAR_MARKER;
}

/* =========================================================================
 * Input capture thread (data-callback path)
 * ========================================================================= */

static void *input_capture_thread(void *arg)
{
    InputStreamEntry *e = (InputStreamEntry *)arg;

    int32_t nframes = e->frames_per_cb > 0 ? e->frames_per_cb : 256;
    /* Allocate enough for stereo float just in case. */
    size_t bytes = (size_t)nframes * 2 * sizeof(float);
    void *buf = malloc(bytes);
    if (!buf) {
        e->thread_exited = 1;
        return NULL;
    }

    while (!e->thread_stop) {
        uint32_t need = (uint32_t)nframes;
        uint32_t avail = mic_nx_frames_available(e->player);

        if (avail < need) {
            /* Wait roughly half a callback period. */
            int64_t wait_ns = (int64_t)nframes * 1000000000LL
                              / (e->sample_rate > 0 ? e->sample_rate : 48000) / 2;
            if (wait_ns < 500000) wait_ns = 500000;  /* >= 0.5 ms */
            svcSleepThread(wait_ns);
            continue;
        }

        /* Read mono i16 from the USB ring, then expand/convert if needed. */
        int16_t *tmp = (int16_t *)buf;
        uint32_t got = mic_nx_read(e->player, tmp, need);
        if (got < need)
            memset(tmp + got, 0, (size_t)(need - got) * sizeof(int16_t));

        if (e->format == AAUDIO_FORMAT_PCM_FLOAT) {
            /* Convert i16 -> float in-place from the end to avoid clobber. */
            float *fbuf = (float *)buf;
            for (int32_t i = (int32_t)need - 1; i >= 0; i--)
                fbuf[i] = (float)tmp[i] / 32768.0f;

            if (e->channel_count >= 2) {
                /* Duplicate mono -> stereo (interleaved). */
                for (int32_t i = (int32_t)need - 1; i >= 0; i--) {
                    float s = fbuf[i];
                    fbuf[i * 2]     = s;
                    fbuf[i * 2 + 1] = s;
                }
            }
        } else {
            /* PCM_I16 */
            if (e->channel_count >= 2) {
                for (int32_t i = (int32_t)need - 1; i >= 0; i--) {
                    int16_t s = tmp[i];
                    tmp[i * 2]     = s;
                    tmp[i * 2 + 1] = s;
                }
            }
        }

        if (e->data_cb) {
            aaudio_data_callback_result_t r =
                e->data_cb(e->stream, e->data_user, buf, nframes);
            if (r == AAUDIO_CALLBACK_RESULT_STOP) break;
        }

        e->frames_delivered += nframes;
    }

    free(buf);
    e->thread_exited = 1;
    return NULL;
}

/* =========================================================================
 * Hooked AAudio entry points - called from aaudio.c
 * ========================================================================= */

/*
 * Layout of AAudioStreamBuilderStruct in aaudio.c (must stay in sync):
 *   int32_t sample_rate;          //  0
 *   int32_t channel_count;        //  4
 *   int32_t device_id;            //  8
 *   aaudio_format_t format;       // 12
 *   aaudio_direction_t direction; // 16
 *   aaudio_sharing_mode_t sharing;// 20
 *   aaudio_performance_mode_t perf; // 24
 *   int32_t buffer_capacity_frames; // 28
 *   int32_t frames_per_cb;        // 32
 *   AAudioStream_dataCallback data_cb; // 36/40
 *   void *data_user;
 */
typedef struct {
    int32_t sample_rate, channel_count, device_id;
    aaudio_format_t format;
    aaudio_direction_t direction;
    aaudio_sharing_mode_t sharing;
    aaudio_performance_mode_t perf;
    int32_t buffer_capacity_frames, frames_per_cb;
    AAudioStream_dataCallback data_cb;
    void *data_user;
} BuilderPeek;

aaudio_result_t usb_singstar_openStream_hook(AAudioStreamBuilder *b,
                                             AAudioStream **out)
{
    if (!b || !out) return AAUDIO_ERROR_NULL;

    const BuilderPeek *peek = (const BuilderPeek *)b;

    if (!g_lock_ready) {
        mutexInit(&g_input_lock);
        g_lock_ready = 1;
    }

    mutexLock(&g_input_lock);

    if (g_input_count >= MAX_INPUT_STREAMS) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_NO_FREE_HANDLES;
    }

    InputStreamEntry *e = &g_input[g_input_count];
    memset(e, 0, sizeof(*e));

    e->stream = (AAudioStream *)calloc(1, sizeof(*e->stream));
    if (!e->stream) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_NO_MEMORY;
    }
    e->stream->_singstar_marker = SINGSTAR_MARKER;

    /* Player assignment: first INPUT stream -> player 0, second -> player 1. */
    e->player = 0;
    for (int i = 0; i < g_input_count; i++) {
        if (g_input[i].stream && g_input[i].player >= e->player)
            e->player = g_input[i].player + 1;
    }
    if (e->player >= MIC_NX_PLAYERS) e->player = MIC_NX_PLAYERS - 1;

    e->sample_rate    = peek->sample_rate > 0 ? peek->sample_rate : 48000;
    e->channel_count  = peek->channel_count > 0 ? peek->channel_count : 1;
    e->format         = (peek->format == AAUDIO_FORMAT_PCM_FLOAT)
                            ? AAUDIO_FORMAT_PCM_FLOAT
                            : AAUDIO_FORMAT_PCM_I16;
    e->frames_per_cb  = peek->frames_per_cb > 0 ? peek->frames_per_cb : 256;
    e->data_cb        = peek->data_cb;   /* may be NULL (pull/GetBuffer path) */
    e->data_user      = peek->data_user;
    e->running        = 0;
    e->thread_stop    = 0;
    e->thread_exited  = 0;
    e->frames_delivered = 0;

    g_input_count++;
    mutexUnlock(&g_input_lock);

    *out = e->stream;
    g_open_count++;
    return AAUDIO_OK;
}

aaudio_result_t usb_singstar_requestStart_hook(AAudioStream *s)
{
    if (!g_lock_ready) return AAUDIO_ERROR_INVALID_HANDLE;

    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_INVALID_HANDLE;
    }

    if (!e->running) {
        e->thread_stop   = 0;
        e->thread_exited = 0;
        e->running       = 1;
        /* Only spawn the callback thread when the core registered a data_cb.
         * Pull-mode (GetBuffer / AAudioStream_read) does not need it. */
        if (e->data_cb)
            pthread_create(&e->thread, NULL, input_capture_thread, e);
        g_start_count++;
    }
    mutexUnlock(&g_input_lock);
    return AAUDIO_OK;
}

aaudio_result_t usb_singstar_requestStop_hook(AAudioStream *s)
{
    if (!g_lock_ready) return AAUDIO_ERROR_INVALID_HANDLE;

    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_INVALID_HANDLE;
    }

    if (e->running) {
        e->thread_stop = 1;
        if (e->data_cb) {
            for (int i = 0; i < 200 && !e->thread_exited; i++)
                svcSleepThread(5000000ULL);
            if (e->thread_exited) pthread_join(e->thread, NULL);
        }
        e->running = 0;
    }
    mutexUnlock(&g_input_lock);
    return AAUDIO_OK;
}

aaudio_result_t usb_singstar_close_hook(AAudioStream *s)
{
    if (!g_lock_ready) return AAUDIO_ERROR_INVALID_HANDLE;

    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_INVALID_HANDLE;
    }

    if (e->running) {
        e->thread_stop = 1;
        if (e->data_cb) {
            for (int i = 0; i < 200 && !e->thread_exited; i++)
                svcSleepThread(5000000ULL);
            if (e->thread_exited) pthread_join(e->thread, NULL);
        }
        e->running = 0;
    }
    free(e->stream);

    int idx = (int)(e - g_input);
    if (idx < g_input_count - 1)
        g_input[idx] = g_input[g_input_count - 1];
    memset(&g_input[g_input_count - 1], 0, sizeof(InputStreamEntry));
    g_input_count--;

    mutexUnlock(&g_input_lock);
    return AAUDIO_OK;
}

int64_t usb_singstar_getFramesRead_hook(AAudioStream *s)
{
    if (!g_lock_ready) return -1;

    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return -1;  /* signals "not our stream" to aaudio.c */
    }
    int64_t n = e->frames_delivered;
    mutexUnlock(&g_input_lock);
    return n;
}

int32_t usb_singstar_read_hook(AAudioStream *s, void *buf, int32_t num_frames,
                               int64_t timeout_nanos)
{
    if (!s || !buf || num_frames <= 0) return AAUDIO_ERROR_NULL;
    if (!g_lock_ready) return AAUDIO_ERROR_INVALID_HANDLE;

    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_INVALID_HANDLE;
    }
    int player = e->player;
    int32_t ch = e->channel_count;
    aaudio_format_t fmt = e->format;
    mutexUnlock(&g_input_lock);

    /* Wait until enough frames are available (or timeout). */
    int64_t waited = 0;
    const int64_t slice = 500000; /* 0.5 ms */
    while (mic_nx_frames_available(player) < (uint32_t)num_frames) {
        if (timeout_nanos >= 0 && waited >= timeout_nanos)
            break;
        svcSleepThread(slice);
        waited += slice;
    }

    int16_t *tmp = (int16_t *)malloc((size_t)num_frames * sizeof(int16_t));
    if (!tmp) return AAUDIO_ERROR_NO_MEMORY;

    uint32_t got = mic_nx_read(player, tmp, (uint32_t)num_frames);
    if (got < (uint32_t)num_frames)
        memset(tmp + got, 0, (size_t)(num_frames - (int32_t)got) * sizeof(int16_t));

    if (fmt == AAUDIO_FORMAT_PCM_FLOAT) {
        float *dst = (float *)buf;
        if (ch >= 2) {
            for (int32_t i = 0; i < num_frames; i++) {
                float s = (float)tmp[i] / 32768.0f;
                dst[i * 2]     = s;
                dst[i * 2 + 1] = s;
            }
        } else {
            for (int32_t i = 0; i < num_frames; i++)
                dst[i] = (float)tmp[i] / 32768.0f;
        }
    } else {
        int16_t *dst = (int16_t *)buf;
        if (ch >= 2) {
            for (int32_t i = 0; i < num_frames; i++) {
                dst[i * 2]     = tmp[i];
                dst[i * 2 + 1] = tmp[i];
            }
        } else {
            memcpy(dst, tmp, (size_t)num_frames * sizeof(int16_t));
        }
    }

    free(tmp);

    mutexLock(&g_input_lock);
    e = find_entry(s);
    if (e) e->frames_delivered += num_frames;
    mutexUnlock(&g_input_lock);

    return num_frames;
}

/* =========================================================================
 * Prefs injection
 * ========================================================================= */

static void inject_singstar_prefs(void)
{
    /*
     * Force (not merely seed) the SingStar device on Port 1.
     * AetherSX2 / PCSX2 keys:
     *   USB1/Type_str  or  USB/Port1/Type  - depending on core vintage.
     * We write both families so either naming scheme works.
     *
     * Type value "singstar" matches MicrophoneDevice::TypeName().
     * Subtype 0 = Singstar dual-mic.
     *
     * Device names must be non-empty so CreateDevice() is actually called.
     * Our AAudio INPUT hook succeeds for any name, so the string is only a
     * label - it does not need to match a real Android AudioManager device.
     */
    /* Exact PCSX2/AetherSX2 keys (section USB1, TypeName "singstar"):
     *   USB1/Type = singstar
     *   USB1/singstar_subtype = 0   (MIC_SINGSTAR dual-mic)
     *   USB1/singstar_player1_device_name = ...
     *   USB1/singstar_player2_device_name = ...
     * FORCE overwrite (prefs_set_*), not seed -- user ini may have Type=None.
     */
    prefs_set_string("USB1/Type", "singstar");
    prefs_set_string("USB1/DeviceType", "singstar");
    prefs_set_string("USB1/Device", "singstar");
    prefs_set_string("USB1/singstar_subtype", "0");
    prefs_set_string("USB1/singstar_player1_device_name", "switch_usb_mic_0");
    prefs_set_string("USB1/singstar_player2_device_name", "switch_usb_mic_1");
    prefs_set_string("USB1/singstar_input_latency", "20");
    prefs_set_string("USB1/singstar_input_device_name", "switch_usb_mic_0");

    /* Older / alternate layouts */
    prefs_set_string("USB/Port1/Type", "singstar");
    prefs_set_string("USB/Port1/singstar/player1_device_name", "switch_usb_mic_0");
    prefs_set_string("USB/Port1/singstar/player2_device_name", "switch_usb_mic_1");
    prefs_set_string("USB/Port1/singstar/input_latency", "20");
    prefs_set_string("USB/USB1/Type", "singstar");
    prefs_set_string("USB/USB1/Device", "singstar");

    prefs_set_string("USB2/Type", "None");
    prefs_seed_public("USB/Port2/Type", "none");
}

/* =========================================================================
 * Public API
 * ========================================================================= */

int usb_singstar_nx_init(void)
{
    if (!g_lock_ready) {
        mutexInit(&g_input_lock);
        g_lock_ready = 1;
    }
    g_input_count = 0;
    memset(g_input, 0, sizeof(g_input));

    /* Inject prefs BEFORE the core reads settings / creates USB devices. */
    inject_singstar_prefs();

    /* Start physical USB capture (OK even if no device is present yet). */
    mic_nx_init();

    return mic_nx_any_connected() ? 0 : 1;
}

void usb_singstar_nx_exit(void)
{
    if (g_lock_ready) {
        mutexLock(&g_input_lock);
        for (int i = 0; i < g_input_count; i++) {
            InputStreamEntry *e = &g_input[i];
            if (e->running) {
                e->thread_stop = 1;
                if (e->data_cb) {
                    for (int j = 0; j < 200 && !e->thread_exited; j++)
                        svcSleepThread(5000000ULL);
                    if (e->thread_exited) pthread_join(e->thread, NULL);
                }
                e->running = 0;
            }
            free(e->stream);
        }
        g_input_count = 0;
        mutexUnlock(&g_input_lock);
    }

    mic_nx_exit();
}

int usb_singstar_nx_connected(void)
{
    return mic_nx_any_connected();
}
