/* usb_singstar_nx.c — Emulated SingStar USB-mic device for NetherSX2
 *
 * How the PS2 USB mic stack works (abridged)
 * ────────────────────────────────────────────
 * The PS2 USB IOP module (usbd.irx + the game's audio driver) enumerates the
 * SingStar adapter using the standard USB stack, then reads PCM via
 * isochronous IN transfers on endpoint 0x81.  AetherSX2/PCSX2 emulates this
 * in the USBMic plugin (our reference: pcsx2/USB/usb-mic/usb-mic.cpp).
 *
 * In NetherSX2 the emulator core (libemucore.so) is the same PCSX2-lineage
 * binary as AetherSX2.  It reads USB device type from the settings key
 * "USB/Port1/Type" (and Port2).  The USBMic plugin then calls the AudioDevice
 * abstraction to get PCM frames.
 *
 * On Android, AudioDevice is backed by AAudio DIRECTION_INPUT.  On Switch we
 * have no AAudio input (aaudio.c only implements DIRECTION_OUTPUT natively),
 * so instead we:
 *
 *  a) Inject the right prefs keys so the core creates a SingStar device on
 *     Port 1 (and optionally Port 2).
 *  b) Export usb_singstar_openStream_hook() which aaudio.c calls instead of
 *     open_audren() when direction == AAUDIO_DIRECTION_INPUT.  We return a
 *     fake AAudioStream handle; the core then drives it through the normal
 *     AAudioStream_request{Start,Stop}/close path which we also hook.
 *  c) Export usb_singstar_owns_stream() so aaudio.c can gate the lifecycle
 *     calls correctly without needing to know about our internal table.
 *
 * This approach requires zero changes to jni_fake.c, imports.c, or the core
 * binary — we intercept only the AAudio shim, which is our own code.
 *
 * MIT License — see LICENSE in the repository root.
 */

#include <switch.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdint.h>

#include "usb_singstar_nx.h"
#include "usb_mic_nx.h"
#include "aaudio.h"
#include "prefs.h"

/* =========================================================================
 * Fake AAudioStream for INPUT direction
 * ========================================================================= */

/* We maintain a small table of "input stream" entries keyed on the fake
 * AAudioStream pointer we hand out.  aaudio.c calls usb_singstar_owns_stream()
 * to check membership before routing lifecycle calls here.               */

#define MAX_INPUT_STREAMS 4

typedef struct {
    AAudioStream *stream;        /* the fake handle returned to the core    */
    int           player;        /* 0 or 1 → which mic ring buffer to drain */
    AAudioStream_dataCallback  data_cb;
    void                      *data_user;
    int32_t       frames_per_cb;
    int32_t       sample_rate;
    int           running;
    pthread_t     thread;
    volatile int  thread_stop;
    volatile int  thread_exited;
    int64_t       frames_delivered;
} InputStreamEntry;

static InputStreamEntry  g_input[MAX_INPUT_STREAMS];
static int               g_input_count = 0;
static Mutex             g_input_lock;

/* Minimal fake AAudioStream.  The core never dereferences the struct directly
 * (it only passes the pointer back into AAudio* functions), so we can store
 * whatever we want and discriminate in usb_singstar_owns_stream().        */
struct AAudioStreamStruct {
    int _singstar_marker;   /* set to 0xCAFE5135 to identify our streams  */
};

#define SINGSTAR_MARKER 0xCAFE5135

static InputStreamEntry *find_entry(AAudioStream *s)
{
    for (int i = 0; i < g_input_count; i++)
        if (g_input[i].stream == s) return &g_input[i];
    return NULL;
}

/* Called by aaudio.c for every AAudioStream_* call to decide routing.
 * Must be fast (called on every getFramesRead, which is ~60/s).          */
int usb_singstar_owns_stream(AAudioStream *s)
{
    if (!s) return 0;
    /* Quick check: our fake structs have a known marker in the first word. */
    return s->_singstar_marker == SINGSTAR_MARKER;
}

/* =========================================================================
 * Input capture thread
 * ========================================================================= */

static void *input_capture_thread(void *arg)
{
    InputStreamEntry *e = (InputStreamEntry *)arg;

    int16_t *buf = (int16_t *)malloc((size_t)e->frames_per_cb * sizeof(int16_t));
    if (!buf) {
        e->thread_exited = 1;
        return NULL;
    }

    while (!e->thread_stop) {
        uint32_t avail = mic_nx_frames_available(e->player);
        if (avail < (uint32_t)e->frames_per_cb) {
            /* Not enough data yet — wait ~half a callback period            */
            int64_t wait_ns = (int64_t)e->frames_per_cb * 1000000000LL
                              / e->sample_rate / 2;
            svcSleepThread(wait_ns);
            continue;
        }

        uint32_t got = mic_nx_read(e->player, buf, (uint32_t)e->frames_per_cb);
        if (got < (uint32_t)e->frames_per_cb)
            memset(buf + got, 0,
                   (size_t)(e->frames_per_cb - (int32_t)got) * sizeof(int16_t));

        /* The AetherSX2 USBMic plugin reads PCM via AudioDevice::GetBuffer()
         * rather than AAudio data callbacks, so data_cb is normally NULL for
         * INPUT streams.  We call it anyway when present.                  */
        if (e->data_cb) {
            aaudio_data_callback_result_t r =
                e->data_cb(e->stream, e->data_user, buf, e->frames_per_cb);
            if (r == AAUDIO_CALLBACK_RESULT_STOP) break;
        }

        e->frames_delivered += e->frames_per_cb;
    }

    free(buf);
    e->thread_exited = 1;
    return NULL;
}

/* =========================================================================
 * Hooked AAudio entry points — called from aaudio.c
 * ========================================================================= */

aaudio_result_t usb_singstar_openStream_hook(AAudioStreamBuilder *b,
                                             AAudioStream **out)
{
    /* b->direction == AAUDIO_DIRECTION_INPUT is guaranteed by aaudio.c     */
    typedef struct { int32_t sample_rate, channel_count, device_id;
                     int32_t format, direction; } BuilderPeek;
    const BuilderPeek *peek = (const BuilderPeek *)b;

    mutexLock(&g_input_lock);

    if (g_input_count >= MAX_INPUT_STREAMS) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_NO_FREE_HANDLES;
    }

    InputStreamEntry *e = &g_input[g_input_count];
    memset(e, 0, sizeof(*e));

    /* Allocate a fake stream handle with our marker set.                   */
    e->stream = (AAudioStream *)calloc(1, sizeof(*e->stream));
    if (!e->stream) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_NO_MEMORY;
    }
    e->stream->_singstar_marker = SINGSTAR_MARKER;

    /* Player assignment: first INPUT stream → player 0, second → player 1. */
    e->player = 0;
    for (int i = 0; i < g_input_count; i++) {
        if (g_input[i].stream && g_input[i].player >= e->player)
            e->player = g_input[i].player + 1;
    }
    if (e->player >= MIC_NX_PLAYERS) e->player = MIC_NX_PLAYERS - 1;

    e->sample_rate    = peek->sample_rate > 0 ? peek->sample_rate : 48000;
    e->frames_per_cb  = 512;
    e->data_cb        = NULL;
    e->data_user      = NULL;
    e->running        = 0;
    e->thread_stop    = 0;
    e->thread_exited  = 0;
    e->frames_delivered = 0;

    g_input_count++;
    mutexUnlock(&g_input_lock);

    *out = e->stream;
    return AAUDIO_OK;
}

aaudio_result_t usb_singstar_requestStart_hook(AAudioStream *s)
{
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
        pthread_create(&e->thread, NULL, input_capture_thread, e);
    }
    mutexUnlock(&g_input_lock);
    return AAUDIO_OK;
}

aaudio_result_t usb_singstar_requestStop_hook(AAudioStream *s)
{
    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_INVALID_HANDLE;
    }

    if (e->running) {
        e->thread_stop = 1;
        for (int i = 0; i < 200 && !e->thread_exited; i++)
            svcSleepThread(5000000ULL);
        if (e->thread_exited) pthread_join(e->thread, NULL);
        e->running = 0;
    }
    mutexUnlock(&g_input_lock);
    return AAUDIO_OK;
}

aaudio_result_t usb_singstar_close_hook(AAudioStream *s)
{
    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return AAUDIO_ERROR_INVALID_HANDLE;
    }

    if (e->running) {
        e->thread_stop = 1;
        for (int i = 0; i < 200 && !e->thread_exited; i++)
            svcSleepThread(5000000ULL);
        if (e->thread_exited) pthread_join(e->thread, NULL);
        e->running = 0;
    }
    free(e->stream);

    /* Compact the table */
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
    mutexLock(&g_input_lock);
    InputStreamEntry *e = find_entry(s);
    if (!e) {
        mutexUnlock(&g_input_lock);
        return -1;  /* -1 signals "not our stream" to aaudio.c */
    }
    int64_t n = e->frames_delivered;
    mutexUnlock(&g_input_lock);
    return n;
}

/* =========================================================================
 * Prefs injection
 * ========================================================================= */

static void inject_singstar_prefs(void)
{
    /* These keys tell the AetherSX2 USB IOP emulation to instantiate a
     * SingStar dual-mic adapter on PS2 USB Port 1.  We use prefs_seed_public
     * so an explicit user entry in nethersx2.ini always wins.              */
    prefs_seed_public("USB/Port1/Type",                         "singstar");
    prefs_seed_public("USB/Port1/singstar/player1_device_name", "switch_usb_mic_0");
    prefs_seed_public("USB/Port1/singstar/player2_device_name", "switch_usb_mic_1");
    prefs_seed_public("USB/Port1/singstar/input_latency",       "20");
}

/* =========================================================================
 * Public API
 * ========================================================================= */

int usb_singstar_nx_init(void)
{
    mutexInit(&g_input_lock);
    g_input_count = 0;
    memset(g_input, 0, sizeof(g_input));

    /* Start physical USB capture (returns OK even if no device found yet)  */
    mic_nx_init();

    /* Inject prefs so the core creates a SingStar device on Port 1         */
    inject_singstar_prefs();

    /* No import-table patching needed: aaudio.c now directly calls
     * usb_singstar_openStream_hook / usb_singstar_owns_stream via the weak
     * symbol linkage defined in aaudio.c.                                  */

    return mic_nx_connected(MIC_NX_PLAYER1) ? 0 : 1;
}

void usb_singstar_nx_exit(void)
{
    /* Stop all active input streams                                         */
    mutexLock(&g_input_lock);
    for (int i = 0; i < g_input_count; i++) {
        InputStreamEntry *e = &g_input[i];
        if (e->running) {
            e->thread_stop = 1;
            for (int j = 0; j < 200 && !e->thread_exited; j++)
                svcSleepThread(5000000ULL);
            if (e->thread_exited) pthread_join(e->thread, NULL);
        }
        free(e->stream);
    }
    g_input_count = 0;
    mutexUnlock(&g_input_lock);

    mic_nx_exit();
}

int usb_singstar_nx_connected(void)
{
    return mic_nx_connected(MIC_NX_PLAYER1)
        || mic_nx_connected(MIC_NX_PLAYER2);
}
