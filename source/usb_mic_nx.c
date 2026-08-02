/* usb_mic_nx.c — UAC host driver for SingStar microphones on Nintendo Switch
 *
 * Architecture overview
 * ─────────────────────
 * The Switch USB stack (libnx UsbHs) can act as a USB host through the dock's
 * USB-A ports.  A SingStar mic adapter presents itself as a USB Audio Class
 * (UAC 1.0) device with isochronous IN endpoints — one per microphone capsule.
 *
 * This module:
 *  1. Waits (non-blocking) for a device matching known SingStar VID/PIDs
 *     (or any UAC AudioStreaming interface).
 *  2. Opens the AudioStreaming interface, selects alternate setting 1,
 *     and opens the isochronous IN endpoint.
 *  3. Runs a capture thread per player that submits usbHsEpPostBuffer()
 *     synchronous transfers and writes the received PCM into a lock-free
 *     power-of-2 ring buffer.
 *  4. Exposes mic_nx_read() / mic_nx_frames_available() for the PS2 USB
 *     gadget layer (usb_singstar_nx.c).
 *  5. Runs a lightweight hotplug thread so mics plugged in after boot are
 *     picked up without restarting the emulator.
 *
 * usbHs ownership
 * ───────────────
 * We do NOT call usbHsInitialize() / usbHsExit().  libusbhsfs (used by
 * SwitchStorage for mass-storage devices) already owns that lifecycle via
 * usbHsFsInitialize().  Our filter targets bInterfaceClass=0x01 (Audio),
 * which is entirely disjoint from libusbhsfs's MASS_STORAGE filter, so
 * the two subsystems never contend for the same interface.
 *
 * mic_nx_init() MUST therefore be called after switchStorageInitializeForPath()
 * (which triggers usbHsFsInitialize).  The call site in main.c already
 * guarantees this ordering.
 *
 * MIT License — see LICENSE in the repository root.
 */

#define _GNU_SOURCE
#include <switch.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdatomic.h>

#include "usb_mic_nx.h"

/* =========================================================================
 * Ring buffer — lock-free single-producer / single-consumer
 * ========================================================================= */

#define RING_CAP  MIC_NX_RING_FRAMES          /* must be power of 2        */
#define RING_MASK (RING_CAP - 1)

typedef struct {
    int16_t  buf[RING_CAP];
    atomic_uint_fast32_t  rpos;
    atomic_uint_fast32_t  wpos;
} MicRing;

static void ring_init(MicRing *r)
{
    memset(r->buf, 0, sizeof(r->buf));
    atomic_store_explicit(&r->rpos, 0, memory_order_relaxed);
    atomic_store_explicit(&r->wpos, 0, memory_order_relaxed);
}

static uint32_t ring_avail(const MicRing *r)
{
    uint32_t w  = atomic_load_explicit(&r->wpos, memory_order_acquire);
    uint32_t rd = atomic_load_explicit(&r->rpos, memory_order_relaxed);
    return (w - rd) & RING_MASK;
}

static void ring_write(MicRing *r, const int16_t *src, uint32_t frames)
{
    uint32_t w  = atomic_load_explicit(&r->wpos, memory_order_relaxed);
    uint32_t rd = atomic_load_explicit(&r->rpos, memory_order_acquire);
    uint32_t free_space = (RING_CAP - 1) - ((w - rd) & RING_MASK);

    if (frames > free_space) {
        uint32_t drop = frames - free_space;
        atomic_fetch_add_explicit(&r->rpos, drop, memory_order_relaxed);
    }

    for (uint32_t i = 0; i < frames; i++)
        r->buf[(w + i) & RING_MASK] = src[i];

    atomic_fetch_add_explicit(&r->wpos, frames, memory_order_release);
}

static uint32_t ring_read(MicRing *r, int16_t *dst, uint32_t frames)
{
    uint32_t avail = ring_avail(r);
    uint32_t n  = (frames < avail) ? frames : avail;
    uint32_t rd = atomic_load_explicit(&r->rpos, memory_order_relaxed);
    for (uint32_t i = 0; i < n; i++)
        dst[i] = r->buf[(rd + i) & RING_MASK];
    atomic_fetch_add_explicit(&r->rpos, n, memory_order_release);
    return n;
}

/* =========================================================================
 * Per-player capture state
 * ========================================================================= */

/*
 * Maximum transfer size for a 48 kHz mono 16-bit mic at 1 ms/frame:
 *   48 samples * 2 bytes = 96 bytes.  Round up to page size (0x1000).
 * usbHsEpPostBuffer requires the buffer address AND size to be 0x1000-aligned.
 */
#define ISO_XFER_SIZE  0x1000u

typedef struct {
    int                     player;
    int                     active;
    UsbHsClientIfSession    iface;
    UsbHsClientEpSession    ep;
    int                     iface_open;
    int                     ep_open;
    pthread_t               thread;
    volatile int            stop_flag;
    MicRing                 ring;
    uint8_t                 vol;
    uint8_t                *xfer_buf;   /* ISO_XFER_SIZE, 0x1000-aligned */
    uint16_t                last_vid;
    uint16_t                last_pid;
    char                    last_name[32];
} MicPlayer;

static MicPlayer  g_players[MIC_NX_PLAYERS];
static int        g_init_done = 0;
static Mutex      g_init_mutex;
static int        g_mutex_ready = 0;

static pthread_t  g_hotplug_thread;
static volatile int g_hotplug_stop = 0;
static int        g_hotplug_running = 0;

static mic_nx_osd_fn g_osd_cb = NULL;

void mic_nx_set_osd_callback(mic_nx_osd_fn fn)
{
    g_osd_cb = fn;
}

static void osd(const char *key, const char *msg, float dur)
{
    if (g_osd_cb)
        g_osd_cb(key, msg, dur);
}

/* =========================================================================
 * USB device matching
 * ========================================================================= */

typedef struct {
    uint16_t vid, pid;  /* pid == 0 → match any PID from this VID */
    const char *name;
} KnownDevice;

static const KnownDevice k_known[] = {
    { SINGSTAR_VID, SINGSTAR_PID, "SingStar" },
    { LOGITECH_VID, 0x0000,       "Logitech" },
    { KONAMI_VID,   KONAMI_PID,   "Konami"   },
};
#define NUM_KNOWN (sizeof(k_known) / sizeof(k_known[0]))

static int device_matches(const UsbHsInterface *iface, const char **out_name)
{
    uint16_t vid = iface->device_desc.idVendor;
    uint16_t pid = iface->device_desc.idProduct;
    for (size_t i = 0; i < NUM_KNOWN; i++) {
        if (vid == k_known[i].vid &&
            (k_known[i].pid == 0x0000 || pid == k_known[i].pid)) {
            if (out_name) *out_name = k_known[i].name;
            return 1;
        }
    }
    /* Accept any UAC AudioStreaming interface regardless of VID/PID */
    if (out_name) *out_name = "UAC mic";
    return 1;
}

/* =========================================================================
 * Interface / endpoint helpers
 * ========================================================================= */

static int open_audio_interface(UsbHsClientIfSession *out_iface,
                                UsbHsInterface *src_iface)
{
    Result rc = usbHsAcquireUsbIf(out_iface, src_iface);
    if (R_FAILED(rc))
        return -1;

    /* Select alternate setting 1 (active streaming, non-zero bandwidth).
     * Some devices only expose alt 1 after the SET_INTERFACE; ignore the
     * return and try the EP open either way. */
    usbHsIfSetInterface(out_iface, NULL, 1);

    return 0;
}

/*
 * Find the first isochronous IN endpoint and open it.
 * bmAttributes & 0x03 == 0x01 → isochronous.
 */
static int open_iso_ep(UsbHsClientEpSession *out_ep,
                       UsbHsClientIfSession  *iface)
{
    const UsbHsInterfaceInfo *info = &iface->inf.inf;

    for (int i = 0; i < 15; i++) {
        struct usb_endpoint_descriptor ep_desc = info->input_endpoint_descs[i];

        if (ep_desc.bLength == 0) break;

        uint8_t type = ep_desc.bmAttributes & 0x03u;
        if (type == 0x01u) {               /* isochronous */
            uint32_t mps = ep_desc.wMaxPacketSize;
            if (mps == 0) mps = ISO_XFER_SIZE;
            /* Some stacks want the transfer size >= wMaxPacketSize and
             * still page-aligned.  Keep ISO_XFER_SIZE (0x1000). */
            Result rc = usbHsIfOpenUsbEp(iface, out_ep, 1, ISO_XFER_SIZE, &ep_desc);
            if (R_SUCCEEDED(rc)) return 0;
            /* Fallback: try with the descriptor's own MPS */
            rc = usbHsIfOpenUsbEp(iface, out_ep, 1, mps, &ep_desc);
            if (R_SUCCEEDED(rc)) return 0;
        }
    }
    return -1;
}

/* =========================================================================
 * Capture thread
 * ========================================================================= */

static void apply_volume(int16_t *buf, uint32_t frames, uint8_t vol)
{
    if (vol == 255) return;
    for (uint32_t i = 0; i < frames; i++)
        buf[i] = (int16_t)((int32_t)buf[i] * vol / 255);
}

static void *capture_thread(void *arg)
{
    MicPlayer *p = (MicPlayer *)arg;

    while (!p->stop_flag) {
        uint32_t transferred = 0;

        Result rc = usbHsEpPostBuffer(&p->ep,
                                      p->xfer_buf,
                                      ISO_XFER_SIZE,
                                      &transferred);

        if (p->stop_flag) break;

        if (R_FAILED(rc) || transferred == 0) {
            svcSleepThread(1000000ULL);   /* 1 ms back-off */
            continue;
        }

        /* Only use whole 16-bit frames; discard a possible trailing byte. */
        uint32_t frames = transferred / MIC_NX_FRAME_BYTES;
        if (frames == 0) continue;

        int16_t *samples = (int16_t *)(void *)p->xfer_buf;
        apply_volume(samples, frames, p->vol);
        ring_write(&p->ring, samples, frames);
    }

    return NULL;
}

/* =========================================================================
 * Device probe / release
 * ========================================================================= */

#define MAX_IFACES 8

static void release_player(MicPlayer *p)
{
    if (p->active) {
        p->stop_flag = 1;
        pthread_join(p->thread, NULL);
        p->active = 0;
    }
    if (p->ep_open) {
        usbHsEpClose(&p->ep);
        p->ep_open = 0;
    }
    if (p->iface_open) {
        usbHsIfClose(&p->iface);
        p->iface_open = 0;
    }
    ring_init(&p->ring);
}

static int probe_player(MicPlayer *p)
{
    /*
     * Filter on bInterfaceClass=0x01 (Audio) and bInterfaceSubClass=0x02
     * (AudioStreaming).  Any UAC mic is accepted.
     */
    UsbHsInterfaceFilter filter;
    memset(&filter, 0, sizeof(filter));
    filter.Flags = (u16)(UsbHsInterfaceFilterFlags_bInterfaceClass |
                         UsbHsInterfaceFilterFlags_bInterfaceSubClass);
    filter.bInterfaceClass    = 0x01;   /* Audio            */
    filter.bInterfaceSubClass = 0x02;   /* AudioStreaming   */

    UsbHsInterface ifaces[MAX_IFACES];
    memset(ifaces, 0, sizeof(ifaces));
    s32 total = 0;

    Result rc = usbHsQueryAvailableInterfaces(&filter, ifaces,
                                              sizeof(ifaces), &total);
    if (R_FAILED(rc) || total <= 0)
        return MIC_NX_ERR_NODEV;

    /* Player 0 → first interface, Player 1 → second interface.
     * If only one interface is present, Player 1 stays silent. */
    if (p->player >= total)
        return MIC_NX_ERR_NODEV;

    const char *dev_name = NULL;
    device_matches(&ifaces[p->player], &dev_name);

    if (open_audio_interface(&p->iface, &ifaces[p->player]) != 0)
        return MIC_NX_ERR_INIT;
    p->iface_open = 1;

    if (open_iso_ep(&p->ep, &p->iface) != 0) {
        usbHsIfClose(&p->iface);
        p->iface_open = 0;
        return MIC_NX_ERR_INIT;
    }
    p->ep_open = 1;

    p->last_vid = ifaces[p->player].device_desc.idVendor;
    p->last_pid = ifaces[p->player].device_desc.idProduct;
    snprintf(p->last_name, sizeof(p->last_name), "%s",
             dev_name ? dev_name : "UAC mic");

    return MIC_NX_OK;
}

static int start_player(MicPlayer *p)
{
    if (p->active) return MIC_NX_OK;

    if (!p->xfer_buf) {
        p->xfer_buf = (uint8_t *)aligned_alloc(0x1000, ISO_XFER_SIZE);
        if (!p->xfer_buf) return MIC_NX_ERR_INIT;
        memset(p->xfer_buf, 0, ISO_XFER_SIZE);
    }

    if (probe_player(p) != MIC_NX_OK)
        return MIC_NX_ERR_NODEV;

    p->stop_flag = 0;
    if (pthread_create(&p->thread, NULL, capture_thread, p) != 0) {
        release_player(p);
        return MIC_NX_ERR_INIT;
    }

    p->active = 1;
    return MIC_NX_OK;
}

/* =========================================================================
 * Hotplug thread — re-probe inactive players every ~1.5 s
 * ========================================================================= */

static void try_hotplug(void)
{
    for (int i = 0; i < MIC_NX_PLAYERS; i++) {
        MicPlayer *p = &g_players[i];
        if (p->active) continue;

        if (start_player(p) == MIC_NX_OK) {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "USB Mic P%d connected (%s %04x:%04x)",
                     i + 1, p->last_name, p->last_vid, p->last_pid);
            osd("usb_mic_conn", msg, 4.0f);
        }
    }
}

static void *hotplug_thread(void *arg)
{
    (void)arg;
    /* Initial delay so UsbHs / dock power settle after boot. */
    svcSleepThread(1500000000ULL);  /* 1.5 s */

    while (!g_hotplug_stop) {
        try_hotplug();
        /* Poll every 1.5 s */
        for (int i = 0; i < 15 && !g_hotplug_stop; i++)
            svcSleepThread(100000000ULL);  /* 100 ms * 15 */
    }
    return NULL;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

int mic_nx_init(void)
{
    if (!g_mutex_ready) {
        mutexInit(&g_init_mutex);
        g_mutex_ready = 1;
    }

    mutexLock(&g_init_mutex);

    if (g_init_done) {
        mutexUnlock(&g_init_mutex);
        return MIC_NX_ERR_BUSY;
    }

    int any_ok = 0;

    for (int i = 0; i < MIC_NX_PLAYERS; i++) {
        MicPlayer *p = &g_players[i];
        memset(p, 0, sizeof(*p));
        p->player = i;
        p->vol    = 240;
        ring_init(&p->ring);

        p->xfer_buf = (uint8_t *)aligned_alloc(0x1000, ISO_XFER_SIZE);
        if (!p->xfer_buf) continue;
        memset(p->xfer_buf, 0, ISO_XFER_SIZE);

        if (start_player(p) == MIC_NX_OK) {
            any_ok = 1;
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "USB Mic P%d connected (%s %04x:%04x)",
                     i + 1, p->last_name, p->last_vid, p->last_pid);
            osd("usb_mic_conn", msg, 4.0f);
        }
    }

    g_init_done = 1;

    /* Always start hotplug so late-docked mics are found. */
    g_hotplug_stop = 0;
    if (pthread_create(&g_hotplug_thread, NULL, hotplug_thread, NULL) == 0)
        g_hotplug_running = 1;

    mutexUnlock(&g_init_mutex);

    if (!any_ok)
        osd("usb_mic_none",
            "USB Mic: no device yet (plug into dock USB-A)", 5.0f);

    return any_ok ? MIC_NX_OK : MIC_NX_ERR_NODEV;
}

void mic_nx_exit(void)
{
    if (!g_mutex_ready) return;

    mutexLock(&g_init_mutex);
    if (!g_init_done) {
        mutexUnlock(&g_init_mutex);
        return;
    }

    if (g_hotplug_running) {
        g_hotplug_stop = 1;
        pthread_join(g_hotplug_thread, NULL);
        g_hotplug_running = 0;
    }

    for (int i = 0; i < MIC_NX_PLAYERS; i++) {
        MicPlayer *p = &g_players[i];
        release_player(p);
        if (p->xfer_buf) {
            free(p->xfer_buf);
            p->xfer_buf = NULL;
        }
    }

    g_init_done = 0;
    mutexUnlock(&g_init_mutex);
}

int mic_nx_connected(int player)
{
    if (player < 0 || player >= MIC_NX_PLAYERS) return 0;
    return g_players[player].active;
}

int mic_nx_any_connected(void)
{
    return mic_nx_connected(0) || mic_nx_connected(1);
}

uint32_t mic_nx_read(int player, int16_t *dst, uint32_t frames)
{
    if (player < 0 || player >= MIC_NX_PLAYERS || !dst || !frames) return 0;
    MicPlayer *p = &g_players[player];
    if (!p->active) {
        memset(dst, 0, (size_t)frames * MIC_NX_FRAME_BYTES);
        return 0;
    }
    return ring_read(&p->ring, dst, frames);
}

uint32_t mic_nx_frames_available(int player)
{
    if (player < 0 || player >= MIC_NX_PLAYERS) return 0;
    return ring_avail(&g_players[player].ring);
}

void mic_nx_set_volume(int player, uint8_t vol)
{
    if (player < 0 || player >= MIC_NX_PLAYERS) return;
    g_players[player].vol = vol;
}

void mic_nx_poll(void)
{
    if (!g_init_done) return;
    try_hotplug();
}
