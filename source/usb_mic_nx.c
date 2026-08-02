/* usb_mic_nx.c - UAC host driver for SingStar microphones on Nintendo Switch
 *
 * Uses libnx UsbHs (usb:hs) as USB host via the dock USB-A ports.
 * Enumerates UAC AudioStreaming interfaces, opens isochronous IN endpoints,
 * and feeds a lock-free ring buffer consumed by usb_singstar_nx.c.
 *
 * IMPORTANT: We call usbHsInitialize() ourselves and register an
 * InterfaceAvailableEvent. Relying solely on libusbhsfs is not enough -
 * its filter is MASS_STORAGE only, and without our own available-event the
 * host stack may not surface Audio interfaces to QueryAvailableInterfaces.
 *
 * MIT License - see LICENSE in the repository root.
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
 * Ring buffer - lock-free SPSC
 * ========================================================================= */

#define RING_CAP  MIC_NX_RING_FRAMES
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
 * Per-player state
 * ========================================================================= */

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
    uint8_t                *xfer_buf;
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

static int        g_usbhs_inited_by_us = 0;
static Event      g_iface_event;
static int        g_iface_event_ok = 0;
static u8         g_iface_event_index = 1; /* 0 often taken by libusbhsfs */

static mic_nx_osd_fn g_osd_cb = NULL;
static char          g_last_diag[128];

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
 * UsbHs bring-up
 * ========================================================================= */

static Result ensure_usbhs(void)
{
    /* Safe to call even if libusbhsfs already initialised usb:hs -
     * libnx returns success when the service is already active. */
    Result rc = usbHsInitialize();
    if (R_SUCCEEDED(rc))
        g_usbhs_inited_by_us = 1;
    /* If it failed because already inited by someone else, continue. */
    return 0;
}

static void setup_available_event(void)
{
    if (g_iface_event_ok) return;

    UsbHsInterfaceFilter filter;
    memset(&filter, 0, sizeof(filter));
    /* Flags MUST be non-zero on FW 7.0+. Match any Audio class interface. */
    filter.Flags = (u16)UsbHsInterfaceFilterFlags_bInterfaceClass;
    filter.bInterfaceClass = 0x01; /* Audio */

    /* Try event index 1, then 2, then 0. */
    for (u8 idx = 1; idx <= 2; idx++) {
        Result rc = usbHsCreateInterfaceAvailableEvent(&g_iface_event, true, idx, &filter);
        if (R_SUCCEEDED(rc)) {
            g_iface_event_index = idx;
            g_iface_event_ok = 1;
            return;
        }
    }
    Result rc = usbHsCreateInterfaceAvailableEvent(&g_iface_event, true, 0, &filter);
    if (R_SUCCEEDED(rc)) {
        g_iface_event_index = 0;
        g_iface_event_ok = 1;
    }
}

static void teardown_available_event(void)
{
    if (g_iface_event_ok) {
        usbHsDestroyInterfaceAvailableEvent(&g_iface_event, g_iface_event_index);
        g_iface_event_ok = 0;
    }
}

/* =========================================================================
 * Device matching
 * ========================================================================= */

typedef struct {
    uint16_t vid, pid;
    const char *name;
} KnownDevice;

static const KnownDevice k_known[] = {
    { SINGSTAR_VID, SINGSTAR_PID, "SingStar" },
    { LOGITECH_VID, 0x0000,       "Logitech" },
    { KONAMI_VID,   KONAMI_PID,   "Konami"   },
};
#define NUM_KNOWN (sizeof(k_known) / sizeof(k_known[0]))

static const char *device_name_for(const UsbHsInterface *iface)
{
    uint16_t vid = iface->device_desc.idVendor;
    uint16_t pid = iface->device_desc.idProduct;
    for (size_t i = 0; i < NUM_KNOWN; i++) {
        if (vid == k_known[i].vid &&
            (k_known[i].pid == 0x0000 || pid == k_known[i].pid))
            return k_known[i].name;
    }
    return "UAC";
}

/* =========================================================================
 * Open helpers
 * ========================================================================= */

static int open_audio_interface(UsbHsClientIfSession *out_iface,
                                UsbHsInterface *src_iface)
{
    Result rc = usbHsAcquireUsbIf(out_iface, src_iface);
    if (R_FAILED(rc))
        return -1;

    /* Try alt-setting 1 (streaming), then 2 (stereo on some SingStar). */
    usbHsIfSetInterface(out_iface, NULL, 1);
    return 0;
}

static int open_iso_ep(UsbHsClientEpSession *out_ep,
                       UsbHsClientIfSession  *iface)
{
    const UsbHsInterfaceInfo *info = &iface->inf.inf;

    for (int i = 0; i < 15; i++) {
        struct usb_endpoint_descriptor ep_desc = info->input_endpoint_descs[i];
        if (ep_desc.bLength == 0) break;

        uint8_t type = ep_desc.bmAttributes & 0x03u;
        /* 0x01 = isochronous, also accept interrupt (0x03) as fallback */
        if (type == 0x01u || type == 0x03u) {
            uint32_t mps = ep_desc.wMaxPacketSize;
            if (mps == 0) mps = 192;

            Result rc = usbHsIfOpenUsbEp(iface, out_ep, 1, ISO_XFER_SIZE, &ep_desc);
            if (R_SUCCEEDED(rc)) return 0;

            rc = usbHsIfOpenUsbEp(iface, out_ep, 1, (mps + 0xFFF) & ~0xFFFu, &ep_desc);
            if (R_SUCCEEDED(rc)) return 0;

            rc = usbHsIfOpenUsbEp(iface, out_ep, 1, mps, &ep_desc);
            if (R_SUCCEEDED(rc)) return 0;
        }
    }

    /* Last resort: open first non-empty IN endpoint regardless of type */
    for (int i = 0; i < 15; i++) {
        struct usb_endpoint_descriptor ep_desc = info->input_endpoint_descs[i];
        if (ep_desc.bLength == 0) break;
        uint32_t mps = ep_desc.wMaxPacketSize ? ep_desc.wMaxPacketSize : 64;
        Result rc = usbHsIfOpenUsbEp(iface, out_ep, 1, ISO_XFER_SIZE, &ep_desc);
        if (R_SUCCEEDED(rc)) return 0;
        (void)mps;
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
        Result rc = usbHsEpPostBuffer(&p->ep, p->xfer_buf, ISO_XFER_SIZE, &transferred);

        if (p->stop_flag) break;

        if (R_FAILED(rc) || transferred == 0) {
            svcSleepThread(2000000ULL); /* 2 ms */
            continue;
        }

        uint32_t frames = transferred / MIC_NX_FRAME_BYTES;
        if (frames == 0) continue;

        int16_t *samples = (int16_t *)(void *)p->xfer_buf;
        apply_volume(samples, frames, p->vol);
        ring_write(&p->ring, samples, frames);
    }
    return NULL;
}

/* =========================================================================
 * Enumerate + probe
 * ========================================================================= */

#define MAX_IFACES 16

/* Try several filters and return total interfaces found into `out`. */
static s32 query_audio_interfaces(UsbHsInterface *out, s32 max_count, Result *out_rc)
{
    s32 total = 0;
    Result rc;

    /* 1) Audio class only (covers control + streaming) */
    {
        UsbHsInterfaceFilter f;
        memset(&f, 0, sizeof(f));
        f.Flags = (u16)UsbHsInterfaceFilterFlags_bInterfaceClass;
        f.bInterfaceClass = 0x01;
        rc = usbHsQueryAvailableInterfaces(&f, out, (size_t)max_count * sizeof(UsbHsInterface), &total);
        if (out_rc) *out_rc = rc;
        if (R_SUCCEEDED(rc) && total > 0) return total;
    }

    /* 2) Audio + AudioStreaming subclass */
    {
        UsbHsInterfaceFilter f;
        memset(&f, 0, sizeof(f));
        f.Flags = (u16)(UsbHsInterfaceFilterFlags_bInterfaceClass |
                        UsbHsInterfaceFilterFlags_bInterfaceSubClass);
        f.bInterfaceClass    = 0x01;
        f.bInterfaceSubClass = 0x02;
        total = 0;
        rc = usbHsQueryAvailableInterfaces(&f, out, (size_t)max_count * sizeof(UsbHsInterface), &total);
        if (out_rc) *out_rc = rc;
        if (R_SUCCEEDED(rc) && total > 0) return total;
    }

    /* 3) Known SingStar VID */
    {
        UsbHsInterfaceFilter f;
        memset(&f, 0, sizeof(f));
        f.Flags = (u16)UsbHsInterfaceFilterFlags_idVendor;
        f.idVendor = SINGSTAR_VID;
        total = 0;
        rc = usbHsQueryAvailableInterfaces(&f, out, (size_t)max_count * sizeof(UsbHsInterface), &total);
        if (out_rc) *out_rc = rc;
        if (R_SUCCEEDED(rc) && total > 0) return total;
    }

    /* 4) Logitech VID */
    {
        UsbHsInterfaceFilter f;
        memset(&f, 0, sizeof(f));
        f.Flags = (u16)UsbHsInterfaceFilterFlags_idVendor;
        f.idVendor = LOGITECH_VID;
        total = 0;
        rc = usbHsQueryAvailableInterfaces(&f, out, (size_t)max_count * sizeof(UsbHsInterface), &total);
        if (out_rc) *out_rc = rc;
        if (R_SUCCEEDED(rc) && total > 0) return total;
    }

    /* 5) Any already-acquired interfaces (in case another layer claimed them) */
    {
        total = 0;
        rc = usbHsQueryAcquiredInterfaces(out, (size_t)max_count * sizeof(UsbHsInterface), &total);
        if (out_rc) *out_rc = rc;
        if (R_SUCCEEDED(rc) && total > 0) return total;
    }

    return 0;
}

static void release_player(MicPlayer *p)
{
    if (p->active) {
        p->stop_flag = 1;
        pthread_join(p->thread, NULL);
        p->active = 0;
    }
    if (p->ep_open)    { usbHsEpClose(&p->ep);    p->ep_open = 0; }
    if (p->iface_open) { usbHsIfClose(&p->iface); p->iface_open = 0; }
    ring_init(&p->ring);
}

static int start_player_from_iface(MicPlayer *p, UsbHsInterface *iface)
{
    if (p->active) return MIC_NX_OK;

    if (!p->xfer_buf) {
        p->xfer_buf = (uint8_t *)aligned_alloc(0x1000, ISO_XFER_SIZE);
        if (!p->xfer_buf) return MIC_NX_ERR_INIT;
        memset(p->xfer_buf, 0, ISO_XFER_SIZE);
    }

    if (open_audio_interface(&p->iface, iface) != 0)
        return MIC_NX_ERR_INIT;
    p->iface_open = 1;

    if (open_iso_ep(&p->ep, &p->iface) != 0) {
        usbHsIfClose(&p->iface);
        p->iface_open = 0;
        return MIC_NX_ERR_INIT;
    }
    p->ep_open = 1;

    p->last_vid = iface->device_desc.idVendor;
    p->last_pid = iface->device_desc.idProduct;
    snprintf(p->last_name, sizeof(p->last_name), "%s", device_name_for(iface));

    p->stop_flag = 0;
    if (pthread_create(&p->thread, NULL, capture_thread, p) != 0) {
        release_player(p);
        return MIC_NX_ERR_INIT;
    }
    p->active = 1;
    return MIC_NX_OK;
}

static int try_probe_all(void)
{
    UsbHsInterface ifaces[MAX_IFACES];
    memset(ifaces, 0, sizeof(ifaces));
    Result qrc = 0;
    s32 total = query_audio_interfaces(ifaces, MAX_IFACES, &qrc);

    snprintf(g_last_diag, sizeof(g_last_diag),
             "USB q:rc=0x%08X n=%d", (unsigned)qrc, (int)total);

    if (total <= 0)
        return 0;

    int started = 0;
    /* Prefer AudioStreaming (subclass 0x02) interfaces; fall back to any. */
    for (int pass = 0; pass < 2; pass++) {
        for (s32 i = 0; i < total; i++) {
            u8 sub = ifaces[i].inf.interface_desc.bInterfaceSubClass;
            u8 cls = ifaces[i].inf.interface_desc.bInterfaceClass;
            if (pass == 0 && !(cls == 0x01 && sub == 0x02))
                continue;
            if (pass == 1 && cls != 0x01)
                continue;

            /* Assign to first free player slot */
            for (int p = 0; p < MIC_NX_PLAYERS; p++) {
                if (g_players[p].active) continue;
                if (start_player_from_iface(&g_players[p], &ifaces[i]) == MIC_NX_OK) {
                    started++;
                    char msg[96];
                    snprintf(msg, sizeof(msg),
                             "USB Mic P%d: %s %04X:%04X",
                             p + 1, g_players[p].last_name,
                             g_players[p].last_vid, g_players[p].last_pid);
                    osd("usb_mic_conn", msg, 5.0f);
                }
                break;
            }
            if (started >= MIC_NX_PLAYERS) return started;
        }
        if (started > 0) return started;
    }
    return started;
}

/* =========================================================================
 * Hotplug
 * ========================================================================= */

static void *hotplug_thread(void *arg)
{
    (void)arg;
    svcSleepThread(1000000000ULL); /* 1 s settle */

    while (!g_hotplug_stop) {
        if (!mic_nx_any_connected())
            try_probe_all();

        for (int i = 0; i < 20 && !g_hotplug_stop; i++)
            svcSleepThread(100000000ULL); /* 100 ms * 20 = 2 s */
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

    for (int i = 0; i < MIC_NX_PLAYERS; i++) {
        memset(&g_players[i], 0, sizeof(g_players[i]));
        g_players[i].player = i;
        g_players[i].vol = 240;
        ring_init(&g_players[i].ring);
    }

    ensure_usbhs();
    setup_available_event();

    /* Give the dock / host controller a moment after storage init. */
    svcSleepThread(500000000ULL); /* 0.5 s */

    int n = try_probe_all();

    g_init_done = 1;

    g_hotplug_stop = 0;
    if (pthread_create(&g_hotplug_thread, NULL, hotplug_thread, NULL) == 0)
        g_hotplug_running = 1;

    mutexUnlock(&g_init_mutex);

    if (n == 0) {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "USB Mic: none (%s). Dock + USB-A, wait 2s",
                 g_last_diag);
        osd("usb_mic_none", msg, 8.0f);
        return MIC_NX_ERR_NODEV;
    }
    return MIC_NX_OK;
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
        release_player(&g_players[i]);
        free(g_players[i].xfer_buf);
        g_players[i].xfer_buf = NULL;
    }

    teardown_available_event();
    /* Do NOT call usbHsExit() - libusbhsfs / other code may still need it. */

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
    if (!mic_nx_any_connected())
        try_probe_all();
}
