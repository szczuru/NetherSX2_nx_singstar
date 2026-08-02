/* usb_mic_nx.c — UAC host driver for SingStar microphones on Nintendo Switch
 *
 * Architecture overview
 * ─────────────────────
 * The Switch USB stack (libnx UsbHs) can act as a USB host through the dock's
 * USB-A ports.  A SingStar mic adapter presents itself as a USB Audio Class
 * (UAC 1.0) device with two isochronous IN endpoints — one per microphone.
 *
 * This module:
 *  1. Waits (non-blocking) for a device matching known SingStar VID/PIDs.
 *  2. Opens the AudioStreaming interface, selects alternate setting 1 (mono)
 *     or 2 (dual-channel), and opens the isochronous IN endpoint.
 *  3. Runs a capture thread per player that submits UsbHsEpSubmitRequest
 *     isochronous transfers and writes the received PCM into a lock-free
 *     power-of-2 ring buffer.
 *  4. Exposes mic_nx_read() / mic_nx_frames_available() for the PS2 USB
 *     gadget layer (usb_singstar_nx.c).
 *
 * Isochronous note
 * ─────────────────
 * libnx's UsbHs exposes isochronous transfers through
 * usbHsEpSubmitRequest() with a UsbHsClientEpSession opened via
 * usbHsIfOpenUsbEp().  We use synchronous (timeout-based) submission in the
 * capture thread rather than async callbacks, which keeps the code simple and
 * avoids ABI gymnastics with the so-loader environment.
 *
 * MIT License — see LICENSE in the repository root.
 */

#define _GNU_SOURCE
#include <switch.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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
    atomic_uint_fast32_t  rpos;   /* consumer read position  */
    atomic_uint_fast32_t  wpos;   /* producer write position */
} MicRing;

static void ring_init(MicRing *r)
{
    memset(r->buf, 0, sizeof(r->buf));
    atomic_store_explicit(&r->rpos, 0, memory_order_relaxed);
    atomic_store_explicit(&r->wpos, 0, memory_order_relaxed);
}

static uint32_t ring_avail(const MicRing *r)
{
    uint32_t w = atomic_load_explicit(&r->wpos, memory_order_acquire);
    uint32_t rd = atomic_load_explicit(&r->rpos, memory_order_relaxed);
    return (w - rd) & RING_MASK;
}

/* Write frames; overwrites oldest if full (keep capture alive without stall) */
static void ring_write(MicRing *r, const int16_t *src, uint32_t frames)
{
    uint32_t w = atomic_load_explicit(&r->wpos, memory_order_relaxed);
    uint32_t rd = atomic_load_explicit(&r->rpos, memory_order_acquire);
    uint32_t free_space = (RING_CAP - 1) - ((w - rd) & RING_MASK);

    /* If we would overflow, advance rpos to make room (drops oldest data) */
    if (frames > free_space) {
        uint32_t drop = frames - free_space;
        atomic_fetch_add_explicit(&r->rpos, drop, memory_order_relaxed);
    }

    for (uint32_t i = 0; i < frames; i++) {
        r->buf[(w + i) & RING_MASK] = src[i];
    }
    atomic_fetch_add_explicit(&r->wpos, frames, memory_order_release);
}

/* Read up to `frames` samples.  Returns count actually read. */
static uint32_t ring_read(MicRing *r, int16_t *dst, uint32_t frames)
{
    uint32_t avail = ring_avail(r);
    uint32_t n = (frames < avail) ? frames : avail;
    uint32_t rd = atomic_load_explicit(&r->rpos, memory_order_relaxed);
    for (uint32_t i = 0; i < n; i++) {
        dst[i] = r->buf[(rd + i) & RING_MASK];
    }
    atomic_fetch_add_explicit(&r->rpos, n, memory_order_release);
    return n;
}

/* =========================================================================
 * Per-player capture state
 * ========================================================================= */

/* Maximum isochronous packet size for a 48 kHz mono 16-bit mic at 1 ms/frame:
 *   48 samples * 2 bytes = 96 bytes.  Round up to 200 (matches PCSX2). */
#define ISO_PACKET_SIZE  200
/* Number of transfer buffers to keep in flight */
#define ISO_NUM_XFERS    4

typedef struct {
    int                     player;          /* 0 or 1                        */
    int                     active;          /* 1 = thread running            */
    UsbHsClientIfSession    iface;           /* opened USB interface          */
    UsbHsClientEpSession    ep;              /* isochronous IN endpoint       */
    int                     iface_open;
    int                     ep_open;
    pthread_t               thread;
    volatile int            stop_flag;
    MicRing                 ring;
    uint8_t                 vol;            /* software gain [0..255]        */

    /* DMA-aligned transfer buffers */
    uint8_t  *xfer_buf;                     /* ISO_NUM_XFERS * ISO_PACKET_SIZE */
} MicPlayer;

static MicPlayer  g_players[MIC_NX_PLAYERS];
static int        g_init_done = 0;
static Mutex      g_init_mutex;

/* =========================================================================
 * USB device matching
 * ========================================================================= */

typedef struct {
    uint16_t vid, pid;
    const char *name;
} KnownDevice;

static const KnownDevice k_known[] = {
    { SINGSTAR_VID, SINGSTAR_PID, "SingStar"  },
    { LOGITECH_VID, 0x0000,       "Logitech"  },  /* match any Logitech PID */
    { KONAMI_VID,   KONAMI_PID,   "Konami"    },
};
#define NUM_KNOWN (sizeof(k_known) / sizeof(k_known[0]))

static int device_matches(const UsbHsDeviceFilter *filter,
                          const UsbHsInterface *iface,
                          const char **out_name)
{
    (void)filter;
    uint16_t vid = iface->device_desc.idVendor;
    uint16_t pid = iface->device_desc.idProduct;
    for (size_t i = 0; i < NUM_KNOWN; i++) {
        if (vid == k_known[i].vid &&
            (k_known[i].pid == 0x0000 || pid == k_known[i].pid)) {
            if (out_name) *out_name = k_known[i].name;
            return 1;
        }
    }
    return 0;
}

/* =========================================================================
 * Interface / endpoint helpers
 * ========================================================================= */

/* Find the AudioStreaming interface and open it.
 * We look for bInterfaceClass=0x01 (Audio), bInterfaceSubClass=0x02 (Streaming),
 * alternate setting 1 (active, mono) or 2 (active, stereo).
 * Returns the bInterfaceNumber on success, -1 on failure. */
static int open_audio_interface(UsbHsClientIfSession *out_iface,
                                UsbHsInterface *src_iface)
{
    /* usbHsAcquireUsbIf takes ownership of the interface slot */
    Result rc = usbHsAcquireUsbIf(out_iface, src_iface);
    if (R_FAILED(rc)) {

        return -1;
    }

    /* Select alternate setting 1 (single-channel, active streaming) */
    rc = usbHsIfSetInterface(out_iface, NULL, 1);
    if (R_FAILED(rc)) {
        /* Some devices only expose alt-setting 0 initially; try anyway */
        usbHsIfResetDevice(out_iface);
    }

    return (int)src_iface->inf.interface_desc.bInterfaceNumber;
}

/* Find the first isochronous IN endpoint in the open interface.
 * wMaxPacketSize is set to at least ISO_PACKET_SIZE. */
static int open_iso_ep(UsbHsClientEpSession *out_ep,
                       UsbHsClientIfSession *iface)
{
    /* Walk the cached endpoint descriptors in the UsbHsInterfaceInfo */
    const UsbHsInterfaceInfo *info = &iface->inf;
    for (int i = 0; i < 15; i++) {
        const struct usb_endpoint_descriptor *ep = &info->input_endpoint_descs[i];
        /* bLength == 0 → no more endpoints */
        if (ep->bLength == 0) break;
        /* bEndpointAddress bit7=1 → IN; bmAttributes & 0x03 == 0x01 → Isochronous */
        uint8_t addr  = ep->bEndpointAddress;
        uint8_t type  = ep->bmAttributes & 0x03u;
        if ((addr & 0x80u) && type == USB_ENDPOINT_XFER_ISOC) {
            uint16_t mps = ep->wMaxPacketSize;
            if (mps < ISO_PACKET_SIZE) mps = ISO_PACKET_SIZE;
            Result rc = usbHsIfOpenUsbEp(iface, out_ep, 1, mps, ep);
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
    if (vol == 255) return;   /* unity — skip */
    for (uint32_t i = 0; i < frames; i++) {
        buf[i] = (int16_t)((int32_t)buf[i] * vol / 255);
    }
}

static void *capture_thread(void *arg)
{
    MicPlayer *p = (MicPlayer *)arg;
    uint8_t *xbuf = p->xfer_buf;

    while (!p->stop_flag) {
        uint32_t bytes_transferred = 0;
        /* Synchronous isochronous read — 5 ms timeout per packet */
        Result rc = usbHsEpSubmitRequest(&p->ep,
                                         xbuf,
                                         ISO_PACKET_SIZE,
                                         5000000ULL,   /* 5 ms in nanoseconds */
                                         &bytes_transferred);
        if (p->stop_flag) break;

        if (R_FAILED(rc) || bytes_transferred == 0) {
            /* Transient USB error or device disconnected — sleep and retry */
            svcSleepThread(1000000ULL);  /* 1 ms */
            continue;
        }

        uint32_t frames = bytes_transferred / MIC_NX_FRAME_BYTES;
        int16_t *samples = (int16_t *)xbuf;

        apply_volume(samples, frames, p->vol);
        ring_write(&p->ring, samples, frames);
    }

    return NULL;
}

/* =========================================================================
 * Device probe: try to find and open a UAC mic for the given player index.
 * Returns 0 on success, negative on failure.
 * ========================================================================= */

#define MAX_IFACES 8

static int probe_player(MicPlayer *p)
{
    UsbHsDeviceFilter filter;
    memset(&filter, 0, sizeof(filter));
    /* Match on bInterfaceClass=0x01 (Audio), bInterfaceSubClass=0x02 */
    filter.Flags = UsbHsDeviceFilterFlags_bcdDevice_Min
                 | UsbHsDeviceFilterFlags_bInterfaceClass
                 | UsbHsDeviceFilterFlags_bInterfaceSubClass;
    filter.bcdDevice_Min   = 0x0000;
    filter.bInterfaceClass    = 0x01;  /* Audio */
    filter.bInterfaceSubClass = 0x02;  /* AudioStreaming */

    UsbHsInterface ifaces[MAX_IFACES];
    s32 total = 0;
    memset(ifaces, 0, sizeof(ifaces));

    Result rc = usbHsQueryAvailableInterfaces(&filter, ifaces, sizeof(ifaces), &total);
    if (R_FAILED(rc) || total <= 0) return MIC_NX_ERR_NODEV;

    /* Pick the interface for this player: player 0 → first, player 1 → second */
    int target = p->player;
    if (target >= total) {
        /* Only one physical device found; player 1 won't get a separate mic.
         * Return NODEV so the gadget layer can use silence for that channel. */
        return MIC_NX_ERR_NODEV;
    }

    /* Check VID/PID against known list (or accept any UAC AudioStreaming) */
    const char *dev_name = "Unknown UAC mic";
    (void)device_matches(&filter, &ifaces[target], &dev_name);

    int inum = open_audio_interface(&p->iface, &ifaces[target]);
    if (inum < 0) return MIC_NX_ERR_INIT;
    p->iface_open = 1;

    if (open_iso_ep(&p->ep, &p->iface) < 0) {
        usbHsIfClose(&p->iface);
        p->iface_open = 0;
        return MIC_NX_ERR_INIT;
    }
    p->ep_open = 1;

    return MIC_NX_OK;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

int mic_nx_init(void)
{
    mutexLock(&g_init_mutex);

    if (g_init_done) {
        mutexUnlock(&g_init_mutex);
        return MIC_NX_ERR_BUSY;
    }

    /* IMPORTANT: usbHs service ownership
     *
     * We do NOT call usbHsInitialize() here.  libusbhsfs (used by
     * SwitchStorage for USB mass-storage devices) already calls
     * usbHsInitialize() inside usbHsFsInitialize(), which runs before us
     * in main().  libnx reference-counts usbHs, so calling it twice is
     * safe in principle, but it is cleaner to share the single instance.
     *
     * Consequence: mic_nx_init() MUST be called after
     * switchStorageInitializeForPath() (which triggers usbHsFsInitialize).
     * The call site in main.c already guarantees this ordering.
     *
     * We also do NOT call usbHsExit() in mic_nx_exit() for the same reason:
     * libusbhsfs owns the lifecycle and calls usbHsExit() in usbHsFsExit().
     *
     * Our usbHsQueryAvailableInterfaces() filter targets bInterfaceClass=0x01
     * (USB Audio), which is entirely disjoint from libusbhsfs's MASS_STORAGE
     * filter, so the two subsystems never contend for the same interface.    */

    int any_ok = 0;

    for (int i = 0; i < MIC_NX_PLAYERS; i++) {
        MicPlayer *p = &g_players[i];
        memset(p, 0, sizeof(*p));
        p->player = i;
        p->vol    = 240;  /* matches PCSX2 default */
        ring_init(&p->ring);

        /* Allocate DMA-aligned transfer buffer (page-aligned for safety) */
        p->xfer_buf = (uint8_t *)aligned_alloc(0x1000,
                        (ISO_PACKET_SIZE + 0xFFFu) & ~0xFFFu);
        if (!p->xfer_buf) continue;
        memset(p->xfer_buf, 0, ISO_PACKET_SIZE);

        if (probe_player(p) != MIC_NX_OK) {
            /* Not an error — device may be plugged in later (hot-plug not yet
             * implemented; re-init on reconnect is handled by the gadget) */
            free(p->xfer_buf);
            p->xfer_buf = NULL;
            continue;
        }

        p->stop_flag = 0;
        if (pthread_create(&p->thread, NULL, capture_thread, p) != 0) {
            usbHsEpClose(&p->ep);
            usbHsIfClose(&p->iface);
            p->ep_open = p->iface_open = 0;
            free(p->xfer_buf);
            p->xfer_buf = NULL;
            continue;
        }

        p->active = 1;
        any_ok = 1;
    }

    g_init_done = 1;
    mutexUnlock(&g_init_mutex);
    return any_ok ? MIC_NX_OK : MIC_NX_ERR_NODEV;
}

void mic_nx_exit(void)
{
    mutexLock(&g_init_mutex);
    if (!g_init_done) {
        mutexUnlock(&g_init_mutex);
        return;
    }
    for (int i = 0; i < MIC_NX_PLAYERS; i++) {
        MicPlayer *p = &g_players[i];
        if (p->active) {
            p->stop_flag = 1;
            pthread_join(p->thread, NULL);
            p->active = 0;
        }
        if (p->ep_open)    { usbHsEpClose(&p->ep);    p->ep_open = 0;    }
        if (p->iface_open) { usbHsIfClose(&p->iface); p->iface_open = 0; }
        if (p->xfer_buf)   { free(p->xfer_buf); p->xfer_buf = NULL; }
    }
    g_init_done = 0;
    mutexUnlock(&g_init_mutex);
}

int mic_nx_connected(int player)
{
    if (player < 0 || player >= MIC_NX_PLAYERS) return 0;
    return g_players[player].active;
}

uint32_t mic_nx_read(int player, int16_t *dst, uint32_t frames)
{
    if (player < 0 || player >= MIC_NX_PLAYERS || !dst || !frames) return 0;
    MicPlayer *p = &g_players[player];
    if (!p->active) { memset(dst, 0, frames * MIC_NX_FRAME_BYTES); return 0; }
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
