/* usb_singstar_nx.h - Emulated SingStar USB-mic device for NetherSX2
 *
 * Presents a virtual SingStar dual-mic adapter to the AetherSX2/PCSX2 USB
 * IOP emulation core by:
 *   1. Seeding USB/Port1 prefs so the core instantiates a SingStar device.
 *   2. Intercepting AAudio DIRECTION_INPUT streams so AudioDevice::CreateDevice
 *      succeeds and GetBuffer / data-callback paths receive real USB PCM.
 *
 * Integration points in the NetherSX2 wrapper:
 *   - main.c   -> call usb_singstar_nx_init() after storage + prefs are up,
 *                 and usb_singstar_nx_exit() on shutdown.
 *   - aaudio.c -> weak hooks for openStream / start / stop / close /
 *                 getFramesRead / read (INPUT path).
 *
 * MIT License - see LICENSE in the repository root.
 */
 
#ifndef USB_SINGSTAR_NX_H
#define USB_SINGSTAR_NX_H
 
#include <stdint.h>
#include "aaudio.h"
 
#ifdef __cplusplus
extern "C" {
#endif
 
/**
 * usb_singstar_nx_init - initialise the emulated SingStar device and start
 * the physical USB capture (calls mic_nx_init internally).
 *
 * Returns 0 if at least one physical mic was found, 1 if not.
 * The emulated SingStar device is always registered on PS2 USB Port 1
 * (via prefs), so the game should see the adapter either way; without a
 * physical mic it simply produces silence.
 */
int  usb_singstar_nx_init(void);
 
/**
 * usb_singstar_nx_exit - tear down the emulated device and physical capture.
 */
void usb_singstar_nx_exit(void);
 
/**
 * usb_singstar_nx_connected - returns 1 if at least one physical mic is live.
 */
int  usb_singstar_nx_connected(void);
int  usb_singstar_open_count(void);
int  usb_singstar_start_count(void);
 
/* ---- hooks called from aaudio.c (must be non-static, non-weak here) ---- */
 
int  usb_singstar_owns_stream(AAudioStream *s);
 
aaudio_result_t usb_singstar_openStream_hook(AAudioStreamBuilder *b,
                                             AAudioStream **out);
aaudio_result_t usb_singstar_requestStart_hook(AAudioStream *s);
aaudio_result_t usb_singstar_requestStop_hook(AAudioStream *s);
aaudio_result_t usb_singstar_close_hook(AAudioStream *s);
int64_t         usb_singstar_getFramesRead_hook(AAudioStream *s);
 
/**
 * Blocking/non-blocking read of captured PCM into the caller's buffer.
 * Used if the core's AudioDevice path calls AAudioStream_read.
 * Returns number of frames written, or a negative AAudio error code.
 */
int32_t usb_singstar_read_hook(AAudioStream *s, void *buf, int32_t num_frames,
                               int64_t timeout_nanos);
 
#ifdef __cplusplus
}
#endif
 
#endif /* USB_SINGSTAR_NX_H */
