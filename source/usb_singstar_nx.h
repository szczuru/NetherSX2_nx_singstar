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

int  usb_singstar_nx_init(void);
void usb_singstar_nx_exit(void);
int  usb_singstar_nx_connected(void);

int  usb_singstar_owns_stream(AAudioStream *s);

aaudio_result_t usb_singstar_openStream_hook(AAudioStreamBuilder *b,
                                             AAudioStream **out);
aaudio_result_t usb_singstar_requestStart_hook(AAudioStream *s);
aaudio_result_t usb_singstar_requestStop_hook(AAudioStream *s);
aaudio_result_t usb_singstar_close_hook(AAudioStream *s);
int64_t         usb_singstar_getFramesRead_hook(AAudioStream *s);

int32_t usb_singstar_read_hook(AAudioStream *s, void *buf, int32_t num_frames,
                               int64_t timeout_nanos);

#ifdef __cplusplus
}
#endif

#endif /* USB_SINGSTAR_NX_H */
