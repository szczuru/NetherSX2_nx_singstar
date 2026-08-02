/* usb_singstar_nx.h — Emulated SingStar USB-mic device for NetherSX2
 *
 * Presents a virtual SingStar dual-mic adapter to the AetherSX2/PCSX2 USB
 * IOP emulation core.  The core calls usb_singstar_handle_packet() on every
 * USB frame (1 ms); we drain PCM from the real Switch mic (usb_mic_nx.c)
 * and return it in the format the PS2 game expects.
 *
 * Integration points in the NetherSX2 wrapper:
 *   • main.c  →  call usb_singstar_nx_init() after the emulator core is up
 *               and call usb_singstar_nx_exit() on shutdown.
 *   • prefs.c →  seed "USB/Port1/Type = singstar" so the core's USB stack
 *               routes Port 1 to this plugin.
 *
 * MIT License — see LICENSE in the repository root.
 */

#ifndef USB_SINGSTAR_NX_H
#define USB_SINGSTAR_NX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * usb_singstar_nx_init — initialise the emulated SingStar device and start
 * the physical USB capture (calls mic_nx_init internally).
 *
 * Returns 0 on success.  A non-zero return means no physical mic was found;
 * the device still appears to the PS2 game but produces silence.
 */
int  usb_singstar_nx_init(void);

/**
 * usb_singstar_nx_exit — tear down the emulated device and physical capture.
 */
void usb_singstar_nx_exit(void);

/**
 * usb_singstar_nx_connected — returns 1 if at least one physical mic is live.
 */
int  usb_singstar_nx_connected(void);

#ifdef __cplusplus
}
#endif
#endif /* USB_SINGSTAR_NX_H */
