/* usb_mic_nx.h — UAC host driver for SingStar microphones on Nintendo Switch
 *
 * Opens real USB Audio Class devices (SingStar VID 0x1415, or any UAC mic)
 * attached to the Switch dock's USB ports, drives isochronous IN transfers,
 * and exposes a simple ring-buffer API that the emulated PS2 USB-mic device
 * (usb_singstar_nx.c) can drain on each USB_TOKEN_IN packet.
 *
 * Two independent instances are supported (Player 1 and Player 2) matching
 * the SingStar dual-mic adapter layout.
 *
 * MIT License — see LICENSE in the repository root.
 */

#ifndef USB_MIC_NX_H
#define USB_MIC_NX_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- result codes -------------------------------------------------------- */
#define MIC_NX_OK          0
#define MIC_NX_ERR_NODEV  -1   /* device not present / enumeration failed   */
#define MIC_NX_ERR_INIT   -2   /* UsbHs / EP open failed                    */
#define MIC_NX_ERR_PARAM  -3   /* bad argument                              */
#define MIC_NX_ERR_BUSY   -4   /* already initialised                       */

/* ---- PCM parameters the Switch UAC driver uses internally --------------- */
/* The SingStar adapter negotiates 48 kHz mono 16-bit (UAC alt-setting 1).  */
/* The emulated device core then resamples if the PS2 game requests 8/11 kHz */
#define MIC_NX_SAMPLE_RATE   48000
#define MIC_NX_CHANNELS      1
#define MIC_NX_BITS          16
#define MIC_NX_FRAME_BYTES   (MIC_NX_CHANNELS * (MIC_NX_BITS / 8))

/* ---- ring-buffer capacity (frames) -------------------------------------- */
/* 480 frames = 10 ms at 48 kHz; plenty for the 1-ms USB service interval.  */
#define MIC_NX_RING_FRAMES   4096

/* ---- known USB IDs ------------------------------------------------------- */
/* Original SingStar dual-mic adapter (Nam Tai E&E) */
#define SINGSTAR_VID   0x1415u
#define SINGSTAR_PID   0x0000u

/* Logitech USB mic */
#define LOGITECH_VID   0x046Du

/* Konami (AK5370) */
#define KONAMI_VID     0x0556u
#define KONAMI_PID     0x0001u

/* ---- player index -------------------------------------------------------- */
#define MIC_NX_PLAYER1  0
#define MIC_NX_PLAYER2  1
#define MIC_NX_PLAYERS  2

/* ---- public API ---------------------------------------------------------- */

/**
 * mic_nx_init — enumerate USB audio devices and start capture threads.
 *
 * Call once at emulator start (after UsbHs has been initialised by the
 * SwitchStorage layer).  Returns MIC_NX_OK even when no mic is connected;
 * capture simply produces silence in that case.
 */
int  mic_nx_init(void);

/**
 * mic_nx_exit — stop capture threads, close USB endpoints, free memory.
 * Safe to call even if mic_nx_init was never called or failed.
 */
void mic_nx_exit(void);

/**
 * mic_nx_connected — non-zero if the given player's mic is open and running.
 */
int  mic_nx_connected(int player);

/**
 * mic_nx_read — drain up to `frames` 16-bit mono samples into `dst`.
 *
 * Returns the number of frames actually written (may be < frames if the
 * ring buffer is sparse).  The caller (usb_singstar_nx.c) zero-pads the rest.
 *
 * Thread-safe: may be called from the emulator's USB service thread while
 * the capture thread is writing.
 */
uint32_t mic_nx_read(int player, int16_t *dst, uint32_t frames);

/**
 * mic_nx_frames_available — how many frames are ready in the ring buffer.
 * Used by the PS2 USB gadget to decide whether to NAK or provide data.
 */
uint32_t mic_nx_frames_available(int player);

/**
 * mic_nx_set_volume — software gain [0..255] applied before the ring write.
 * 255 = unity gain.  Default: 240 (matches PCSX2 SingStar default).
 */
void mic_nx_set_volume(int player, uint8_t vol);

#ifdef __cplusplus
}
#endif
#endif /* USB_MIC_NX_H */
