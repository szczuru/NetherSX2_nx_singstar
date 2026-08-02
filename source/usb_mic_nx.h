/* usb_mic_nx.h - UAC host driver for SingStar microphones on Nintendo Switch
 *
 * Opens real USB Audio Class devices (SingStar VID 0x1415, or any UAC mic)
 * attached to the Switch dock's USB ports, drives isochronous IN transfers,
 * and exposes a simple ring-buffer API that the emulated PS2 USB-mic device
 * (usb_singstar_nx.c) can drain on each USB_TOKEN_IN packet.
 *
 * Two independent instances are supported (Player 1 and Player 2) matching
 * the SingStar dual-mic adapter layout.
 *
 * MIT License - see LICENSE in the repository root.
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
/* 4096 frames ~ 85 ms at 48 kHz; enough headroom for 1-ms USB intervals.  */
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
