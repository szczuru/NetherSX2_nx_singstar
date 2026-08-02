/* usb_mic_nx.h - UAC host driver for SingStar mics on Switch */
#ifndef USB_MIC_NX_H
#define USB_MIC_NX_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MIC_NX_OK          0
#define MIC_NX_ERR_NODEV  -1
#define MIC_NX_ERR_INIT   -2
#define MIC_NX_ERR_PARAM  -3
#define MIC_NX_ERR_BUSY   -4

#define MIC_NX_SAMPLE_RATE   48000
#define MIC_NX_CHANNELS      1
#define MIC_NX_BITS          16
#define MIC_NX_FRAME_BYTES   2
#define MIC_NX_RING_FRAMES   4096

#define SINGSTAR_VID   0x1415u
#define SINGSTAR_PID   0x0000u
#define LOGITECH_VID   0x046Du
#define KONAMI_VID     0x0556u
#define KONAMI_PID     0x0001u

#define MIC_NX_PLAYER1  0
#define MIC_NX_PLAYER2  1
#define MIC_NX_PLAYERS  2

int  mic_nx_init(void);
void mic_nx_exit(void);
int  mic_nx_connected(int player);
int  mic_nx_any_connected(void);
uint32_t mic_nx_read(int player, int16_t *dst, uint32_t frames);
uint32_t mic_nx_frames_available(int player);
void mic_nx_set_volume(int player, uint8_t vol);
void mic_nx_poll(void);

typedef void (*mic_nx_osd_fn)(const char *key, const char *msg, float duration);
void mic_nx_set_osd_callback(mic_nx_osd_fn fn);
const char *mic_nx_last_diag(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_MIC_NX_H */
