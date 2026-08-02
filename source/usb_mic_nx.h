 * started automatically so mics plugged in later are picked up.
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
 * mic_nx_any_connected — non-zero if at least one player mic is live.
 */
int  mic_nx_any_connected(void);
 
/**
 * mic_nx_read — drain up to `frames` 16-bit mono samples into `dst`.
 *
 * Returns the number of frames actually written (may be < frames if the
 * ring buffer is sparse).  The caller zero-pads the rest.
 *
 * Thread-safe: may be called from the emulator's USB service thread while
 * the capture thread is writing.
 */
uint32_t mic_nx_read(int player, int16_t *dst, uint32_t frames);
 
/**
 * mic_nx_frames_available — how many frames are ready in the ring buffer.
 */
uint32_t mic_nx_frames_available(int player);
 
/**
 * mic_nx_set_volume — software gain [0..255] applied before the ring write.
 * 255 = unity gain.  Default: 240 (matches PCSX2 SingStar default).
 */
void mic_nx_set_volume(int player, uint8_t vol);
 
/**
 * mic_nx_poll — optional foreground poll (e.g. from main loop).
 * The background hotplug thread already covers this; calling poll is cheap
 * and safe if you want immediate reaction after docking.
 */
void mic_nx_poll(void);
 
/**
 * Optional OSD callback.  Set from main.c so we can show "Mic detected"
 * notifications without depending on the full emulator JNI surface.
 * signature: void cb(const char *key, const char *msg, float duration_sec)
 */
typedef void (*mic_nx_osd_fn)(const char *key, const char *msg, float duration);
void mic_nx_set_osd_callback(mic_nx_osd_fn fn);
 
#ifdef __cplusplus
}
#endif
#endif /* USB_MIC_NX_H */
