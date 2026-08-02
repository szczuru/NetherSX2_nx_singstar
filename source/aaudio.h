};
enum {
  AAUDIO_STREAM_STATE_UNINITIALIZED = 0,
  AAUDIO_STREAM_STATE_UNKNOWN = 1,
  AAUDIO_STREAM_STATE_OPEN = 2,
  AAUDIO_STREAM_STATE_STARTING = 3,
  AAUDIO_STREAM_STATE_STARTED = 4,
  AAUDIO_STREAM_STATE_PAUSING = 5,
  AAUDIO_STREAM_STATE_PAUSED = 6,
  AAUDIO_STREAM_STATE_FLUSHING = 7,
  AAUDIO_STREAM_STATE_FLUSHED = 8,
  AAUDIO_STREAM_STATE_STOPPING = 9,
  AAUDIO_STREAM_STATE_STOPPED = 10,
  AAUDIO_STREAM_STATE_CLOSING = 11,
  AAUDIO_STREAM_STATE_CLOSED = 12,
  AAUDIO_STREAM_STATE_DISCONNECTED = 13,
};
enum { AAUDIO_CALLBACK_RESULT_CONTINUE = 0, AAUDIO_CALLBACK_RESULT_STOP = 1 };
 
typedef struct AAudioStreamStruct AAudioStream;
typedef struct AAudioStreamBuilderStruct AAudioStreamBuilder;
 
typedef aaudio_data_callback_result_t (*AAudioStream_dataCallback)(
    AAudioStream *stream, void *userData, void *audioData, int32_t numFrames);
typedef void (*AAudioStream_errorCallback)(
    AAudioStream *stream, void *userData, aaudio_result_t error);
 
/* ---- builder ---- */
aaudio_result_t AAudio_createStreamBuilder(AAudioStreamBuilder **builder);
void AAudioStreamBuilder_setDeviceId(AAudioStreamBuilder *b, int32_t deviceId);
void AAudioStreamBuilder_setDirection(AAudioStreamBuilder *b, aaudio_direction_t d);
void AAudioStreamBuilder_setSharingMode(AAudioStreamBuilder *b, aaudio_sharing_mode_t m);
void AAudioStreamBuilder_setSampleRate(AAudioStreamBuilder *b, int32_t sampleRate);
void AAudioStreamBuilder_setChannelCount(AAudioStreamBuilder *b, int32_t channelCount);
void AAudioStreamBuilder_setFormat(AAudioStreamBuilder *b, aaudio_format_t format);
void AAudioStreamBuilder_setBufferCapacityInFrames(AAudioStreamBuilder *b, int32_t frames);
void AAudioStreamBuilder_setPerformanceMode(AAudioStreamBuilder *b, aaudio_performance_mode_t m);
void AAudioStreamBuilder_setFramesPerDataCallback(AAudioStreamBuilder *b, int32_t frames);
void AAudioStreamBuilder_setDataCallback(AAudioStreamBuilder *b, AAudioStream_dataCallback cb, void *user);
void AAudioStreamBuilder_setErrorCallback(AAudioStreamBuilder *b, AAudioStream_errorCallback cb, void *user);
aaudio_result_t AAudioStreamBuilder_openStream(AAudioStreamBuilder *b, AAudioStream **stream);
aaudio_result_t AAudioStreamBuilder_delete(AAudioStreamBuilder *b);
 
/* ---- stream ---- */
aaudio_result_t AAudioStream_requestStart(AAudioStream *s);
aaudio_result_t AAudioStream_requestPause(AAudioStream *s);
aaudio_result_t AAudioStream_requestStop(AAudioStream *s);
aaudio_result_t AAudioStream_close(AAudioStream *s);
int64_t AAudioStream_getFramesRead(AAudioStream *s);
aaudio_result_t AAudioStream_waitForStateChange(AAudioStream *s, aaudio_stream_state_t inputState,
                                                aaudio_stream_state_t *nextState, int64_t timeoutNanoseconds);
 
/* AAudioStream_read - pull captured PCM from an INPUT stream.
 * Not used by the stock AAudioMod OUTPUT path, but the USBMic / AudioDevice
 * layer may call it when data callbacks are not registered.
 * Returns frames written (>=0) or a negative aaudio_result_t. */
int32_t AAudioStream_read(AAudioStream *s, void *buffer, int32_t numFrames,
                          int64_t timeoutNanoseconds);
 
#endif /* __AAUDIO_H__ */
