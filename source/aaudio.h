/* aaudio.h -- minimal NDK AAudio ABI shim, backed by libnx audren.
 *
 * libemucore.so (DT_NEEDED libaaudio) drives audio through the AetherSX2
 * "AAudioMod" pull-callback backend: it registers a data callback and the audio
 * device thread calls it to FILL interleaved stereo frames. We implement just
 * the ~19 symbols the core imports over one audren FLOAT stereo voice @ 48000.
 *
 * INPUT streams (SingStar / USB mic) are delegated to usb_singstar_nx.c.
 *
 * The enum values + struct layouts must match the NDK <aaudio/AAudio.h> ABI the
 * core was compiled against.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
 
#ifndef __AAUDIO_H__
#define __AAUDIO_H__
 
#include <stdint.h>
 
typedef int32_t aaudio_result_t;
typedef int32_t aaudio_format_t;
typedef int32_t aaudio_direction_t;
typedef int32_t aaudio_sharing_mode_t;
typedef int32_t aaudio_performance_mode_t;
typedef int32_t aaudio_stream_state_t;
typedef int32_t aaudio_data_callback_result_t;
 
enum {
  AAUDIO_OK = 0,
  AAUDIO_ERROR_BASE = -900,
  AAUDIO_ERROR_DISCONNECTED = -899,
  AAUDIO_ERROR_INVALID_STATE = -895,
  AAUDIO_ERROR_INVALID_HANDLE = -892,
  AAUDIO_ERROR_UNAVAILABLE = -878,
  AAUDIO_ERROR_NO_FREE_HANDLES = -877,
  AAUDIO_ERROR_NO_MEMORY = -876,
  AAUDIO_ERROR_NULL = -875,
  AAUDIO_ERROR_TIMEOUT = -874,
};
 
enum {
  AAUDIO_FORMAT_INVALID = -1,
  AAUDIO_FORMAT_UNSPECIFIED = 0,
  AAUDIO_FORMAT_PCM_I16 = 1,
  AAUDIO_FORMAT_PCM_FLOAT = 2,
};
 
enum { AAUDIO_DIRECTION_OUTPUT = 0, AAUDIO_DIRECTION_INPUT = 1 };
enum { AAUDIO_SHARING_MODE_EXCLUSIVE = 0, AAUDIO_SHARING_MODE_SHARED = 1 };
enum {
  AAUDIO_PERFORMANCE_MODE_NONE = 10,
  AAUDIO_PERFORMANCE_MODE_POWER_SAVING = 11,
  AAUDIO_PERFORMANCE_MODE_LOW_LATENCY = 12,
};
enum {
  AAUDIO_STREAM_STATE_UNINITIALIZED = 0,
  AAUDIO_STREAM_STATE_UNKNOWN = 1,
  AAUDIO_STREAM_STATE_OPEN = 2,
  AAUDIO_STREAM_STATE_STARTING = 3,
  AAUDIO_STREAM_STATE_STARTED = 4,
