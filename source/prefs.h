/* prefs.h -- native key/value store backing the Android SharedPreferences the
 * core reads its settings from.
 *
 * libemucore.so reads every setting through PreferenceHelpers JNI callbacks
 * (getDefaultSharedPreferences + SharedPreferences.get{Boolean,Int,Float,String}
 * + Editor.put*). Keys are flat "Section/Key" strings (e.g. "EmuCore/GS/Renderer").
 * We back that with this in-memory map persisted to nethersx2.ini, seeded with
 * the defaults needed to boot the OpenGL renderer.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef __PREFS_H__
#define __PREFS_H__

#include <stdbool.h>

// load nethersx2.ini (if present), then seed any missing boot-critical defaults
// (Folders/*, EmuCore/GS/Renderer=OpenGL, UI/HasRunWizard, EmuCore/DiscPath).
void prefs_init(const char *ini_path);

// flush the current map back to the ini file
void prefs_save(void);

// lookups: return the stored value, or `def` on miss. `key` is the flat
// "Section/Key" string the SharedPreferences shim passes through.
bool        prefs_get_bool  (const char *key, bool def);
int         prefs_get_int   (const char *key, int def);
float       prefs_get_float (const char *key, float def);
const char *prefs_get_string(const char *key, const char *def);
bool        prefs_contains  (const char *key);

// setters (Editor.put* / Set*Value path). Values are stored as strings.
void prefs_set_bool  (const char *key, bool v);
void prefs_set_int   (const char *key, int v);
void prefs_set_float (const char *key, float v);
void prefs_set_string(const char *key, const char *v);
void prefs_remove    (const char *key);

// override the boot disc path (the launcher's EmuCore/DiscPath) before seeding
void prefs_set_disc_path(const char *path);

// prefs_seed_public -- like the internal prefs_seed() but callable from other
// modules (e.g. usb_singstar_nx.c).  Only writes the key if not already present
// in the ini (i.e. never clobbers a user's explicit choice).
void prefs_seed_public(const char *key, const char *val);

#endif
