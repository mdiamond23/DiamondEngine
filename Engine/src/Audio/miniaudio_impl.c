// The single translation unit that compiles the miniaudio implementation. Isolated
// here (like cgltf_impl.c) so the large implementation rarely recompiles, and so the
// dependency stays PRIVATE to the engine.
//
// OGG/Vorbis support is enabled by integrating stb_vorbis (shipped in miniaudio's
// extras/). The header-only declaration must come BEFORE the miniaudio implementation,
// and the stb_vorbis implementation AFTER it — this is the integration order miniaudio
// documents. WAV / MP3 / FLAC are built into miniaudio natively.

#define STB_VORBIS_HEADER_ONLY
#include "extras/stb_vorbis.c"

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

// stb_vorbis implementation must follow miniaudio's. Undefine the header-only guard so
// the second include emits the actual code.
#undef STB_VORBIS_HEADER_ONLY
#include "extras/stb_vorbis.c"
