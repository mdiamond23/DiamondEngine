#pragma once
#include <cstdint>

// Public audio vocabulary — no miniaudio types leak through here, so this header is
// safe to include anywhere (scripts, editor, components). See Docs/audio-system-design.md.
namespace Audio {

    // Fixed mixer buses. Master is the engine endpoint; the rest are sound groups
    // parented to it. Sources route to one bus; one-shots default to SFX, PlayUI to UI.
    enum class Bus { Master, Music, SFX, UI, Ambience };

    // Number of routable sub-buses (everything except Master).
    inline constexpr int kBusGroupCount = 4;

    // Opaque reference to a loaded clip (index into the engine's clip table). Compare
    // against kInvalidClip to detect a failed load.
    using ClipHandle = uint32_t;
    inline constexpr ClipHandle kInvalidClip = 0xFFFFFFFFu;

    // Default ceiling on simultaneously playing voices; the oldest is stolen when hit.
    inline constexpr int kDefaultVoiceCap = 32;

} // namespace Audio
