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

    // 3D distance-attenuation curves, mirroring miniaudio's models. Inverse is the
    // physically natural default; None disables distance falloff entirely.
    enum class AttenuationModel { None, Inverse, Linear, Exponential };

    // Opaque reference to a persistent, component-owned voice — an
    // AudioSourceComponent's playing sound, as opposed to a fire-and-forget one-shot.
    // The owner keeps it alive and must release it; 0 means "no voice".
    using VoiceHandle = uint64_t;
    inline constexpr VoiceHandle kInvalidVoice = 0;

    // Bundle of settings for starting a component-owned source voice: routing, level,
    // looping, and the full 3D spatializer configuration. Cone angles are in degrees;
    // 360 (omnidirectional) disables directional attenuation.
    struct SourceParams {
        Bus   bus     = Bus::SFX;
        float volume  = 1.0f;
        float pitch   = 1.0f;
        bool  loop    = false;
        bool  spatial = true;          // 3D positioned vs. flat 2D
        bool  stream  = false;         // long tracks: decode from disk on the fly
        AttenuationModel attenuation = AttenuationModel::Inverse;
        float minDistance    = 1.0f;   // full volume within this radius
        float maxDistance    = 100.0f;
        float rolloff        = 1.0f;
        float coneInnerAngle = 360.0f; // degrees; within = full gain
        float coneOuterAngle = 360.0f; // degrees; beyond = coneOuterGain
        float coneOuterGain  = 0.0f;
    };

} // namespace Audio
