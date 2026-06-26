#pragma once
#include <glm/glm.hpp>
#include <string>
#include "AudioTypes.h"

// The audio scripting facade. Free functions that forward to the active AudioEngine,
// so game scripts and the editor depend only on this header (no miniaudio, no engine
// internals). Every call is a silent no-op when no engine is active — same contract as
// the Physics:: API. See Docs/audio-system-design.md.
namespace Audio {

    // ---- Clips -------------------------------------------------------------
    // Loads (or references, if already cached) an audio file and returns a handle.
    // `stream` decodes on the fly from disk instead of fully into RAM — use it for
    // long music tracks; leave false for short SFX. Returns kInvalidClip on failure.
    ClipHandle Load(const std::string& path, bool stream = false);

    // Drops the engine's reference to a clip handle. In-flight voices keep playing.
    void Unload(ClipHandle clip);

    // ---- Playback ----------------------------------------------------------
    // Fire-and-forget spatial SFX at a world position. The voice is reclaimed
    // automatically when it finishes. No-op on an invalid clip or full-and-uncapped
    // voice pool. `bus` selects the mixer group; `pitch` is a playback-rate multiplier.
    void PlayOneShot(ClipHandle clip, const glm::vec3& position,
                     Bus bus = Bus::SFX, float volume = 1.0f, float pitch = 1.0f);

    // Fire-and-forget NON-spatial sound — UI clicks, music stingers, narration,
    // ambient one-shots. Position/attenuation/listener are ignored; plays at full
    // level. `bus` defaults to UI (the most common 2D case) but is overridable, e.g.
    // Bus::Music for a stinger or Bus::SFX for a non-positional effect.
    void Play2D(ClipHandle clip, Bus bus = Bus::UI, float volume = 1.0f, float pitch = 1.0f);

    // ---- Source voices (component-owned) -----------------------------------
    // Starts a persistent voice for an AudioSourceComponent and returns a handle to
    // drive it. Unlike PlayOneShot, this voice is NOT auto-reclaimed and is exempt
    // from the voice cap — the owner keeps it alive and must StopVoice() it (looping
    // sources play until then). Returns kInvalidVoice on an invalid clip or inactive
    // engine. `params.spatial` toggles 3D positioning; the rest configure the mixer
    // bus, level, looping, and the spatializer (attenuation / distance / cone).
    VoiceHandle PlaySource(ClipHandle clip, const glm::vec3& position, const SourceParams& params);

    // Stops and releases a source voice. A silent no-op on kInvalidVoice or a handle
    // that has already been reclaimed.
    void StopVoice(VoiceHandle voice);

    // Live edits to a playing source voice; no-ops on a stale handle. Lets the owning
    // component push transform/volume/pitch changes to an in-flight sound each frame.
    void SetVoicePosition(VoiceHandle voice, const glm::vec3& position);
    void SetVoiceVolume  (VoiceHandle voice, float volume);
    void SetVoicePitch   (VoiceHandle voice, float pitch);

    // True while the voice exists and has not finished. A non-looping voice that has
    // played to its end reports false, so the owner can clear its handle and release it.
    bool IsVoicePlaying(VoiceHandle voice);

    // ---- Listener ----------------------------------------------------------
    // Sets the single 3D listener (typically driven from the active camera). `forward`
    // and `up` need not be normalized. `velocity` feeds Doppler; pass 0 to disable it.
    void SetListener(const glm::vec3& position, const glm::vec3& forward,
                     const glm::vec3& up, const glm::vec3& velocity = glm::vec3(0.0f));

    // ---- Editor audition ---------------------------------------------------
    // Plays a single non-spatial PREVIEW voice for a file path, replacing any current
    // preview. Streamed (instant start/stop, flat memory) and routed past the buses at
    // unity gain — for content-browser / inspector auditioning in edit mode. Separate
    // from the fire-and-forget voice pool and exempt from the voice cap. Returns false
    // if the engine is inactive or the file won't open.
    bool PreviewClip(const std::string& path);
    void StopPreview();
    bool IsPreviewPlaying();

    // ---- Mixer -------------------------------------------------------------
    // Per-bus volume in linear gain (1 = unity). Master scales everything.
    void  SetBusVolume(Bus bus, float volume);
    float GetBusVolume(Bus bus);

    // Mute remembers and restores the pre-mute volume, so toggling is lossless.
    void  SetBusMuted(Bus bus, bool muted);
    bool  IsBusMuted(Bus bus);

    // ---- Diagnostics -------------------------------------------------------
    // Number of live fire-and-forget one-shot voices (the pool subject to the
    // voice cap). Excludes component-owned source voices and the preview voice.
    // Returns 0 when no engine is active.
    int ActiveVoiceCount();

    // The current ceiling on one-shot voices (kDefaultVoiceCap). 0 when inactive.
    int VoiceCap();

} // namespace Audio
