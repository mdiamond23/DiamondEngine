#pragma once
#include <memory>
#include <string>
#include "AudioTypes.h"

// AudioEngine owns the miniaudio device, the bus groups, the clip table, and the
// active-voice pool. All miniaudio state lives behind the PIMPL so this header pulls
// in no miniaudio types — keeping the dependency PRIVATE, exactly like Jolt.
//
// The application creates ONE instance and ticks it each frame. The free-function
// facade in AudioAPI.h forwards to whichever instance is currently active, so game
// scripts never touch this class directly. Mirrors the Physics session model: every
// Audio:: call is a silent no-op when no engine is active.
class AudioEngine
{
public:
    AudioEngine();
    ~AudioEngine();

    AudioEngine(const AudioEngine&)            = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    // Brings up the playback device and bus graph, and registers this instance as the
    // active target for the Audio:: facade. Returns false if the device failed to open
    // (the rest of the engine keeps running silently). Safe to call once.
    bool Init();

    // Stops every voice, tears down the device, and clears the active target. Safe to
    // call without a successful Init.
    void Shutdown();

    // Per-frame housekeeping: reclaims finished one-shot voices and enforces the voice
    // cap. Cheap; call once per frame regardless of play state.
    void Update();

    bool IsActive() const;

private:
    friend class AudioFacadeAccess;   // lets the Audio:: facade reach the impl
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};
