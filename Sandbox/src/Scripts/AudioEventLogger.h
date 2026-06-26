#pragma once
// Test/diagnostic script for the audio event bus (Phase 5). Subscribes to the
// audio events on Scene::Events() and logs each one, so you can confirm they fire:
//   - SoundStarted  / SoundFinished — component-owned source voices begin / end.
//   - UISoundPlayed — a fired UI button carried an AudioSourceComponent and clicked.
//
// To see SoundStarted/SoundFinished: give an entity an AudioSourceComponent with a
// short, non-looping clip and Play On Start, then enter play mode and watch the
// console. To see UISoundPlayed: put an AudioSourceComponent (playOnStart OFF) on a
// UI Button and click it.
//
// No component of its own — a global system. Subscriptions are made in OnStart and
// dropped automatically when StopPlay clears the dispatcher.
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/Events.h"
#include <spdlog/spdlog.h>

class AudioEventLoggerSystem : public GameSystem
{
    DECLARE_SYSTEM(AudioEventLoggerSystem, 700)   // after AudioSystem (600)
public:
    void OnStart(Scene& scene) override
    {
        auto& bus = scene.Events();
        bus.sink<SoundStarted>() .connect<&AudioEventLoggerSystem::OnSoundStarted>(*this);
        bus.sink<SoundFinished>().connect<&AudioEventLoggerSystem::OnSoundFinished>(*this);
        bus.sink<UISoundPlayed>().connect<&AudioEventLoggerSystem::OnUISoundPlayed>(*this);
    }

private:
    void OnSoundStarted (const SoundStarted&  e) { spdlog::info("[AudioEvent] SoundStarted  source={}", (uint32_t)e.source); }
    void OnSoundFinished(const SoundFinished& e) { spdlog::info("[AudioEvent] SoundFinished source={}", (uint32_t)e.source); }
    void OnUISoundPlayed(const UISoundPlayed& e) { spdlog::info("[AudioEvent] UISoundPlayed button={}", (uint32_t)e.button); }
};
