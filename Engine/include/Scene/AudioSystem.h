#pragma once
#include "Scripting.h"

struct UIButtonFired;   // Scene/Events.h

// Drives the audio scene during play. Each frame it:
//   - syncs the first enabled AudioListenerComponent's world transform to the
//     single 3D listener (Audio::SetListener);
//   - starts each AudioSourceComponent's voice on play (playOnStart) and follows
//     its transform while the source is 3D, pushing volume/pitch edits live;
//   - reclaims a non-looping source voice once it has played to its end.
//
// It also wires audio into the event bus (Scene::Events()): it publishes
// SoundStarted / SoundFinished for source voices, and subscribes to UIButtonFired
// so a button carrying an AudioSourceComponent plays it as a one-shot (UI-bus
// integration), emitting UISoundPlayed.
//
// Runs late (after gameplay/physics, like ParticleSystem) so emitter/listener
// transforms are settled. All source voices are stopped on OnDestroy.
class AudioSystem : public GameSystem
{
public:
    void OnStart(Scene& scene) override;
    void OnUpdate(Scene& scene, float dt) override;
    void OnDestroy(Scene& scene) override;

    DECLARE_SYSTEM(AudioSystem, 600)

private:
    // Handler for UIButtonFired: plays the button's AudioSourceComponent (if any)
    // as a one-shot on its bus. Connected in OnStart, dropped when the scene clears
    // its dispatcher on StopPlay.
    void OnButtonFired(const UIButtonFired& ev);

    Scene* m_scene = nullptr;   // set in OnStart, used by OnButtonFired
};
