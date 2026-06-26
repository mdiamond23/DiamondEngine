#pragma once
#include "Scripting.h"

// Drives the audio scene during play. Each frame it:
//   - syncs the first enabled AudioListenerComponent's world transform to the
//     single 3D listener (Audio::SetListener);
//   - starts each AudioSourceComponent's voice on play (playOnStart) and follows
//     its transform while the source is 3D, pushing volume/pitch edits live;
//   - reclaims a non-looping source voice once it has played to its end.
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
};
