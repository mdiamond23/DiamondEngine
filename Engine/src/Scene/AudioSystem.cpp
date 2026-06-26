#include "Scene/AudioSystem.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Audio/AudioAPI.h"

#include <glm/glm.hpp>

namespace {

// Packs an AudioSourceComponent's configuration into the start-params struct.
Audio::SourceParams MakeParams(const AudioSourceComponent& s)
{
    Audio::SourceParams p;
    p.bus            = s.bus;
    p.volume         = s.volume;
    p.pitch          = s.pitch;
    p.loop           = s.loop;
    p.spatial        = s.is3D;
    p.stream         = s.stream;
    p.attenuation    = s.attenuation;
    p.minDistance    = s.minDistance;
    p.maxDistance    = s.maxDistance;
    p.rolloff        = s.rolloff;
    p.coneInnerAngle = s.coneInnerAngle;
    p.coneOuterAngle = s.coneOuterAngle;
    p.coneOuterGain  = s.coneOuterGain;
    return p;
}

} // namespace

void AudioSystem::OnStart(Scene& scene)
{
    // Fresh play session: clear any runtime voice state left in the components so
    // playOnStart fires cleanly. Voices themselves are spun up in OnUpdate.
    for (auto [e, s] : scene.View<AudioSourceComponent>().each()) {
        s._voice   = Audio::kInvalidVoice;
        s._started = false;
    }
}

void AudioSystem::OnUpdate(Scene& scene, float /*dt*/)
{
    auto& ts = scene.GetTransformSystem();

    // --- Listener: first enabled AudioListenerComponent drives the 3D listener.
    // Position from the world matrix, forward = -Z, up = +Y (same basis the camera
    // listener uses in main). One-frame-late transforms are fine here. ---
    for (auto [e, l] : scene.View<AudioListenerComponent>().each()) {
        if (!l.enabled) continue;
        const glm::mat4 w = ts.GetWorldMatrix(e);
        Audio::SetListener(glm::vec3(w[3]), -glm::vec3(w[2]), glm::vec3(w[1]));
        break;
    }

    // --- Sources: start on play, follow the transform, push live edits, reclaim. ---
    for (auto [e, s] : scene.View<AudioSourceComponent>().each()) {
        const glm::vec3 pos = glm::vec3(ts.GetWorldMatrix(e)[3]);

        if (!s._started && s.playOnStart && !s.clipPath.empty()) {
            s._started = true;
            const Audio::ClipHandle clip = Audio::Load(s.clipPath, s.stream);
            s._voice = Audio::PlaySource(clip, pos, MakeParams(s));
        }

        if (s._voice != Audio::kInvalidVoice) {
            if (!Audio::IsVoicePlaying(s._voice)) {
                // A non-looping source finished — release it and forget the handle.
                Audio::StopVoice(s._voice);
                s._voice = Audio::kInvalidVoice;
            } else {
                if (s.is3D) Audio::SetVoicePosition(s._voice, pos);
                Audio::SetVoiceVolume(s._voice, s.volume);
                Audio::SetVoicePitch (s._voice, s.pitch);
            }
        }
    }
}

void AudioSystem::OnDestroy(Scene& scene)
{
    // Stop every live source voice when play ends and reset runtime state.
    for (auto [e, s] : scene.View<AudioSourceComponent>().each()) {
        Audio::StopVoice(s._voice);
        s._voice   = Audio::kInvalidVoice;
        s._started = false;
    }
}
