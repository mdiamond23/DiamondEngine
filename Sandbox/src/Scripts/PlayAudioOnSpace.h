#pragma once
// Simple test script: press SPACE (in play mode, with the game viewport focused)
// to fire every AudioSourceComponent in the scene as a one-shot, using each
// source's clip, bus, volume, pitch and 3D flag. This is a no-component global
// system — just drop a clip onto an AudioSource, enter play mode, focus the game
// viewport, and tap Space to hear it. Independent of the source's playOnStart /
// looping voice that AudioSystem owns, so it won't fight that lifecycle.
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Core/Input.h"
#include "Audio/AudioAPI.h"
#include <glm/glm.hpp>

class PlayAudioOnSpaceSystem : public GameSystem
{
    DECLARE_SYSTEM(PlayAudioOnSpaceSystem, 150)
public:
    void OnUpdate(Scene& scene, float /*dt*/) override
    {
        // Edge-triggered: fires once per key press, not while held.
        if (!Input::IsKeyPressed(Key::Space)) return;

        auto& ts = scene.GetTransformSystem();
        for (auto [e, src] : scene.View<AudioSourceComponent>().each())
        {
            if (src.clipPath.empty()) continue;

            const Audio::ClipHandle clip = Audio::Load(src.clipPath, src.stream);
            if (clip == Audio::kInvalidClip) continue;

            if (src.is3D) {
                const glm::vec3 pos = glm::vec3(ts.GetWorldMatrix(e)[3]);
                Audio::PlayOneShot(clip, pos, src.bus, src.volume, src.pitch);
            } else {
                Audio::Play2D(clip, src.bus, src.volume, src.pitch);
            }
        }
    }
};
