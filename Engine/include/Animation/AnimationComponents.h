#pragma once

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <glm/glm.hpp>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/Frustum.h"
#include "Animation/Skeleton.h"
#include "Animation/AnimationClip.h"
#include "Animation/AnimationStateMachine.h"

// Skinned-character components. Kept in the global namespace to match the other
// ECS components in Scene/Components.h. Populated from GltfImporter::LoadModel.

// Render data for a skinned model: GPU meshes (one per glTF primitive) sharing a
// skeleton, plus its animation clips. The animated pose lives on AnimatorComponent.
struct SkinnedMeshComponent {
    std::vector<std::shared_ptr<Diamond::Mesh>> meshes;
    std::shared_ptr<Diamond::PBRMaterial>       material;
    Diamond::Skeleton                           skeleton;
    std::vector<Diamond::AnimationClip>         clips;
    Diamond::AABB                               localBounds;

    bool visible     = true;
    bool castsShadow = true;
    std::string meshPath;       // for future serialization
};

// Playback state + the per-frame bone-matrix palette produced by the animation
// system. `palette` is uploaded to the skinning shader as `uBones`.
struct AnimatorComponent {
    int   clip    = 0;       // index into SkinnedMeshComponent::clips (-1 = bind pose)
    float time    = 0.0f;    // seconds
    float speed   = 1.0f;
    bool  loop    = true;
    bool  playing = true;

    // Cross-fade state (transient — rebuilt at runtime, never serialized). While a
    // fade is active the outgoing clip keeps advancing as `prevClip`/`prevTime`,
    // and the animation system blends it toward the current clip by
    // fadeElapsed/fadeDuration. prevClip < 0 means no fade in progress.
    int   prevClip     = -1;
    float prevTime     = 0.0f;
    float fadeDuration = 0.0f;
    float fadeElapsed  = 0.0f;

    std::vector<glm::mat4> palette;

    // Begin a cross-fade to `newClip` over `fade` seconds. The current clip keeps
    // playing as the outgoing pose; the new clip starts at time 0. fade <= 0 (or
    // switching to the same clip) snaps instantly. Does not change `playing`.
    void CrossFade(int newClip, float fade) {
        if (newClip == clip) return;
        if (fade > 0.0f) {
            prevClip     = clip;
            prevTime     = time;
            fadeDuration = fade;
            fadeElapsed  = 0.0f;
        } else {
            prevClip     = -1;
            fadeDuration = 0.0f;
        }
        clip = newClip;
        time = 0.0f;
    }
};

// High-level animation controller: drives a sibling AnimatorComponent from a
// shared, reusable state machine (the .animsm asset). The asset is the static
// graph; this component holds the per-entity instance state — the live parameter
// values gameplay sets and which state we're currently in. UpdateStateMachines
// evaluates transitions and issues AnimatorComponent::CrossFade calls; without
// this component an entity just plays its AnimatorComponent clip directly.
struct AnimStateMachineComponent {
    std::shared_ptr<Diamond::AnimStateMachine> machine;     // the asset (may be null)
    std::string                                assetPath;   // for serialization

    // Live parameter values. Triggers are one-shot: set by SetTrigger, persisted
    // until a transition that tests them fires (then consumed). Floats/bools persist.
    std::unordered_map<std::string, float> floats;
    std::unordered_map<std::string, bool>  bools;
    std::unordered_set<std::string>        triggers;

    int currentState = -1;   // -1 = uninitialized; the runtime enters entryState

    void  SetFloat(const std::string& n, float v) { floats[n] = v; }
    void  SetBool (const std::string& n, bool  v) { bools[n]  = v; }
    void  SetTrigger(const std::string& n)        { triggers.insert(n); }
    float GetFloat(const std::string& n) const { auto it = floats.find(n); return it == floats.end() ? 0.0f  : it->second; }
    bool  GetBool (const std::string& n) const { auto it = bools.find(n);  return it == bools.end()  ? false : it->second; }

    // Seed any missing live values from the asset's parameter defaults. Existing
    // values (e.g. loaded from a scene) are kept. Call after assigning `machine`.
    void SyncParams() {
        if (!machine) return;
        for (const auto& p : machine->parameters) {
            if (p.type == Diamond::AnimParamType::Float) floats.emplace(p.name, p.defaultFloat);
            else if (p.type == Diamond::AnimParamType::Bool) bools.emplace(p.name, p.defaultBool);
        }
    }
};
