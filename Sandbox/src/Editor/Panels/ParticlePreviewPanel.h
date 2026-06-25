#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include "Scene/Components.h"
#include "Core/Camera.h"
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include <cstdint>

// Cascade/Shuriken-style preview viewport. Continuously plays the selected
// entity's ParticleEmitterComponent in an empty offscreen scene, independent of
// the editor's play state, so emitters can be authored and previewed live.
//
// The panel owns a working *copy* of the emitter (with its own runtime pool) so
// the simulation never touches the scene. By default it mirrors the selected
// emitter's config every frame — Inspector edits appear instantly. "Detach"
// stops mirroring and exposes the emitter inspector on the copy, with
// "Apply to Entity" to push the edited effect back onto the selection.
//
// main.cpp drives it: Tick() each frame, then renders Emitter()'s pool into the
// preview FBO using ViewMatrix()/ProjMatrix() and hands the texture to SetTexture().
class ParticlePreviewPanel : public Panel {
public:
    ParticlePreviewPanel();

    void SetContext(EditorContext* ctx) { m_Context = ctx; }
    void OnImGuiRender() override;

    // --- Driven by main.cpp, before OnImGuiRender ---
    void Tick(float dt);                       // resolve source, mirror config, step sim
    void SetTexture(uint32_t id) { m_TextureID = id; }

    // --- Read by main.cpp to render the preview into its FBO ---
    bool                            HasParticles() const { return m_HasSource; }
    const ParticleEmitterComponent& Emitter()     const { return m_Emitter; }
    glm::mat4 ViewMatrix() const { return m_Camera.GetViewMatrix(); }
    glm::mat4 ProjMatrix() const;
    glm::vec3 BackgroundColor() const { return m_BgColor; }
    glm::ivec2 DesiredSize() const { return m_DesiredSize; }   // content size in pixels

private:
    entt::entity ResolveSource() const;        // pinned, else selection with an emitter
    void LoadFromSource(const ParticleEmitterComponent& src);  // copy config + restart
    void MirrorConfig(const ParticleEmitterComponent& src);    // copy config, keep pool
    void Restart();                            // clear the pool (re-fires bursts)
    void ApplyToEntity(entt::entity target);   // push copy's config onto the entity (undoable)

    EditorContext*           m_Context = nullptr;
    ParticleEmitterComponent m_Emitter;        // working copy with its own pool
    Diamond::Camera          m_Camera;
    entt::entity             m_Source = entt::null;   // entity currently mirrored

    uint32_t   m_TextureID   = 0;
    glm::ivec2 m_DesiredSize { 1, 1 };
    float      m_Aspect      = 1.0f;
    glm::vec3  m_BgColor     { 0.10f, 0.10f, 0.12f };

    // Playback
    bool  m_Paused    = false;
    float m_TimeScale = 1.0f;
    bool  m_AutoLoop  = true;     // restart once the pool empties (re-fires one-shot bursts)

    // Source binding
    bool         m_Pinned       = false;
    entt::entity m_PinnedEntity = entt::null;
    bool         m_SyncFromSelection = true;   // false = detached editing of the copy
    bool         m_HasSource    = false;
};
