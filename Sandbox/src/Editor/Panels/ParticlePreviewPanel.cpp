#include "ParticlePreviewPanel.h"
#include "../Command.h"
#include "Scene/Scene.h"
#include "Scene/ParticleSim.h"
// Reuses the registered emitter inspector (DrawComponentInspector specialization)
// for detached editing of the working copy. The DECLARE_COMPONENT it carries is an
// inline variable, so including it here doesn't double-register.
#include "Scripts/ParticleEmitterEditor.h"

#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <memory>
#include <utility>

ParticlePreviewPanel::ParticlePreviewPanel()
    : m_Camera(glm::vec3(0.0f, 1.0f, 6.0f))
{
    // Frame the origin from slightly above, looking down the -Z axis. Particles
    // emit along +Y by default, so this stages a fountain nicely.
    m_Camera.Pivot = m_Camera.TargetPivot = glm::vec3(0.0f, 1.0f, 0.0f);
    m_Camera.Distance = m_Camera.TargetDistance = 6.0f;
    m_Camera.Yaw   = m_Camera.TargetYaw   = -90.0f;
    m_Camera.Pitch = m_Camera.TargetPitch = -10.0f;
    m_Camera.UpdateOrbit(1.0f);   // snap Position onto the orbit sphere
}

glm::mat4 ParticlePreviewPanel::ProjMatrix() const
{
    return glm::perspective(glm::radians(m_Camera.Zoom), m_Aspect, 0.1f, 100.0f);
}

entt::entity ParticlePreviewPanel::ResolveSource() const
{
    if (!m_Context || !m_Context->ActiveScene) return entt::null;
    auto& reg = m_Context->ActiveScene->GetRegistry();
    auto hasEmitter = [&](entt::entity e) {
        return e != entt::null && reg.valid(e) && reg.all_of<ParticleEmitterComponent>(e);
    };
    if (m_Pinned && hasEmitter(m_PinnedEntity)) return m_PinnedEntity;
    entt::entity sel = m_Context->PrimarySelection();
    return hasEmitter(sel) ? sel : entt::null;
}

void ParticlePreviewPanel::LoadFromSource(const ParticleEmitterComponent& src)
{
    m_Emitter = src;                 // config + texture (and src's empty runtime pool)
    ParticleSim::Reset(m_Emitter);   // start a fresh session
}

void ParticlePreviewPanel::MirrorConfig(const ParticleEmitterComponent& src)
{
    // Pull the config from the live component but keep our own running pool so the
    // preview keeps animating across Inspector edits.
    auto        pool = std::move(m_Emitter.pool);
    std::size_t live = m_Emitter.liveCount;
    float       acc  = m_Emitter.spawnAccumulator;
    float       bt   = m_Emitter.burstTimer;
    m_Emitter = src;
    m_Emitter.pool             = std::move(pool);
    m_Emitter.liveCount        = live;
    m_Emitter.spawnAccumulator = acc;
    m_Emitter.burstTimer       = bt;
}

void ParticlePreviewPanel::Restart()
{
    ParticleSim::Reset(m_Emitter);
}

void ParticlePreviewPanel::ApplyToEntity(entt::entity target)
{
    if (!m_Context || !m_Context->ActiveScene || target == entt::null) return;
    Scene* scene = m_Context->ActiveScene;
    auto&  reg   = scene->GetRegistry();
    if (!reg.valid(target) || !reg.all_of<ParticleEmitterComponent>(target)) return;

    // Snapshot config-only states (pools cleared) for a clean undo round-trip.
    ParticleEmitterComponent oldC = reg.get<ParticleEmitterComponent>(target);
    ParticleSim::Reset(oldC);
    ParticleEmitterComponent newC = m_Emitter;
    ParticleSim::Reset(newC);

    auto setter = [scene, target](const ParticleEmitterComponent& v) {
        auto& r = scene->GetRegistry();
        if (!r.valid(target) || !r.all_of<ParticleEmitterComponent>(target)) return;
        auto& dst = r.get<ParticleEmitterComponent>(target);
        dst = v;
        ParticleSim::Reset(dst);   // never push live particles into the scene component
    };

    m_Context->Commands.ExecuteCommand(
        std::make_unique<ValueChangeCommand<ParticleEmitterComponent>>(
            setter, oldC, newC, "Apply Particle Effect"));
}

void ParticlePreviewPanel::Tick(float dt)
{
    entt::entity src = ResolveSource();

    // On a new source, load its config and restart so the preview tracks selection.
    if (src != m_Source) {
        m_Source = src;
        if (src != entt::null)
            LoadFromSource(m_Context->ActiveScene->GetRegistry().get<ParticleEmitterComponent>(src));
    }

    m_HasSource = (src != entt::null);
    if (!m_HasSource) return;

    if (m_SyncFromSelection)
        MirrorConfig(m_Context->ActiveScene->GetRegistry().get<ParticleEmitterComponent>(src));

    if (!m_Paused) {
        ParticleSim::Step(m_Emitter, dt * m_TimeScale, glm::mat4(1.0f));
        // Re-fire one-shot bursts once everything has died out.
        if (m_AutoLoop && m_Emitter.liveCount == 0)
            Restart();
    }
}

void ParticlePreviewPanel::OnImGuiRender()
{
    ImGui::Begin("Particle Preview");

    // --- Toolbar: playback on the left, Restart pinned to the right ---
    if (ImGui::Button(m_Paused ? "Play" : "Pause")) m_Paused = !m_Paused;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("Speed", &m_TimeScale, 0.0f, 3.0f, "%.2fx");
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &m_AutoLoop);
    ImGui::SameLine();
    bool pinnedBefore = m_Pinned;
    ImGui::Checkbox("Pin", &m_Pinned);
    if (m_Pinned && !pinnedBefore) m_PinnedEntity = m_Source;   // lock onto current source

    // Restart, right-aligned.
    {
        const char* label = "Restart";
        float w = ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SameLine(ImGui::GetWindowWidth() - w - ImGui::GetStyle().WindowPadding.x);
        if (ImGui::Button(label)) Restart();
    }

    // --- Source / editing row ---
    ImGui::Checkbox("Sync from selection", &m_SyncFromSelection);
    if (!m_SyncFromSelection) {
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_HasSource);
        if (ImGui::Button("Apply to Entity")) ApplyToEntity(m_Source);
        ImGui::SameLine();
        if (ImGui::Button("Reload") && m_HasSource)
            LoadFromSource(m_Context->ActiveScene->GetRegistry().get<ParticleEmitterComponent>(m_Source));
        ImGui::EndDisabled();

        // Detached: edit the working copy directly; previews live.
        if (ImGui::CollapsingHeader("Emitter Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::BeginChild("##emitterSettings", ImVec2(0, 220.0f), true);
            DrawComponentInspector<ParticleEmitterComponent>(m_Emitter);
            ImGui::EndChild();
        }
    }

    ImGui::Separator();

    // --- Preview image fills the remaining space ---
    ImVec2 avail = ImGui::GetContentRegionAvail();
    m_DesiredSize = { std::max(1, (int)avail.x), std::max(1, (int)avail.y) };
    m_Aspect      = avail.y > 0.0f ? avail.x / avail.y : 1.0f;

    ImVec2 imgPos = ImGui::GetCursorScreenPos();
    if (m_TextureID != 0)
        ImGui::Image((ImTextureID)(uintptr_t)m_TextureID, avail, {0, 1}, {1, 0});

    bool imgHovered = ImGui::IsItemHovered();

    if (!m_HasSource) {
        const char* hint = "Select an entity with a Particle Emitter";
        ImVec2 ts = ImGui::CalcTextSize(hint);
        ImGui::GetWindowDrawList()->AddText(
            { imgPos.x + (avail.x - ts.x) * 0.5f, imgPos.y + (avail.y - ts.y) * 0.5f },
            ImGui::GetColorU32(ImGuiCol_TextDisabled), hint);
    }

    // --- Orbit camera: left-drag to orbit (Shift = pan), wheel to dolly ---
    ImGuiIO& io = ImGui::GetIO();
    if (imgHovered) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            if (io.KeyShift) m_Camera.Pan(io.MouseDelta.x, io.MouseDelta.y, avail.y);
            else             m_Camera.Orbit(io.MouseDelta.x, io.MouseDelta.y);
        }
        if (io.MouseWheel != 0.0f) m_Camera.Dolly(io.MouseWheel);
    }
    m_Camera.UpdateOrbit(io.DeltaTime);

    ImGui::End();
}
