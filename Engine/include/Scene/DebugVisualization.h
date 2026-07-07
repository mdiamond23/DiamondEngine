#pragma once
#include <entt/entt.hpp>

class Scene;

// Editor-only debug-line drawers shared by both render backends: they only
// accumulate into DebugDraw's generic vertex buffer (see DebugDraw.h), so the
// same call works whether the frame is later flushed by GL or uploaded to the
// Vulkan debug-draw pass. Gated by EditorContext::showDebugDraw at the call
// site (GLEditorBackend / VulkanEditorBackend).
namespace Diamond {

// Draws the IK chains of the selected entity: each chain's root->mid->tip bone
// segments, joint markers, the resolved world target (with a line to the tip),
// and the pole point. The active chain is highlighted; the rest are dimmed.
void DrawIKDebug(Scene& scene, entt::entity sel, int activeChain);

// Draws spatial-audio cues: a min/max distance sphere pair per 3D
// AudioSourceComponent (the selected one highlighted), the directional cone
// when the source is not omnidirectional, and a marker + forward line for each
// enabled AudioListenerComponent.
void DrawAudioDebug(Scene& scene, entt::entity sel);

} // namespace Diamond
