#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <imgui.h>
#include <entt/entt.hpp>

// The Animator window — a view over the selected entity's animation data.
//   • Timeline tab: scrub / step / play the AnimatorComponent's current clip.
//   • State Machine tab: a hand-rolled node canvas for the entity's .animsm graph
//     (states as boxes, transitions as arrows), with property + parameter editing.
// Follows the primary selection; everything edits live and the graph saves back
// to the .animsm asset on disk.
class AnimatorPanel : public Panel {
public:
    void SetContext(EditorContext* ctx) { m_Context = ctx; }
    void OnImGuiRender() override;

private:
    void DrawTimeline(entt::entity e);
    void DrawStateMachine(entt::entity e);

    EditorContext* m_Context = nullptr;

    // State-machine canvas editing state (indices into the machine's vectors).
    int    m_SelState      = -1;
    int    m_SelTransition = -1;
    int    m_DragState     = -1;   // node being dragged this frame
    ImVec2 m_Pan           = ImVec2(0.0f, 0.0f);
};
