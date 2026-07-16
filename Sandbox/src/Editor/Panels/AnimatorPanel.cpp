#include "AnimatorPanel.h"
#include "Assets/AnimStateMachineAsset.h"
#include "Animation/AnimationComponents.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>

using namespace Diamond;

// ---- helpers ----------------------------------------------------------------

// Sorted, de-duplicated keyframe times across every channel of a clip — used to
// draw timeline ticks and to step the playhead key-to-key.
static std::vector<float> UniqueKeyTimes(const AnimationClip& clip) {
    std::vector<float> t;
    for (const auto& ch : clip.channels) {
        for (const auto& k : ch.positions) t.push_back(k.time);
        for (const auto& k : ch.rotations) t.push_back(k.time);
        for (const auto& k : ch.scales)    t.push_back(k.time);
    }
    std::sort(t.begin(), t.end());
    t.erase(std::unique(t.begin(), t.end(),
            [](float a, float b) { return std::fabs(a - b) < 1e-4f; }), t.end());
    return t;
}

// ---- panel ------------------------------------------------------------------

void AnimatorPanel::OnImGuiRender() {
    if (!m_Open) return;
    ImGui::Begin("Animator", &m_Open);

    if (!m_Context || !m_Context->HasSelection()) {
        ImGui::TextDisabled("Select an entity with a Skinned Mesh / Animator.");
        ImGui::End();
        return;
    }

    entt::entity e   = m_Context->PrimarySelection();
    auto&        reg = m_Context->ActiveScene->GetRegistry();
    if (!reg.valid(e)) {
        ImGui::TextDisabled("(invalid selection)");
        ImGui::End();
        return;
    }

    if (ImGui::BeginTabBar("##animtabs")) {
        if (ImGui::BeginTabItem("State Machine")) {
            DrawStateMachine(e);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Timeline")) {
            DrawTimeline(e);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}

// ---- timeline (4a) ----------------------------------------------------------

void AnimatorPanel::DrawTimeline(entt::entity e) {
    auto& reg  = m_Context->ActiveScene->GetRegistry();
    auto* smc  = reg.try_get<SkinnedMeshComponent>(e);
    auto* anim = reg.try_get<AnimatorComponent>(e);

    if (!smc || !anim) { ImGui::TextDisabled("Needs a Skinned Mesh and an Animator."); return; }
    if (smc->clips.empty()) { ImGui::TextDisabled("This mesh has no clips."); return; }

    const int clipCount = (int)smc->clips.size();
    auto clipName = [&](int i) {
        if (i < 0) return std::string("(bind pose)");
        std::string n = smc->clips[i].name;
        return n.empty() ? std::to_string(i) : n;
    };

    // Clip selector — instant switch here (the inspector owns cross-fading).
    int clip = anim->clip;
    if (ImGui::BeginCombo("Clip", clipName(clip).c_str())) {
        for (int i = 0; i < clipCount; ++i)
            if (ImGui::Selectable(clipName(i).c_str(), clip == i)) {
                anim->clip = i; anim->time = 0.0f; anim->prevClip = -1;
            }
        ImGui::EndCombo();
    }

    if (clip < 0 || clip >= clipCount) { ImGui::TextDisabled("(bind pose — no timeline)"); return; }
    const AnimationClip& ac = smc->clips[clip];
    const float duration = ac.duration > 0.0f ? ac.duration : 1.0f;
    std::vector<float> keys = UniqueKeyTimes(ac);

    // Transport.
    if (ImGui::Button(anim->playing ? "Pause" : "Play")) anim->playing = !anim->playing;
    ImGui::SameLine();
    if (ImGui::Button("|<")) anim->time = 0.0f;
    ImGui::SameLine();
    // Step to previous keyframe.
    if (ImGui::Button("<")) {
        float best = 0.0f;
        for (float k : keys) if (k < anim->time - 1e-4f) best = k;
        anim->time = best;
    }
    ImGui::SameLine();
    if (ImGui::Button(">")) {
        float best = duration;
        for (auto it = keys.rbegin(); it != keys.rend(); ++it) if (*it > anim->time + 1e-4f) best = *it;
        anim->time = best;
    }
    ImGui::SameLine();
    if (ImGui::Button(">|")) anim->time = duration;
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &anim->loop);

    ImGui::Text("%.3f / %.3f s   |   %zu keys", anim->time, duration, keys.size());
    if (m_Context->ActiveScene->IsPlaying() && !m_Context->ActiveScene->IsPaused())
        ImGui::TextDisabled("(playing — time is driven by the runtime; scrub in edit mode)");

    // Scrub bar.
    const float h = 36.0f;
    ImVec2 origin = ImGui::GetCursorScreenPos();
    float  width  = ImGui::GetContentRegionAvail().x;
    ImGui::InvisibleButton("##scrub", { width, h });
    bool   active = ImGui::IsItemActive();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(origin, { origin.x + width, origin.y + h }, IM_COL32(30, 30, 30, 255), 3.0f);
    dl->AddRect(origin, { origin.x + width, origin.y + h }, IM_COL32(70, 70, 70, 255), 3.0f);

    auto timeToX = [&](float t) { return origin.x + (t / duration) * width; };

    // Keyframe ticks.
    for (float k : keys) {
        float x = timeToX(k);
        dl->AddLine({ x, origin.y + 4.0f }, { x, origin.y + h - 4.0f }, IM_COL32(120, 120, 130, 200));
    }

    // Drag to scrub.
    if (active) {
        float mx = ImGui::GetIO().MousePos.x;
        float t  = (mx - origin.x) / std::max(width, 1.0f) * duration;
        anim->time = std::clamp(t, 0.0f, duration);
    }

    // Playhead.
    float px = timeToX(std::clamp(anim->time, 0.0f, duration));
    dl->AddLine({ px, origin.y }, { px, origin.y + h }, IM_COL32(255, 170, 60, 255), 2.0f);
}

// ---- state machine (4b) -----------------------------------------------------

namespace {

constexpr float kNodeW = 132.0f;
constexpr float kNodeH = 48.0f;

const char* kOpNames[] = { ">", "<", ">=", "<=", "==", "!=", "is true", "is false", "triggered" };

// Removes state `s`, then fixes up every index that referenced states by position:
// drops transitions touching it, shifts higher from/to/entry indices down by one.
void DeleteState(AnimStateMachine& M, int s) {
    M.states.erase(M.states.begin() + s);
    auto& T = M.transitions;
    T.erase(std::remove_if(T.begin(), T.end(),
            [&](const AnimTransition& t) { return t.fromState == s || t.toState == s; }), T.end());
    for (auto& t : T) {
        if (t.fromState > s) --t.fromState;
        if (t.toState   > s) --t.toState;
    }
    if (M.entryState == s) M.entryState = M.states.empty() ? 0 : 0;
    else if (M.entryState > s) --M.entryState;
}

} // namespace

void AnimatorPanel::DrawStateMachine(entt::entity e) {
    auto& reg = m_Context->ActiveScene->GetRegistry();
    auto* smCmp = reg.try_get<AnimStateMachineComponent>(e);
    if (!smCmp) {
        ImGui::TextDisabled("No Animator State Machine on this entity.");
        ImGui::TextDisabled("Add one in the Inspector and assign a .animsm asset.");
        return;
    }
    if (!smCmp->machine) {
        ImGui::TextDisabled("No .animsm assigned. Drag one onto the component in the Inspector.");
        return;
    }
    AnimStateMachine& M   = *smCmp->machine;
    auto*             smc = reg.try_get<SkinnedMeshComponent>(e);

    // -- toolbar --
    if (ImGui::Button("Save")) {
        if (!smCmp->assetPath.empty()) SaveAnimStateMachine(smCmp->assetPath, M);
    }
    ImGui::SameLine();
    if (ImGui::Button("+ State")) {
        AnimState s;
        s.name    = "State " + std::to_string(M.states.size());
        s.nodePos = { -m_Pan.x + 40.0f, -m_Pan.y + 40.0f };
        M.states.push_back(s);
        m_SelState = (int)M.states.size() - 1;
        m_SelTransition = -1;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("|  drag nodes • middle-drag to pan • click to select");

    // -- split: canvas (left) + properties (right) --
    float propW = 290.0f;
    ImGui::BeginChild("##canvas", { ImGui::GetContentRegionAvail().x - propW, 0 }, true,
                      ImGuiWindowFlags_NoScrollWithMouse);
    {
        ImDrawList* dl     = ImGui::GetWindowDrawList();
        ImVec2      cOrig  = ImGui::GetWindowPos();
        bool        clickedSomething = false;

        auto nodeMin = [&](const AnimState& s) {
            return ImVec2(cOrig.x + m_Pan.x + s.nodePos.x, cOrig.y + m_Pan.y + s.nodePos.y);
        };
        auto nodeCenter = [&](const AnimState& s) {
            ImVec2 m = nodeMin(s);
            return ImVec2(m.x + kNodeW * 0.5f, m.y + kNodeH * 0.5f);
        };

        // Transitions (drawn under the nodes), each with a small hit-target at its midpoint.
        for (int i = 0; i < (int)M.transitions.size(); ++i) {
            const AnimTransition& t = M.transitions[i];
            if (t.fromState < 0 || t.fromState >= (int)M.states.size()) continue;
            if (t.toState   < 0 || t.toState   >= (int)M.states.size()) continue;
            ImVec2 a = nodeCenter(M.states[t.fromState]);
            ImVec2 b = nodeCenter(M.states[t.toState]);
            bool sel = (m_SelTransition == i);
            ImU32 col = sel ? IM_COL32(255, 200, 90, 255) : IM_COL32(150, 150, 160, 220);
            dl->AddLine(a, b, col, sel ? 3.0f : 2.0f);
            // Arrowhead near the target.
            ImVec2 d = { b.x - a.x, b.y - a.y };
            float  len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len > 1.0f) {
                d.x /= len; d.y /= len;
                ImVec2 tip = { b.x - d.x * (kNodeW * 0.5f), b.y - d.y * (kNodeH * 0.5f) };
                ImVec2 n   = { -d.y, d.x };
                dl->AddTriangleFilled(
                    tip,
                    { tip.x - d.x * 12.0f + n.x * 6.0f, tip.y - d.y * 12.0f + n.y * 6.0f },
                    { tip.x - d.x * 12.0f - n.x * 6.0f, tip.y - d.y * 12.0f - n.y * 6.0f },
                    col);
            }
            ImVec2 mid = { (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f };
            ImGui::SetCursorScreenPos({ mid.x - 8.0f, mid.y - 8.0f });
            ImGui::PushID(10000 + i);
            if (ImGui::InvisibleButton("##tr", { 16.0f, 16.0f })) {
                m_SelTransition = i; m_SelState = -1; clickedSomething = true;
            }
            ImGui::PopID();
        }

        // Nodes.
        for (int i = 0; i < (int)M.states.size(); ++i) {
            AnimState& s   = M.states[i];
            ImVec2     mn  = nodeMin(s);
            ImVec2     mx  = { mn.x + kNodeW, mn.y + kNodeH };

            ImGui::SetCursorScreenPos(mn);
            ImGui::PushID(i);
            ImGui::InvisibleButton("##node", { kNodeW, kNodeH });
            bool hov = ImGui::IsItemHovered();
            if (ImGui::IsItemActivated()) { m_SelState = i; m_SelTransition = -1; clickedSomething = true; }
            if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                s.nodePos.x += ImGui::GetIO().MouseDelta.x;
                s.nodePos.y += ImGui::GetIO().MouseDelta.y;
                m_DragState = i;
            }
            ImGui::PopID();

            bool  isEntry   = (i == M.entryState);
            bool  isCurrent = (smCmp->currentState == i);
            bool  isSel     = (m_SelState == i);
            ImU32 fill = isCurrent ? IM_COL32(40, 70, 55, 255) : IM_COL32(48, 48, 56, 255);
            ImU32 border = isSel    ? IM_COL32(255, 200, 90, 255)
                         : isCurrent? IM_COL32(90, 220, 140, 255)
                         : isEntry  ? IM_COL32(90, 200, 90, 255)
                         : hov      ? IM_COL32(140, 140, 160, 255)
                                    : IM_COL32(90, 90, 100, 255);
            dl->AddRectFilled(mn, mx, fill, 5.0f);
            dl->AddRect(mn, mx, border, 5.0f, 0, isSel ? 2.5f : 1.5f);

            dl->AddText({ mn.x + 8.0f, mn.y + 6.0f }, IM_COL32(235, 235, 235, 255),
                        s.name.empty() ? "(unnamed)" : s.name.c_str());
            std::string sub = s.clipName.empty() ? "(no clip)" : s.clipName;
            dl->AddText({ mn.x + 8.0f, mn.y + 26.0f }, IM_COL32(150, 150, 160, 255), sub.c_str());
            if (isEntry)
                dl->AddText({ mx.x - 42.0f, mn.y + 6.0f }, IM_COL32(90, 200, 90, 255), "entry");
        }

        // Pan with middle mouse over empty canvas; click empty to deselect.
        if (ImGui::IsWindowHovered()) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
                m_Pan.x += ImGui::GetIO().MouseDelta.x;
                m_Pan.y += ImGui::GetIO().MouseDelta.y;
            }
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !clickedSomething) {
                m_SelState = -1; m_SelTransition = -1;
            }
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) m_DragState = -1;
    }
    ImGui::EndChild();

    ImGui::SameLine();
    ImGui::BeginChild("##props", { 0, 0 }, true);
    {
        // ---- selected state ----
        if (m_SelState >= 0 && m_SelState < (int)M.states.size()) {
            AnimState& s = M.states[m_SelState];
            ImGui::TextDisabled("STATE");

            char buf[64];
            std::strncpy(buf, s.name.c_str(), sizeof(buf)); buf[sizeof(buf) - 1] = '\0';
            if (ImGui::InputText("Name", buf, sizeof(buf))) s.name = buf;

            // Clip combo from the entity's skinned mesh.
            std::string cur = s.clipName.empty() ? "(none)" : s.clipName;
            if (ImGui::BeginCombo("Clip", cur.c_str())) {
                if (smc) for (const auto& c : smc->clips) {
                    bool selrow = (c.name == s.clipName);
                    if (ImGui::Selectable(c.name.c_str(), selrow)) s.clipName = c.name;
                }
                if (!smc) ImGui::TextDisabled("(no skinned mesh)");
                ImGui::EndCombo();
            }

            ImGui::DragFloat("Speed", &s.speed, 0.01f, -10.0f, 10.0f, "%.2f");
            ImGui::Checkbox("Loop", &s.loop);

            if (m_SelState == M.entryState) ImGui::TextDisabled("(entry state)");
            else if (ImGui::Button("Set as Entry")) M.entryState = m_SelState;

            // Add a transition from this state to another.
            ImGui::Spacing();
            if (ImGui::BeginCombo("+ Transition to", "select...")) {
                for (int j = 0; j < (int)M.states.size(); ++j) {
                    if (j == m_SelState) continue;
                    if (ImGui::Selectable(M.states[j].name.c_str())) {
                        AnimTransition t; t.fromState = m_SelState; t.toState = j;
                        M.transitions.push_back(t);
                        m_SelTransition = (int)M.transitions.size() - 1;
                        m_SelState = -1;
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
            if (ImGui::Button("Delete State")) { DeleteState(M, m_SelState); m_SelState = -1; }
            ImGui::PopStyleColor();
        }
        // ---- selected transition ----
        else if (m_SelTransition >= 0 && m_SelTransition < (int)M.transitions.size()) {
            AnimTransition& t = M.transitions[m_SelTransition];
            ImGui::TextDisabled("TRANSITION");
            auto stateName = [&](int i) {
                return (i >= 0 && i < (int)M.states.size()) ? M.states[i].name.c_str() : "?";
            };
            ImGui::Text("%s  ->  %s", stateName(t.fromState), stateName(t.toState));
            ImGui::DragFloat("Blend (s)", &t.blendDuration, 0.01f, 0.0f, 2.0f, "%.2f");

            ImGui::Spacing();
            ImGui::TextDisabled("Conditions (all must pass)");
            int removeCond = -1;
            for (int ci = 0; ci < (int)t.conditions.size(); ++ci) {
                AnimCondition& c = t.conditions[ci];
                ImGui::PushID(ci);
                ImGui::SetNextItemWidth(110.0f);
                std::string pcur = c.parameter.empty() ? "(param)" : c.parameter;
                if (ImGui::BeginCombo("##p", pcur.c_str())) {
                    for (const auto& p : M.parameters)
                        if (ImGui::Selectable(p.name.c_str(), p.name == c.parameter)) c.parameter = p.name;
                    ImGui::EndCombo();
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0f);
                int op = (int)c.op;
                if (ImGui::Combo("##op", &op, kOpNames, IM_ARRAYSIZE(kOpNames))) c.op = (AnimCondOp)op;
                // Threshold only matters for the float comparisons.
                if (c.op <= AnimCondOp::NotEqual) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(60.0f);
                    ImGui::DragFloat("##v", &c.value, 0.01f);
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("x")) removeCond = ci;
                ImGui::PopID();
            }
            if (removeCond >= 0) t.conditions.erase(t.conditions.begin() + removeCond);
            if (ImGui::SmallButton("+ Condition")) t.conditions.push_back({});

            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
            if (ImGui::Button("Delete Transition")) {
                M.transitions.erase(M.transitions.begin() + m_SelTransition);
                m_SelTransition = -1;
            }
            ImGui::PopStyleColor();
        }
        else {
            ImGui::TextDisabled("Select a state or transition.");
        }

        // ---- parameters (always shown) ----
        ImGui::Separator();
        ImGui::TextDisabled("PARAMETERS");
        int removeParam = -1;
        for (int pi = 0; pi < (int)M.parameters.size(); ++pi) {
            AnimParameter& p = M.parameters[pi];
            ImGui::PushID(20000 + pi);
            const char* ty = p.type == AnimParamType::Float ? "f"
                           : p.type == AnimParamType::Bool  ? "b" : "t";
            ImGui::Text("[%s] %s", ty, p.name.c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) removeParam = pi;
            ImGui::PopID();
        }
        if (removeParam >= 0) {
            std::string gone = M.parameters[removeParam].name;
            M.parameters.erase(M.parameters.begin() + removeParam);
            for (auto& t : M.transitions)
                t.conditions.erase(std::remove_if(t.conditions.begin(), t.conditions.end(),
                    [&](const AnimCondition& c) { return c.parameter == gone; }), t.conditions.end());
        }

        ImGui::Spacing();
        static char  newParamName[64] = "";
        static int   newParamType     = 0;
        ImGui::SetNextItemWidth(110.0f);
        ImGui::InputText("##np", newParamName, sizeof(newParamName));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70.0f);
        const char* types[] = { "Float", "Bool", "Trigger" };
        ImGui::Combo("##nt", &newParamType, types, IM_ARRAYSIZE(types));
        ImGui::SameLine();
        if (ImGui::SmallButton("+ Param") && newParamName[0]) {
            AnimParameter p;
            p.name = newParamName;
            p.type = (AnimParamType)newParamType;
            M.parameters.push_back(p);
            smCmp->SyncParams();
            newParamName[0] = '\0';
        }
    }
    ImGui::EndChild();
}
