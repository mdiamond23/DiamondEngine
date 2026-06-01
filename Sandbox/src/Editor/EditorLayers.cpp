#include <imgui_internal.h>
#include <cstdint>
#include "EditorLayers.h"

EditorLayer::EditorLayer(Scene* scene)
{
    m_Context.ActiveScene = scene;
    m_Hierarchy.SetContext(&m_Context);
    m_Inspector.SetContext(&m_Context);
}

void EditorLayer::SetupDockspace()
{
    if (m_LayoutInitialized) return;
    m_LayoutInitialized = true;

    ImGuiID id = ImGui::GetID("MainDockSpace");

    // Set up central dock
    ImGui::DockBuilderRemoveNode(id);
    ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(id, ImGui::GetMainViewport()->Size);

    // Split dock into 4
    ImGuiID center = id;
    ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.2f, nullptr, &center);
    ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f, nullptr, &center);
    ImGuiID down = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.28f, nullptr, &center);

    // Attach and finish dock
    ImGui::DockBuilderDockWindow("Hierarchy", left);
    ImGui::DockBuilderDockWindow("Viewport", center);
    ImGui::DockBuilderDockWindow("Inspector", right);
    ImGui::DockBuilderDockWindow("Content Browser", down);
    ImGui::DockBuilderFinish(id);
}

void EditorLayer::SetViewportTexture(uint32_t textureID)
{
    m_Viewport.SetTexture(textureID);
}

void EditorLayer::OnImGuiRender()
{
    // Fullscreen invisible host window
    ImGuiViewport* vp = ImGui::GetMainViewport();
    if (vp)
    {
        ImGui::SetNextWindowPos(vp->Pos);
        ImGui::SetNextWindowSize(vp->Size);
        ImGui::SetNextWindowViewport(vp->ID);
    }


    ImGuiWindowFlags hostFlags =
    ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
    ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove     |
    ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
    ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDocking;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("DockSpaceHost", nullptr, hostFlags);
    ImGui::PopStyleVar();

    ImGuiID dockID = ImGui::GetID("MainDockSpace");
    ImGui::DockSpace(dockID, {0, 0}, ImGuiDockNodeFlags_PassthruCentralNode);
    SetupDockspace();

    ImGui::End();

    // Render each panel
    m_Hierarchy.OnImGuiRender();
    m_Viewport.OnImGuiRender();
    m_Inspector.OnImGuiRender();
    m_Content.OnImGuiRender();
}