#pragma once
#include "Panels.h"
#include <imgui.h>
#include <filesystem>
#include <string>
#include <vector>

struct ContentItem {
    std::filesystem::path path;
    bool isDirectory;
    std::string name;
    std::string typeLabel;
};

class ContentPanel : public Panel {
public:
    ContentPanel();
    void OnImGuiRender() override;

private:
    void Refresh();
    void DrawToolbar();
    void DrawItems();
    void DrawFolderIcon(ImVec2 topLeft, float size);
    void DrawFileIcon(ImVec2 topLeft, float size, const std::string& ext);
    std::string GetFileTypeLabel(const std::filesystem::path& p);

    std::filesystem::path m_AssetsDir;
    std::filesystem::path m_CurrentPath;
    std::vector<ContentItem> m_Items;

    bool m_OpenNewFolderModal = false;
    char m_NewFolderName[256]{};

    float m_IconSize = 68.0f;
};
