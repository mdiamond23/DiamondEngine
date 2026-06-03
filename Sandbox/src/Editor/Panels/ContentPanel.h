#pragma once
#include "Panels.h"
#include "../MeshThumbnailRenderer.h"
#include <imgui.h>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

enum class AssetType { Folder, Texture, Mesh, Material, Shader, Scene, File };

struct ContentItem {
    std::filesystem::path path;
    bool isDirectory  = false;
    std::string name;
    AssetType type    = AssetType::File;
};

class ContentPanel : public Panel {
public:
    ContentPanel();
    ~ContentPanel();
    void OnImGuiRender() override;

    void SetOnSceneOpen(std::function<void(const std::string&)> cb) { m_OnSceneOpen = std::move(cb); }

private:
    void Refresh();
    void DrawToolbar();
    void DrawItems();
    void DrawFolderIcon(ImVec2 topLeft, float size);
    void DrawFileIcon(ImVec2 topLeft, float size, const std::string& ext);
    AssetType GetAssetType(const std::filesystem::path& p);
    uint32_t LoadThumbnail(const std::filesystem::path& p, AssetType type);

    std::filesystem::path m_AssetsDir;
    std::filesystem::path m_CurrentPath;
    std::vector<ContentItem> m_Items;

    std::unordered_map<std::string, uint32_t> m_ThumbnailCache;
    std::unordered_set<std::string>           m_FailedPaths;
    MeshThumbnailRenderer                     m_MeshRenderer;

    std::function<void(const std::string&)> m_OnSceneOpen;

    bool m_OpenNewFolderModal = false;
    char m_NewFolderName[256]{};

    float m_IconSize = 68.0f;
};
