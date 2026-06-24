#pragma once
#include "Panels.h"
#include "../MeshThumbnailRenderer.h"
#include <imgui.h>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

enum class AssetType { Folder, Texture, Mesh, SkinnedMesh, Material, Shader, Scene, PhysicsMat, AnimSM, Font, File };

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
    void QueueDroppedFiles(int count, const char** paths);
    void Refresh();

    std::filesystem::path GetCurrentPath() const { return m_CurrentPath; }

    // Returns the cached (or freshly generated) GL texture ID for the given asset path.
    // Returns 0 if the path is empty, the asset type has no preview, or loading fails.
    uint32_t GetThumbnail(const std::string& path, AssetType type);

    // Drops the cached thumbnail for an asset so it re-renders next frame — used
    // when a material is edited so its preview ball reflects the change.
    void InvalidateThumbnail(const std::string& path);

private:
    void DrawToolbar();
    void DrawItems();
    void DrawFolderIcon(ImVec2 topLeft, float size, bool highlighted = false);
    void DrawFileIcon(ImVec2 topLeft, float size, const std::string& ext);
    AssetType GetAssetType(const std::filesystem::path& p);
    uint32_t LoadThumbnail(const std::filesystem::path& p, AssetType type);

    std::filesystem::path m_AssetsDir;
    std::filesystem::path m_CurrentPath;
    std::vector<ContentItem> m_Items;

    std::unordered_map<std::string, uint32_t> m_ThumbnailCache;
    std::unordered_set<std::string>           m_FailedPaths;
    MeshThumbnailRenderer                     m_MeshRenderer;

    // glTF skinned-vs-static results, cached so GetAssetType doesn't re-parse a
    // file on every Refresh(). Keyed by UTF-8 path.
    std::unordered_map<std::string, bool>     m_SkinnedCache;

    std::function<void(const std::string&)> m_OnSceneOpen;
    std::vector<std::filesystem::path>      m_PendingDropFiles;

    bool m_OpenNewFolderModal = false;
    char m_NewFolderName[256]{};

    bool                   m_OpenDeleteModal = false;
    std::filesystem::path  m_DeletingPath;

    bool m_OpenMultiDeleteModal = false;

    // Internal drag-drop moves — queued during render, applied after loop exits.
    std::vector<std::pair<std::filesystem::path, std::filesystem::path>> m_PendingMoves;

    std::filesystem::path m_RenamingPath;
    char                  m_RenameBuffer[256]{};
    bool                  m_RenameFocusSet = false;

    // Multi-selection state
    std::unordered_set<std::string> m_SelectedPaths;
    int                             m_SelectionPivotIdx = -1;

    float m_IconSize = 68.0f;
};
