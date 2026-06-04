#include "ContentPanel.h"
#include <imgui.h>

#ifndef ASSETS_DIR
#define ASSETS_DIR "Assets"
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <glad/gl.h>
#include <Assets/ImageLoader.h>
#include <Assets/ModelImporter.h>

#include <filesystem>
#include <algorithm>
#include <cctype>
#include <cstring>

namespace fs = std::filesystem;

// MSVC's path::string() and u8string() both throw on paths with characters
// outside the code page / invalid UTF-16 surrogates. WideCharToMultiByte with
// no error flags substitutes U+FFFD instead of throwing — safe for UI display.
static std::string ToUtf8(const fs::path& p) {
    const std::wstring& ws = p.native();
    if (ws.empty()) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, ws.data(), (int)ws.size(),
                                nullptr, 0, nullptr, nullptr);
    if (n <= 0) return {};
    std::string out(n, '\0');
    WideCharToMultiByte(CP_UTF8, 0, ws.data(), (int)ws.size(),
                        out.data(), n, nullptr, nullptr);
    return out;
}

static std::string LowerExt(const fs::path& p) {
    std::string ext = ToUtf8(p.extension());
    for (char& c : ext) c = (char)std::tolower((unsigned char)c);
    return ext;
}

ContentPanel::ContentPanel()
    : m_AssetsDir(ASSETS_DIR), m_CurrentPath(ASSETS_DIR)
{
    Refresh();
}

ContentPanel::~ContentPanel() {
    for (auto& [key, texID] : m_ThumbnailCache)
        glDeleteTextures(1, &texID);
}

void ContentPanel::Refresh() {
    m_Items.clear();
    m_RenamingPath.clear();
    if (!fs::exists(m_CurrentPath)) return;

    for (auto& entry : fs::directory_iterator(m_CurrentPath)) {
        ContentItem item;
        item.path        = entry.path();
        item.isDirectory = entry.is_directory();
        item.name        = ToUtf8(entry.path().filename());
        item.type        = GetAssetType(entry.path());
        m_Items.push_back(std::move(item));
    }

    std::sort(m_Items.begin(), m_Items.end(), [](const ContentItem& a, const ContentItem& b) {
        if (a.isDirectory != b.isDirectory) return a.isDirectory > b.isDirectory;
        return a.name < b.name;
    });
}

static const char* AssetTypeName(AssetType t) {
    switch (t) {
        case AssetType::Folder:   return "Folder";
        case AssetType::Texture:  return "Texture";
        case AssetType::Mesh:     return "Mesh";
        case AssetType::Material: return "Material";
        case AssetType::Shader:   return "Shader";
        case AssetType::Scene:    return "Scene";
        default:                  return "File";
    }
}

AssetType ContentPanel::GetAssetType(const fs::path& p) {
    if (fs::is_directory(p)) return AssetType::Folder;
    std::string ext = LowerExt(p);
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".tga")
        return AssetType::Texture;
    if (ext == ".obj" || ext == ".fbx" || ext == ".gltf" || ext == ".glb")
        return AssetType::Mesh;
    if (ext == ".mat")  return AssetType::Material;
    if (ext == ".glsl" || ext == ".vert" || ext == ".frag") return AssetType::Shader;
    if (ext == ".scene") return AssetType::Scene;
    return AssetType::File;
}

uint32_t ContentPanel::LoadThumbnail(const fs::path& p, AssetType type) {
    std::string key = ToUtf8(p);

    auto it = m_ThumbnailCache.find(key);
    if (it != m_ThumbnailCache.end()) return it->second;
    if (m_FailedPaths.count(key))     return 0;

    uint32_t texID = 0;

    if (type == AssetType::Texture) {
        Diamond::ImageData img = Diamond::ImageLoader::Load(key, false);
        if (img.Pixels.empty()) { m_FailedPaths.insert(key); return 0; }

        GLenum fmt;
        switch (img.Channels) {
            case 1:  fmt = GL_RED;  break;
            case 3:  fmt = GL_RGB;  break;
            case 4:  fmt = GL_RGBA; break;
            default: m_FailedPaths.insert(key); return 0;
        }

        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        glTexImage2D(GL_TEXTURE_2D, 0, fmt, img.Width, img.Height, 0,
                     fmt, GL_UNSIGNED_BYTE, img.Pixels.data());
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
    } else if (type == AssetType::Mesh) {
        auto meshes = Diamond::ModelImporter::Load(key);
        if (meshes.empty()) { m_FailedPaths.insert(key); return 0; }
        texID = m_MeshRenderer.Render(meshes);
    }

    if (!texID) { m_FailedPaths.insert(key); return 0; }
    m_ThumbnailCache[key] = texID;
    return texID;
}

uint32_t ContentPanel::GetThumbnail(const std::string& path, AssetType type) {
    if (path.empty()) return 0;
    return LoadThumbnail(fs::path(path), type);
}

void ContentPanel::DrawFolderIcon(ImVec2 tl, float size) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    float tabW = size * 0.45f;
    float tabH = size * 0.20f;

    ImU32 tabCol  = IM_COL32(240, 190, 65, 255);
    ImU32 bodyCol = IM_COL32(215, 160, 45, 255);

    // Tab (top-left bump above the body)
    dl->AddRectFilled(
        {tl.x, tl.y + tabH}, {tl.x + tabW, tl.y},
        tabCol, 3.0f, ImDrawFlags_RoundCornersTop);

    // Body
    dl->AddRectFilled(
        {tl.x, tl.y + tabH - 1.0f}, {tl.x + size, tl.y + size},
        bodyCol, 4.0f,
        ImDrawFlags_RoundCornersBottom | ImDrawFlags_RoundCornersTopRight);
}

void ContentPanel::DrawFileIcon(ImVec2 tl, float size, const std::string& ext) {
    ImDrawList* dl  = ImGui::GetWindowDrawList();
    float fold = size * 0.22f;

    ImU32 pageCol;
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".tga")
        pageCol = IM_COL32(80, 130, 220, 255);
    else if (ext == ".obj" || ext == ".fbx" || ext == ".gltf" || ext == ".glb")
        pageCol = IM_COL32(80, 195, 110, 255);
    else if (ext == ".glsl" || ext == ".vert" || ext == ".frag")
        pageCol = IM_COL32(200, 120, 70, 255);
    else
        pageCol = IM_COL32(155, 155, 170, 255);

    // Darken each channel for the dog-ear
    ImU32 foldCol = IM_COL32(
        (ImU32)(((pageCol >> 0)  & 0xFF) * 6 / 10),
        (ImU32)(((pageCol >> 8)  & 0xFF) * 6 / 10),
        (ImU32)(((pageCol >> 16) & 0xFF) * 6 / 10),
        255);

    // Page body (5-point polygon with clipped top-right corner)
    ImVec2 body[5] = {
        {tl.x,              tl.y},
        {tl.x + size - fold, tl.y},
        {tl.x + size,       tl.y + fold},
        {tl.x + size,       tl.y + size},
        {tl.x,              tl.y + size},
    };
    dl->AddConvexPolyFilled(body, 5, pageCol);

    // Dog-ear triangle
    ImVec2 ear[3] = {
        {tl.x + size - fold, tl.y},
        {tl.x + size,        tl.y + fold},
        {tl.x + size - fold, tl.y + fold},
    };
    dl->AddConvexPolyFilled(ear, 3, foldCol);
}

void ContentPanel::DrawToolbar() {
    // Breadcrumb: build full path segments from Assets root down to current
    std::vector<std::pair<std::string, fs::path>> crumbs;
    {
        fs::path acc = m_AssetsDir.parent_path();
        fs::path rel = fs::relative(m_CurrentPath, acc);
        for (auto& part : rel) {
            acc /= part;
            crumbs.push_back({ToUtf8(part), acc});
        }
    }

    for (size_t i = 0; i < crumbs.size(); ++i) {
        if (i > 0) { ImGui::SameLine(); ImGui::TextUnformatted(">"); ImGui::SameLine(); }
        std::string id = crumbs[i].first + "##crumb" + std::to_string(i);
        if (ImGui::SmallButton(id.c_str())) {
            m_CurrentPath = crumbs[i].second;
            Refresh();
        }
    }

    float rightEdge = ImGui::GetContentRegionAvail().x;
    ImGui::SameLine(rightEdge - 145.0f);
    if (ImGui::Button("+ New Folder")) {
        std::memset(m_NewFolderName, 0, sizeof(m_NewFolderName));
        m_OpenNewFolderModal = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Refresh"))
        Refresh();

    ImGui::Separator();
}

void ContentPanel::DrawItems() {
    float panelW  = ImGui::GetContentRegionAvail().x;
    float cellW   = m_IconSize + 16.0f;
    int   cols    = std::max(1, (int)(panelW / cellW));
    float labelH  = ImGui::GetTextLineHeight() * 2.0f + 4.0f;
    float cellH   = m_IconSize + labelH;

    ImGui::Columns(cols, nullptr, false);

    fs::path pendingNav;
    fs::path pendingRenameOld, pendingRenameNew;

    for (auto& item : m_Items) {
        ImGui::PushID(ToUtf8(item.path).c_str());

        ImVec2 cellOrigin = ImGui::GetCursorScreenPos();

        ImGui::InvisibleButton("##cell", {m_IconSize, cellH});
        bool hovered  = ImGui::IsItemHovered();
        bool dblClick = hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);

        // Right-click context menu
        if (ImGui::BeginPopupContextItem("##ctx")) {
            if (ImGui::MenuItem("Rename")) {
                m_RenamingPath   = item.path;
                m_RenameFocusSet = false;
                // Pre-fill with stem only so the user can't accidentally edit the extension
                std::string stem = ToUtf8(item.path.stem());
                std::strncpy(m_RenameBuffer, stem.c_str(), sizeof(m_RenameBuffer) - 1);
                m_RenameBuffer[sizeof(m_RenameBuffer) - 1] = '\0';
            }
            ImGui::EndPopup();
        }

        if (hovered) {
            ImGui::GetWindowDrawList()->AddRectFilled(
                cellOrigin,
                {cellOrigin.x + m_IconSize, cellOrigin.y + cellH},
                IM_COL32(110, 110, 110, 90), 4.0f);
        }

        // Icon (inset slightly)
        float pad  = m_IconSize * 0.09f;
        float iSz  = m_IconSize - pad * 2.0f;
        ImVec2 iPos = {cellOrigin.x + pad, cellOrigin.y + pad};

        uint32_t thumbID = 0;
        if (item.type == AssetType::Texture || item.type == AssetType::Mesh) {
            std::string key = ToUtf8(item.path);
            auto cit = m_ThumbnailCache.find(key);
            if (cit != m_ThumbnailCache.end())
                thumbID = cit->second;
            else if (!m_FailedPaths.count(key) &&
                     ImGui::IsRectVisible(cellOrigin, {cellOrigin.x + m_IconSize, cellOrigin.y + cellH}))
                thumbID = LoadThumbnail(item.path, item.type);
        }

        if (thumbID) {
            ImGui::GetWindowDrawList()->AddImage(
                (ImTextureID)(uintptr_t)thumbID,
                iPos, {iPos.x + iSz, iPos.y + iSz});
        } else if (item.isDirectory) {
            DrawFolderIcon(iPos, iSz);
        } else {
            DrawFileIcon(iPos, iSz, LowerExt(item.path));
        }

        // Name + type label drawn directly via DrawList so we don't disturb the cursor
        ImDrawList* dl    = ImGui::GetWindowDrawList();
        float        lineH = ImGui::GetTextLineHeight();
        float        ty    = cellOrigin.y + m_IconSize + 2.0f;

        // Truncate name to fit cell width (binary search to avoid O(n) loop)
        std::string dispName = item.name;
        float maxW = m_IconSize - 4.0f;
        if (ImGui::CalcTextSize(dispName.c_str()).x > maxW) {
            int lo = 0, hi = (int)dispName.size();
            while (lo < hi) {
                int mid = (lo + hi + 1) / 2;
                std::string probe = dispName.substr(0, mid) + "...";
                if (ImGui::CalcTextSize(probe.c_str()).x <= maxW) lo = mid;
                else hi = mid - 1;
            }
            dispName = dispName.substr(0, lo) + "...";
        }

        if (m_RenamingPath == item.path) {
            // Inject an InputText into the label area below the icon
            ImGui::SetCursorScreenPos({cellOrigin.x, ty});
            ImGui::SetNextItemWidth(m_IconSize);

            if (!m_RenameFocusSet) {
                ImGui::SetKeyboardFocusHere();
                m_RenameFocusSet = true;
            }

            bool confirm = ImGui::InputText("##rename", m_RenameBuffer, sizeof(m_RenameBuffer),
                                            ImGuiInputTextFlags_EnterReturnsTrue);
            bool lostFocus = !ImGui::IsItemActive() && ImGui::IsItemDeactivated();

            if ((confirm || lostFocus) && m_RenameBuffer[0] != '\0') {
                // Re-attach the original extension so it can never be lost
                fs::path newPath = item.path.parent_path()
                                   / (std::string(m_RenameBuffer) + ToUtf8(item.path.extension()));
                if (newPath != item.path) {
                    pendingRenameOld = item.path;
                    pendingRenameNew = newPath;
                }
                m_RenamingPath.clear();
            } else if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                m_RenamingPath.clear();
            }

            dl->AddText({cellOrigin.x + 2.0f, ty + lineH},
                        IM_COL32(130, 130, 130, 255), AssetTypeName(item.type));
        } else {
            dl->AddText({cellOrigin.x + 2.0f, ty},
                        IM_COL32(220, 220, 220, 255), dispName.c_str());
            dl->AddText({cellOrigin.x + 2.0f, ty + lineH},
                        IM_COL32(130, 130, 130, 255), AssetTypeName(item.type));
        }

        // Drag-drop source for mesh and texture assets.
        // BeginDragDropSource checks the InvisibleButton above — no ImGui widgets
        // between it and here, only DrawList calls, so the last item is still valid.
        if (item.type == AssetType::Mesh || item.type == AssetType::Texture) {
            if (ImGui::BeginDragDropSource()) {
                std::string pathStr = ToUtf8(item.path);
                ImGui::SetDragDropPayload("CONTENT_ITEM_PATH",
                                          pathStr.c_str(), pathStr.size() + 1);
                if (thumbID) {
                    ImGui::Image((ImTextureID)(uintptr_t)thumbID, {40.0f, 40.0f});
                    ImGui::SameLine();
                }
                ImGui::TextUnformatted(item.name.c_str());
                ImGui::EndDragDropSource();
            }
        }

        if (dblClick && item.isDirectory)
            pendingNav = item.path;
        else if (dblClick && item.type == AssetType::Scene && m_OnSceneOpen)
            m_OnSceneOpen(ToUtf8(item.path));

        ImGui::NextColumn();
        ImGui::PopID();
    }

    ImGui::Columns(1);

    // Apply deferred operations — Refresh() inside the loop would invalidate iterators.
    if (!pendingRenameOld.empty()) {
        std::error_code ec;
        fs::rename(pendingRenameOld, pendingRenameNew, ec);
        Refresh();
    } else if (!pendingNav.empty()) {
        m_CurrentPath = pendingNav;
        Refresh();
    }
}

void ContentPanel::OnImGuiRender() {
    ImGui::Begin("Content Browser");

    DrawToolbar();
    DrawItems();

    // Trigger modal on the same frame as the button press
    if (m_OpenNewFolderModal) {
        ImGui::OpenPopup("New Folder##modal");
        m_OpenNewFolderModal = false;
    }

    if (ImGui::BeginPopupModal("New Folder##modal", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Folder name:");
        ImGui::SetNextItemWidth(260.0f);
        bool confirm = ImGui::InputText("##fname", m_NewFolderName,
                                        sizeof(m_NewFolderName),
                                        ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SetItemDefaultFocus();

        if ((confirm || ImGui::Button("Create", {125.0f, 0})) && m_NewFolderName[0]) {
            fs::create_directory(m_CurrentPath / m_NewFolderName);
            Refresh();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", {125.0f, 0}))
            ImGui::CloseCurrentPopup();

        ImGui::EndPopup();
    }

    ImGui::End();
}
