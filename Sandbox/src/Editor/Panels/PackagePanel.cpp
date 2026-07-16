#include "PackagePanel.h"

#include "Assets/AssetPathUtils.h"
#include "Assets/DDSLoader.h"
#include "Assets/ImageLoader.h"
#include "AssetPipeline/TextureCooker.h"

#include <imgui.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_set>

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <windows.h>
    #include <commdlg.h>
    #include <shobjidl.h>
    #include <shellapi.h>
    #define DIAMOND_POPEN  _popen
    #define DIAMOND_PCLOSE _pclose
#else
    #define DIAMOND_POPEN  popen
    #define DIAMOND_PCLOSE pclose
#endif

namespace fs = std::filesystem;

namespace {

// SceneRenderer bakes this environment unconditionally at creation (see the
// default-IBL path in SceneRenderer.cpp) — every package needs it regardless
// of what the scenes reference.
constexpr const char* kDefaultIblHdr = "Assets/Textures/citrus_orchard_road_puresky_4k.hdr";

// Text asset formats the dependency walk scans for further "Assets/..." refs.
bool IsTextAsset(const std::string& lowerExt)
{
    return lowerExt == ".scene"   || lowerExt == ".prefab"  || lowerExt == ".mat" ||
           lowerExt == ".physmat" || lowerExt == ".ragdoll" || lowerExt == ".animsm";
}

std::string LowerExtOf(const std::string& ref)
{
    std::string e = fs::path(ref).extension().string();
    for (char& c : e) c = (char)std::tolower((unsigned char)c);
    return e;
}

bool StartsWithAssets(const std::string& ref)
{
    return AssetPaths::LowerGeneric(ref).rfind("assets/", 0) == 0;
}

std::string ReadFileText(const fs::path& p)
{
    std::ifstream f(p, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Every quoted "Assets/..." string in a serialized asset is a dependency —
// exactly the grep the hand-assembly script (which the packaged runtime was
// verified against) used. AssetPaths::MakePortable guarantees serialized
// project paths take this shape.
void ScanAssetRefs(const std::string& text, std::vector<std::string>& out)
{
    size_t pos = 0;
    while ((pos = text.find("\"Assets/", pos)) != std::string::npos) {
        const size_t start = pos + 1;
        const size_t end   = text.find('"', start);
        if (end == std::string::npos) break;
        out.push_back(text.substr(start, end - start));
        pos = end + 1;
    }
}

// glTF URIs are percent-encoded ("my%20texture.png").
std::string PercentDecode(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '%' && i + 2 < s.size() &&
            std::isxdigit((unsigned char)s[i + 1]) && std::isxdigit((unsigned char)s[i + 2])) {
            out += (char)std::strtol(s.substr(i + 1, 2).c_str(), nullptr, 16);
            i += 2;
        } else {
            out += s[i];
        }
    }
    return out;
}

// .gltf sidecars: external buffers (.bin) and images are named by relative
// URI, not "Assets/..." strings, so the quoted-ref scan misses them.
void ScanGltfRefs(const fs::path& abs, const std::string& dirPortable,
                  std::vector<std::string>& out)
{
    try {
        std::ifstream f(abs);
        const nlohmann::json j = nlohmann::json::parse(f);
        for (const char* key : { "buffers", "images" }) {
            if (!j.contains(key)) continue;
            for (const auto& item : j[key]) {
                const std::string uri = item.value("uri", std::string());
                if (uri.empty() || uri.rfind("data:", 0) == 0) continue;
                out.push_back((fs::path(dirPortable) / PercentDecode(uri))
                                  .lexically_normal().generic_string());
            }
        }
    } catch (const std::exception& e) {
        spdlog::warn("[Packager] failed to parse gltf '{}': {}", abs.string(), e.what());
    }
}

// .mtl texture statements: "map_Kd [options] file.png" and friends. Options
// precede the filename, so the last whitespace token is the path (breaks on
// filenames with spaces — as does most of the OBJ ecosystem).
void ScanMtlRefs(const fs::path& abs, const std::string& dirPortable,
                 std::vector<std::string>& out)
{
    std::ifstream f(abs);
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string keyword;
        if (!(ss >> keyword)) continue;
        for (char& c : keyword) c = (char)std::tolower((unsigned char)c);
        if (keyword.rfind("map_", 0) != 0 && keyword != "bump" &&
            keyword != "norm" && keyword != "disp" && keyword != "refl" &&
            keyword != "decal")
            continue;
        std::string tok, file;
        while (ss >> tok) file = tok;
        if (file.empty()) continue;
        fs::path p(file);
        if (p.is_relative()) p = fs::path(dirPortable) / p;
        out.push_back(p.lexically_normal().generic_string());
    }
}

std::string HumanSize(uint64_t bytes)
{
    char buf[32];
    if (bytes >= (1ull << 30))
        std::snprintf(buf, sizeof(buf), "%.2f GB", (double)bytes / (1ull << 30));
    else if (bytes >= (1ull << 20))
        std::snprintf(buf, sizeof(buf), "%.1f MB", (double)bytes / (1ull << 20));
    else
        std::snprintf(buf, sizeof(buf), "%.0f KB", (double)bytes / (1ull << 10));
    return buf;
}

#ifdef _WIN32
std::string WideToUtf8(const wchar_t* w)
{
    const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (n <= 1) return {};
    std::string s((size_t)n - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
    return s;
}
#endif

// Native folder picker. The classic GetOpenFileNameA can't pick directories —
// this is the one place the editor needs IFileDialog.
std::string PickFolderDialog()
{
#ifdef _WIN32
    std::string result;
    const HRESULT hrInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    IFileDialog* dlg = nullptr;
    if (SUCCEEDED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_ALL,
                                   IID_PPV_ARGS(&dlg)))) {
        DWORD opts = 0;
        dlg->GetOptions(&opts);
        dlg->SetOptions(opts | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM);
        if (SUCCEEDED(dlg->Show(nullptr))) {
            IShellItem* item = nullptr;
            if (SUCCEEDED(dlg->GetResult(&item))) {
                PWSTR psz = nullptr;
                if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &psz))) {
                    result = WideToUtf8(psz);
                    CoTaskMemFree(psz);
                }
                item->Release();
            }
        }
        dlg->Release();
    }
    if (SUCCEEDED(hrInit)) CoUninitialize();
    return result;
#else
    return {};
#endif
}

std::string OpenFileDialogFiltered(const char* filter, const char* initialDir)
{
#ifdef _WIN32
    OPENFILENAMEA ofn{};
    char buf[MAX_PATH]{};
    ofn.lStructSize     = sizeof(ofn);
    ofn.lpstrFilter     = filter;
    ofn.lpstrFile       = buf;
    ofn.nMaxFile        = MAX_PATH;
    ofn.lpstrInitialDir = initialDir;
    ofn.Flags           = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    return GetOpenFileNameA(&ofn) ? buf : "";
#else
    (void)filter; (void)initialDir;
    return {};
#endif
}

// Drag-drop from the Content Browser: scenes travel as "CONTENT_MOVE" (only
// inspector-droppable types use "CONTENT_ITEM_PATH"), but accept both so a
// future payload reclassification doesn't silently break the drop.
std::string AcceptContentDrop()
{
    std::string path;
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload("CONTENT_MOVE"))
            path = (const char*)p->Data;
        else if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH"))
            path = (const char*)p->Data;
        ImGui::EndDragDropTarget();
    }
    return path;
}

#ifdef _WIN32
// Explorer-visible exe icon, injected via UpdateResource — no rcedit-style
// external tool, no .ico file on disk. The source image is decoded with the
// engine's ImageLoader, area-downscaled to the standard Explorer sizes, and
// each size is embedded as a classic 32bpp BMP ICO entry, so any source
// resolution works (the 2K-texture-as-icon case included). The runtime's
// window/taskbar icon is set separately from boot.json at launch and is
// unaffected by any of this.
#pragma pack(push, 2)
struct GrpIconDirEntry {
    BYTE  width, height, colorCount, reserved;
    WORD  planes, bitCount;
    DWORD bytesInRes;
    WORD  id;
};
#pragma pack(pop)

// Box-filter downscale to a square dst x dst (never upscales — callers pick
// sizes <= source). Non-square sources stretch; icons are square by convention.
std::vector<uint8_t> DownscaleRgba(const uint8_t* src, int sw, int sh, int dst)
{
    std::vector<uint8_t> out((size_t)dst * dst * 4);
    for (int y = 0; y < dst; ++y) {
        const int y0 = y * sh / dst, y1 = std::max(y0 + 1, (y + 1) * sh / dst);
        for (int x = 0; x < dst; ++x) {
            const int x0 = x * sw / dst, x1 = std::max(x0 + 1, (x + 1) * sw / dst);
            uint32_t acc[4] = {};
            for (int sy = y0; sy < y1; ++sy)
                for (int sx = x0; sx < x1; ++sx) {
                    const uint8_t* p = src + ((size_t)sy * sw + sx) * 4;
                    acc[0] += p[0]; acc[1] += p[1]; acc[2] += p[2]; acc[3] += p[3];
                }
            const uint32_t n = (uint32_t)(y1 - y0) * (uint32_t)(x1 - x0);
            uint8_t* q = &out[((size_t)y * dst + x) * 4];
            for (int c = 0; c < 4; ++c) q[c] = (uint8_t)(acc[c] / n);
        }
    }
    return out;
}

// One RT_ICON payload: BITMAPINFOHEADER (height doubled — XOR then AND block)
// + bottom-up BGRA rows + an all-zero 1bpp AND mask (alpha governs for 32bpp
// entries, but the mask must still be present and row-padded to 32 bits).
std::vector<uint8_t> EncodeIcoBmp(const std::vector<uint8_t>& rgba, int s)
{
    const uint32_t xorStride = (uint32_t)s * 4;
    const uint32_t andStride = (((uint32_t)s + 31) / 32) * 4;
    std::vector<uint8_t> out(40 + (xorStride + andStride) * (uint32_t)s, 0);
    auto put32 = [&](size_t o, uint32_t v) {
        out[o]     = (uint8_t)v;         out[o + 1] = (uint8_t)(v >> 8);
        out[o + 2] = (uint8_t)(v >> 16); out[o + 3] = (uint8_t)(v >> 24);
    };
    put32(0, 40);                                       // biSize
    put32(4, (uint32_t)s);                              // biWidth
    put32(8, (uint32_t)s * 2);                          // biHeight (doubled)
    out[12] = 1;                                        // biPlanes
    out[14] = 32;                                       // biBitCount
    put32(20, (xorStride + andStride) * (uint32_t)s);   // biSizeImage
    for (int y = 0; y < s; ++y) {
        const uint8_t* srcRow = &rgba[(size_t)(s - 1 - y) * s * 4];
        uint8_t*       dstRow = &out[40 + (size_t)y * xorStride];
        for (int x = 0; x < s; ++x) {                   // RGBA -> BGRA
            dstRow[x * 4 + 0] = srcRow[x * 4 + 2];
            dstRow[x * 4 + 1] = srcRow[x * 4 + 1];
            dstRow[x * 4 + 2] = srcRow[x * 4 + 0];
            dstRow[x * 4 + 3] = srcRow[x * 4 + 3];
        }
    }
    return out;
}

bool EmbedExeIcon(const fs::path& exe, const fs::path& image, std::string& whyNot)
{
    const Diamond::ImageData img =
        Diamond::ImageLoader::Load(image.string(), /*flipVertically*/ false);
    if (img.Pixels.empty() || (img.Channels != 3 && img.Channels != 4)) {
        whyNot = "could not decode the image (or unsupported channel count)";
        return false;
    }
    // Expand RGB to opaque RGBA once so the scaler sees a single format.
    std::vector<uint8_t> expanded;
    const uint8_t* pixels = img.Pixels.data();
    if (img.Channels == 3) {
        expanded.assign((size_t)img.Width * img.Height * 4, 255);
        for (size_t i = 0, n = (size_t)img.Width * img.Height; i < n; ++i) {
            expanded[i * 4 + 0] = img.Pixels[i * 3 + 0];
            expanded[i * 4 + 1] = img.Pixels[i * 3 + 1];
            expanded[i * 4 + 2] = img.Pixels[i * 3 + 2];
        }
        pixels = expanded.data();
    }

    // Standard Explorer set, capped at the source size (never upscale); a
    // tiny source still gets one entry at its own size.
    std::vector<int> sizes;
    for (int s : { 16, 32, 48, 256 })
        if (s <= img.Width && s <= img.Height) sizes.push_back(s);
    if (sizes.empty()) sizes.push_back(std::min(img.Width, img.Height));

    HANDLE update = BeginUpdateResourceW(exe.wstring().c_str(), FALSE);
    if (!update) {
        whyNot = "BeginUpdateResource failed";
        return false;
    }

    // RT_ICON/RT_GROUP_ICON macros expand ANSI in a non-UNICODE build; spell
    // out the wide forms (ordinals 3 and 14) for the W-suffixed API.
    const WORD lang = MAKELANGID(LANG_NEUTRAL, SUBLANG_NEUTRAL);
    bool ok = true;

    // GRPICONDIR: 6-byte header (reserved, type=1, count) + 14-byte entries.
    std::vector<uint8_t> group(6, 0);
    group[2] = 1;
    group[4] = (uint8_t)sizes.size();
    for (size_t i = 0; i < sizes.size() && ok; ++i) {
        const int s = sizes[i];
        const std::vector<uint8_t> entryData =
            EncodeIcoBmp(DownscaleRgba(pixels, img.Width, img.Height, s), s);
        ok = UpdateResourceW(update, MAKEINTRESOURCEW(3),
                             MAKEINTRESOURCEW((WORD)(i + 1)), lang,
                             (void*)entryData.data(), (DWORD)entryData.size());
        GrpIconDirEntry e{};
        e.width      = (BYTE)(s >= 256 ? 0 : s);   // 0 encodes 256
        e.height     = (BYTE)(s >= 256 ? 0 : s);
        e.planes     = 1;
        e.bitCount   = 32;
        e.bytesInRes = (DWORD)entryData.size();
        e.id         = (WORD)(i + 1);
        const uint8_t* eb = reinterpret_cast<const uint8_t*>(&e);
        group.insert(group.end(), eb, eb + sizeof(e));
    }
    ok = ok && UpdateResourceW(update, MAKEINTRESOURCEW(14), MAKEINTRESOURCEW(1),
                               lang, group.data(), (DWORD)group.size());
    if (!EndUpdateResourceW(update, /*discard*/ !ok)) ok = false;
    if (!ok) whyNot = "UpdateResource failed";
    return ok;
}
#endif

// Source images the cooker can BCn-compress (mirror of TextureCooker's set).
bool IsCookableImage(const std::string& lowerExt)
{
    return lowerExt == ".png" || lowerExt == ".tga" ||
           lowerExt == ".jpg" || lowerExt == ".jpeg";
}

fs::path SettingsPath()
{
    return AssetPaths::ProjectRoot() / "ProjectSettings" / "Package.json";
}

} // namespace

PackagePanel::PackagePanel()
{
    m_Open = false;   // opt-in via the View menu
    LoadSettings();
}

PackagePanel::~PackagePanel()
{
    SaveSettings();
    // Blocks until the job's current stage returns (a mid-flight cmake build
    // can't be interrupted) — the worker captures `this`, so it must not
    // outlive the panel.
    m_Cancel = true;
    if (m_Worker.joinable()) m_Worker.join();
}

void PackagePanel::LoadSettings()
{
    std::ifstream f(SettingsPath());
    if (!f.is_open()) return;
    try {
        const nlohmann::json j = nlohmann::json::parse(f);
        m_Scenes = j.value("scenes", m_Scenes);
        const std::string out   = j.value("outputDir", std::string());
        const std::string title = j.value("title", std::string(m_Title));
        std::snprintf(m_OutputDir, sizeof(m_OutputDir), "%s", out.c_str());
        std::snprintf(m_Title,     sizeof(m_Title),     "%s", title.c_str());
        m_Width         = j.value("width",  m_Width);
        m_Height        = j.value("height", m_Height);
        m_IconPath      = j.value("icon",   m_IconPath);
        m_DebugBuild    = j.value("debugBuild",    m_DebugBuild);
        m_FullAssetCopy = j.value("fullAssetCopy", m_FullAssetCopy);
    } catch (const std::exception& e) {
        spdlog::warn("[Packager] failed to parse '{}': {}", SettingsPath().string(), e.what());
    }
}

void PackagePanel::SaveSettings()
{
    // Nothing configured yet — don't create ProjectSettings/ for an untouched panel.
    if (m_Scenes.empty() && m_OutputDir[0] == '\0' && !fs::exists(SettingsPath())) return;

    nlohmann::json j;
    j["scenes"]        = m_Scenes;
    j["outputDir"]     = m_OutputDir;
    j["title"]         = m_Title;
    j["width"]         = m_Width;
    j["height"]        = m_Height;
    j["icon"]          = m_IconPath;
    j["debugBuild"]    = m_DebugBuild;
    j["fullAssetCopy"] = m_FullAssetCopy;

    std::error_code ec;
    fs::create_directories(SettingsPath().parent_path(), ec);
    std::ofstream f(SettingsPath());
    if (f.is_open()) f << j.dump(2) << "\n";
    else spdlog::warn("[Packager] failed to write '{}'", SettingsPath().string());
}

void PackagePanel::Log(std::string line)
{
    std::lock_guard<std::mutex> lock(m_LogMutex);
    m_PendingLog.push_back(std::move(line));
}

void PackagePanel::AddScene(const std::string& path)
{
    if (path.empty()) return;
    const std::string portable = AssetPaths::ToPortable(path);
    if (LowerExtOf(portable) != ".scene") {
        Log("[warn] not a .scene file: " + path);
        return;
    }
    if (!StartsWithAssets(portable)) {
        Log("[warn] scene must live inside the project's Assets/ folder: " + path);
        return;
    }
    const std::string key = AssetPaths::LowerGeneric(portable);
    for (const std::string& s : m_Scenes)
        if (AssetPaths::LowerGeneric(s) == key) return;   // already listed
    m_Scenes.push_back(portable);
}

// ---------------------------------------------------------------------------
// Background job
// ---------------------------------------------------------------------------

bool PackagePanel::RunBuild(bool debugBuild)
{
    const std::string cmd = std::string("cmake --build \"") + DIAMOND_BUILD_DIR +
        "\" --config " + (debugBuild ? "Debug" : "Release") + " --target Runtime 2>&1";
    Log("[build] " + cmd);

    FILE* pipe = DIAMOND_POPEN(cmd.c_str(), "r");
    if (!pipe) {
        Log("[error] failed to launch cmake — is it on PATH?");
        return false;
    }
    char line[1024];
    while (std::fgets(line, sizeof(line), pipe)) {
        std::string s(line);
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
        if (!s.empty()) Log("    " + s);
    }
    const int rc = DIAMOND_PCLOSE(pipe);
    if (rc != 0) {
        Log("[error] cmake --build exited with code " + std::to_string(rc));
        return false;
    }
    return true;
}

void PackagePanel::StartJob()
{
    if (m_Worker.joinable()) m_Worker.join();   // reap the previous run

    Config cfg;
    cfg.scenes        = m_Scenes;
    cfg.outputDir     = m_OutputDir;
    cfg.title         = m_Title;
    cfg.iconPath      = m_IconPath;
    cfg.width         = std::max(64, m_Width);
    cfg.height        = std::max(64, m_Height);
    cfg.debugBuild    = m_DebugBuild;
    cfg.fullAssetCopy = m_FullAssetCopy;

    if (cfg.scenes.empty())    { Log("[error] add at least one scene — entry 0 boots the game"); return; }
    if (cfg.outputDir.empty()) { Log("[error] choose an output folder"); return; }

    // Refuse an output folder inside Assets/: the full-copy walk would
    // recursively copy the package into itself.
    const std::string outLow    = AssetPaths::LowerGeneric(fs::path(cfg.outputDir).lexically_normal().string());
    const std::string assetsLow = AssetPaths::LowerGeneric((AssetPaths::ProjectRoot() / "Assets").lexically_normal().string());
    if (outLow.rfind(assetsLow, 0) == 0) {
        Log("[error] output folder must be outside the project's Assets/ directory");
        return;
    }

    SaveSettings();   // a validated config is worth keeping even if the editor dies mid-job

    m_LogLines.clear();
    {
        std::lock_guard<std::mutex> lock(m_LogMutex);
        m_PendingLog.clear();
    }
    m_Cancel      = false;
    m_FilesDone   = 0;
    m_FilesTotal  = 0;
    m_BytesCopied = 0;
    m_Running     = true;
    m_Stage       = (int)Stage::Building;
    m_Worker = std::thread([this, cfg = std::move(cfg)]() mutable { RunJob(std::move(cfg)); });
}

void PackagePanel::RunJob(Config cfg)
{
    const fs::path srcRoot = AssetPaths::ProjectRoot();
    const fs::path outRoot = fs::path(cfg.outputDir);
    auto finish = [&](Stage s) { m_Stage = (int)s; m_Running = false; };

    // ---- 1. build the Runtime target -------------------------------------
    if (!RunBuild(cfg.debugBuild)) return finish(Stage::Failed);
    if (m_Cancel)                  return finish(Stage::Cancelled);

    // VS multi-config puts the exe under a per-config subdir; single-config
    // generators (Ninja) put it flat.
    const char* config = cfg.debugBuild ? "Debug" : "Release";
    fs::path exe = fs::path(DIAMOND_BUILD_DIR) / "Runtime" / config / "Runtime.exe";
    std::error_code ec;
    if (!fs::exists(exe, ec)) exe = fs::path(DIAMOND_BUILD_DIR) / "Runtime" / "Runtime.exe";
    if (!fs::exists(exe, ec)) {
        Log("[error] built, but Runtime.exe not found under " + (fs::path(DIAMOND_BUILD_DIR) / "Runtime").string());
        return finish(Stage::Failed);
    }

    // ---- 2. collect the scene list's dependency closure ------------------
    m_Stage = (int)Stage::Collecting;
    Log("[collect] walking dependencies of " + std::to_string(cfg.scenes.size()) + " scene(s)");

    std::unordered_set<std::string> listedScenes;
    for (const std::string& s : cfg.scenes) listedScenes.insert(AssetPaths::LowerGeneric(s));

    std::vector<std::string>        files;
    std::unordered_set<std::string> seen;
    std::deque<std::string>         queue(cfg.scenes.begin(), cfg.scenes.end());
    queue.push_back(kDefaultIblHdr);
    if (!cfg.iconPath.empty() && StartsWithAssets(cfg.iconPath))
        queue.push_back(cfg.iconPath);

    bool bootSceneMissing = false;
    while (!queue.empty()) {
        if (m_Cancel) return finish(Stage::Cancelled);
        const std::string ref = std::move(queue.front());
        queue.pop_front();

        const std::string key = AssetPaths::LowerGeneric(ref);
        if (!seen.insert(key).second) continue;
        if (key.rfind("assets/", 0) != 0) {
            Log("[warn] external reference not packaged: " + ref);
            continue;
        }

        const fs::path abs = srcRoot / ref;
        if (!fs::exists(abs, ec)) {
            Log("[warn] MISSING: " + ref);
            if (key == AssetPaths::LowerGeneric(cfg.scenes[0])) bootSceneMissing = true;
            continue;
        }
        files.push_back(ref);

        const std::string ext         = LowerExtOf(ref);
        const std::string dirPortable = fs::path(ref).parent_path().generic_string();

        // Reachability lint (advisory by design: scripts can reference scenes
        // in C++, unscannably — the explicit list stays the source of truth).
        if (ext == ".scene" && !listedScenes.count(key))
            Log("[warn] '" + ref + "' is referenced by walked assets but not in the scene list — "
                "the runtime only guarantees listed scenes are loadable");

        std::vector<std::string> refs;
        if (IsTextAsset(ext)) {
            ScanAssetRefs(ReadFileText(abs), refs);
        } else if (ext == ".obj") {
            const std::string mtl = (fs::path(ref).parent_path() / fs::path(ref).stem()).generic_string() + ".mtl";
            if (fs::exists(srcRoot / mtl, ec)) refs.push_back(mtl);
        } else if (ext == ".mtl") {
            ScanMtlRefs(abs, dirPortable, refs);
        } else if (ext == ".gltf") {
            ScanGltfRefs(abs, dirPortable, refs);
        }
        for (std::string& r : refs) queue.push_back(std::move(r));
    }

    if (bootSceneMissing) {
        Log("[error] boot scene '" + cfg.scenes[0] + "' does not exist — nothing to package");
        return finish(Stage::Failed);
    }
    Log("[collect] " + std::to_string(files.size()) + " asset file(s)");

    // ---- 3. cook textures --------------------------------------------------
    // Package time is the last chance to close cook gaps: any collected source
    // image whose Assets/Cache .dds is missing or stale gets cooked here, so
    // the package ships BCn everywhere without a manual pre-pass.
    m_Stage = (int)Stage::Cooking;
#ifndef DIAMOND_TEXCONV
    Log("[warn] texconv not configured (see Sandbox/CMakeLists.txt) — skipping texture "
        "cooking; textures without a cooked .dds will load raw in the package");
#else
    if (cfg.fullAssetCopy) {
        // Full-copy ships the whole Cache mirror, so freshen the whole tree.
        Log("[cook] cooking all of Assets/ (full-copy mode) — this can take a while");
        const int cooked = Diamond::TextureCooker::CookAll();
        Log("[cook] " + std::to_string(cooked) + " texture(s) (re)cooked");
    } else {
        std::vector<const std::string*> images;
        for (const std::string& ref : files)
            if (IsCookableImage(LowerExtOf(ref))) images.push_back(&ref);
        m_FilesDone  = 0;
        m_FilesTotal = (int)images.size();
        int cooked = 0, uncooked = 0;
        for (const std::string* ref : images) {
            if (m_Cancel) return finish(Stage::Cancelled);
            if (Diamond::TextureCooker::CookOne((srcRoot / *ref).string())) {
                ++cooked;
                Log("[cook] " + *ref);
            }
            const std::string c = Diamond::DDSLoader::CookedPathFor(*ref);
            if (c.empty() || !fs::exists(srcRoot / c, ec)) ++uncooked;
            ++m_FilesDone;
        }
        Log("[cook] " + std::to_string(cooked) + " (re)cooked, " +
            std::to_string((int)images.size() - cooked) + " already fresh");
        if (uncooked > 0)
            Log("[warn] " + std::to_string(uncooked) + " texture(s) still have no cooked "
                ".dds (cook failed — see console) — they will load raw in the package");
    }
#endif
    if (m_Cancel) return finish(Stage::Cancelled);

    // ---- 4. copy assets ---------------------------------------------------
    m_Stage = (int)Stage::Copying;
    m_FilesDone = 0;
    fs::create_directories(outRoot, ec);
    if (fs::exists(outRoot) && !fs::is_empty(outRoot, ec))
        Log("[note] output folder is not empty — files are overwritten in place, stale files are not removed");

    int copyErrors = 0;
    auto copyRel = [&](const std::string& rel) {
        const fs::path src = srcRoot / rel;
        const fs::path dst = outRoot / rel;
        std::error_code cec;
        fs::create_directories(dst.parent_path(), cec);
        if (!fs::copy_file(src, dst, fs::copy_options::overwrite_existing, cec)) {
            Log("[error] copy failed: " + rel + " (" + cec.message() + ")");
            ++copyErrors;
            return;
        }
        m_BytesCopied += fs::file_size(src, cec);
    };

    if (cfg.fullAssetCopy) {
        const fs::path assetsRoot = srcRoot / "Assets";
        int total = 0;
        for (const auto& e : fs::recursive_directory_iterator(assetsRoot, ec))
            if (e.is_regular_file()) ++total;
        m_FilesTotal = total;
        for (const auto& e : fs::recursive_directory_iterator(assetsRoot, ec)) {
            if (m_Cancel) return finish(Stage::Cancelled);
            if (!e.is_regular_file()) continue;
            copyRel(fs::relative(e.path(), srcRoot, ec).generic_string());
            ++m_FilesDone;
        }
    } else {
        m_FilesTotal = (int)files.size();
        for (const std::string& ref : files) {
            if (m_Cancel) return finish(Stage::Cancelled);
            copyRel(ref);
            // Cooked BCn counterpart (Assets/X.png -> Assets/Cache/X.dds):
            // the runtime's cooked lookup follows the package root, so ship
            // the cache mirror wherever one exists.
            const std::string cooked = Diamond::DDSLoader::CookedPathFor(ref);
            if (!cooked.empty() && fs::exists(srcRoot / cooked, ec))
                copyRel(fs::path(cooked).generic_string());
            ++m_FilesDone;
        }
    }

    // ---- 5. exe + shaders + boot.json -------------------------------------
    m_Stage = (int)Stage::Finalizing;
    if (m_Cancel) return finish(Stage::Cancelled);

    if (!fs::copy_file(exe, outRoot / exe.filename(), fs::copy_options::overwrite_existing, ec)) {
        Log("[error] failed to copy " + exe.string() + " (" + ec.message() + ")");
        return finish(Stage::Failed);
    }
    m_BytesCopied += fs::file_size(exe, ec);

    const fs::path shaderSrc = fs::path(DIAMOND_BUILD_DIR) / "shaders";
    fs::create_directories(outRoot / "shaders", ec);
    int shaderCount = 0;
    for (const auto& e : fs::directory_iterator(shaderSrc, ec)) {
        if (!e.is_regular_file() || e.path().extension() != ".spv") continue;
        std::error_code cec;
        if (fs::copy_file(e.path(), outRoot / "shaders" / e.path().filename(),
                          fs::copy_options::overwrite_existing, cec)) {
            m_BytesCopied += fs::file_size(e.path(), cec);
            ++shaderCount;
        } else {
            Log("[error] shader copy failed: " + e.path().filename().string());
            ++copyErrors;
        }
    }
    if (shaderCount == 0) {
        Log("[error] no .spv shaders found in " + shaderSrc.string());
        return finish(Stage::Failed);
    }

    // External icon (outside Assets/): pull it into the package under a
    // portable name so the runtime's AssetPaths::Resolve finds it.
    std::string iconRef = cfg.iconPath;
    if (!iconRef.empty() && !StartsWithAssets(iconRef)) {
        const std::string dest = "Assets/BootIcon" + fs::path(iconRef).extension().string();
        std::error_code cec;
        fs::create_directories(outRoot / "Assets", cec);
        if (fs::copy_file(fs::path(iconRef), outRoot / dest,
                          fs::copy_options::overwrite_existing, cec)) {
            iconRef = dest;
        } else {
            Log("[warn] icon copy failed (" + cec.message() + ") — packaging without an icon");
            iconRef.clear();
        }
    }

#ifdef _WIN32
    // Explorer icon on the packaged exe (the copy, never the build-tree one).
    // The icon file is guaranteed in the package by now: the dep walk queued a
    // portable icon, and the external branch above copied the rest.
    if (!iconRef.empty()) {
        std::string whyNot;
        if (EmbedExeIcon(outRoot / exe.filename(), outRoot / iconRef, whyNot))
            Log("[icon] Explorer icon embedded into " + exe.filename().string() +
                " at 16/32/48/256 px (Explorer caches icons — a stale one may "
                "linger until refresh)");
        else
            Log("[warn] Explorer icon not embedded (" + whyNot + ") — the in-game "
                "window icon is unaffected");
    }
#endif

    nlohmann::json boot;
    boot["scenes"] = cfg.scenes;
    boot["title"]  = cfg.title;
    boot["width"]  = cfg.width;
    boot["height"] = cfg.height;
    if (!iconRef.empty()) boot["icon"] = iconRef;
    std::ofstream bootFile(outRoot / "boot.json");
    if (!bootFile.is_open()) {
        Log("[error] failed to write boot.json");
        return finish(Stage::Failed);
    }
    bootFile << boot.dump(2) << "\n";
    bootFile.close();

    if (copyErrors > 0) {
        Log("[error] finished with " + std::to_string(copyErrors) + " copy error(s)");
        return finish(Stage::Failed);
    }

    Log("[done] " + std::to_string(m_FilesDone.load()) + " asset file(s) + " +
        std::to_string(shaderCount) + " shaders, " + HumanSize(m_BytesCopied) +
        " -> " + cfg.outputDir);
    if (cfg.debugBuild)
        Log("[note] Debug builds link the debug CRT, which is NOT redistributable — use Release for anything you share");
    else
        Log("[note] target machines need the Microsoft Visual C++ x64 redistributable (Runtime.exe links /MD)");
    finish(Stage::Done);
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------

void PackagePanel::DrawSceneList()
{
    ImGui::TextDisabled("Entry 0 boots the game. Drag .scene files from the Content Browser onto the button below.");

    int removeAt = -1, moveFrom = -1, moveTo = -1;
    for (int i = 0; i < (int)m_Scenes.size(); ++i) {
        ImGui::PushID(i);
        if (ImGui::ArrowButton("up", ImGuiDir_Up)   && i > 0)                        { moveFrom = i; moveTo = i - 1; }
        ImGui::SameLine();
        if (ImGui::ArrowButton("dn", ImGuiDir_Down) && i + 1 < (int)m_Scenes.size()) { moveFrom = i; moveTo = i + 1; }
        ImGui::SameLine();
        if (ImGui::SmallButton("X")) removeAt = i;
        ImGui::SameLine();
        if (i == 0) ImGui::Text("%d  %s  (boot)", i, m_Scenes[i].c_str());
        else        ImGui::Text("%d  %s",         i, m_Scenes[i].c_str());
        ImGui::PopID();
    }
    if (moveFrom >= 0) std::swap(m_Scenes[moveFrom], m_Scenes[moveTo]);
    if (removeAt >= 0) m_Scenes.erase(m_Scenes.begin() + removeAt);

    if (ImGui::Button("Add Scene...", ImVec2(-FLT_MIN, 0.0f)))
        AddScene(OpenFileDialogFiltered("Scene Files (*.scene)\0*.scene\0All Files\0*.*\0",
                                        ASSETS_DIR "/Scenes"));
    AddScene(AcceptContentDrop());
}

void PackagePanel::DrawStatusRow()
{
    const Stage stage = (Stage)m_Stage.load();

    if (m_Running) {
        const char* label = "";
        switch (stage) {
            case Stage::Building:   label = "Building Runtime (cmake)"; break;
            case Stage::Collecting: label = "Collecting dependencies";  break;
            case Stage::Cooking:    label = "Cooking textures (texconv)"; break;
            case Stage::Copying:    label = "Copying assets";           break;
            case Stage::Finalizing: label = "Writing exe / shaders / boot.json"; break;
            default: break;
        }
        // Animated ellipsis so the indeterminate stages visibly aren't hung.
        const int dots = 1 + (int)(ImGui::GetTime() * 2.0) % 3;
        ImGui::Text("%s%.*s", label, dots, "...");
        if ((stage == Stage::Copying || stage == Stage::Cooking) && m_FilesTotal > 0) {
            char overlay[64];
            std::snprintf(overlay, sizeof(overlay), "%d / %d files (%s)",
                          m_FilesDone.load(), m_FilesTotal.load(),
                          HumanSize(m_BytesCopied).c_str());
            ImGui::ProgressBar((float)m_FilesDone / (float)m_FilesTotal,
                               ImVec2(-FLT_MIN, 0.0f), overlay);
        }
        if (!m_Cancel) {
            if (ImGui::Button("Cancel"))
                m_Cancel = true;   // honored between files/stages; can't interrupt cmake mid-build
        } else {
            ImGui::TextDisabled("Cancelling at the next safe point...");
        }
        return;
    }

    if (ImGui::Button("Package", ImVec2(120.0f, 0.0f)))
        StartJob();

    switch (stage) {
        case Stage::Done:
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Package complete");
#ifdef _WIN32
            ImGui::SameLine();
            if (ImGui::Button("Open Folder"))
                ShellExecuteA(nullptr, "open", m_OutputDir, nullptr, nullptr, SW_SHOWNORMAL);
#endif
            break;
        case Stage::Failed:
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Packaging failed — see log");
            break;
        case Stage::Cancelled:
            ImGui::SameLine();
            ImGui::TextDisabled("Cancelled");
            break;
        default:
            break;
    }
}

void PackagePanel::OnImGuiRender()
{
    // Reap a finished worker and drain its log even while the window is
    // closed, so nothing accumulates or dangles behind a hidden panel.
    if (!m_Running && m_Worker.joinable()) m_Worker.join();
    {
        std::lock_guard<std::mutex> lock(m_LogMutex);
        for (std::string& l : m_PendingLog) m_LogLines.push_back(std::move(l));
        m_PendingLog.clear();
    }

    if (!m_Open) return;
    if (!ImGui::Begin("Packager", &m_Open)) { ImGui::End(); return; }

    ImGui::BeginDisabled(m_Running);

    ImGui::TextDisabled("SCENES");
    ImGui::Separator();
    DrawSceneList();

    ImGui::Spacing();
    ImGui::TextDisabled("GAME WINDOW");
    ImGui::Separator();
    ImGui::SetNextItemWidth(300.0f);
    ImGui::InputText("Title", m_Title, sizeof(m_Title));
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputInt("##width", &m_Width, 0);
    ImGui::SameLine(); ImGui::Text("x"); ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputInt("Resolution", &m_Height, 0);

    ImGui::Text("Icon: %s", m_IconPath.empty() ? "(none)" : m_IconPath.c_str());
    {
        const std::string dropped = AcceptContentDrop();
        if (!dropped.empty()) {
            if (LowerExtOf(dropped) == ".png") m_IconPath = AssetPaths::ToPortable(dropped);
            else                               Log("[warn] window icon must be a .png");
        }
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Browse...##icon")) {
        const std::string p = OpenFileDialogFiltered("PNG Image (*.png)\0*.png\0", ASSETS_DIR);
        if (!p.empty()) m_IconPath = AssetPaths::ToPortable(p);
    }
    if (!m_IconPath.empty()) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear##icon")) m_IconPath.clear();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("BUILD");
    ImGui::Separator();
    int build = m_DebugBuild ? 1 : 0;
    ImGui::RadioButton("Release", &build, 0); ImGui::SameLine();
    ImGui::RadioButton("Debug", &build, 1);
    m_DebugBuild = (build == 1);
    if (m_DebugBuild)
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                           "Debug links the debug CRT — not redistributable, dev machines only");
    ImGui::Checkbox("Copy entire Assets folder", &m_FullAssetCopy);
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Default: only the listed scenes' dependency closure is copied.\n"
                          "This copies all of Assets/ instead — multiple GB.");

    ImGui::Spacing();
    ImGui::TextDisabled("OUTPUT");
    ImGui::Separator();
    ImGui::SetNextItemWidth(-160.0f);
    ImGui::InputText("##outdir", m_OutputDir, sizeof(m_OutputDir));
    ImGui::SameLine();
    if (ImGui::Button("Browse...##outdir")) {
        const std::string dir = PickFolderDialog();
        if (!dir.empty()) std::snprintf(m_OutputDir, sizeof(m_OutputDir), "%s", dir.c_str());
    }
    ImGui::SameLine();
    ImGui::Text("Folder");

    ImGui::EndDisabled();

    ImGui::Spacing();
    DrawStatusRow();

    ImGui::Spacing();
    ImGui::TextDisabled("LOG");
    ImGui::Separator();
    ImGui::BeginChild("PackageLog", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    ImGuiListClipper clipper;
    clipper.Begin((int)m_LogLines.size());
    while (clipper.Step())
        for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
            const std::string& l = m_LogLines[i];
            ImVec4 color(0.85f, 0.85f, 0.85f, 1.0f);
            if (l.rfind("[error]", 0) == 0)      color = ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
            else if (l.rfind("[warn]", 0) == 0)  color = ImVec4(1.0f, 0.8f, 0.0f, 1.0f);
            else if (l.rfind("[done]", 0) == 0)  color = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
            ImGui::TextColored(color, "%s", l.c_str());
        }
    if (m_Running && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
        ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();

    ImGui::End();
}
