#include "AssetPipeline/TextureCooker.h"

#include <Assets/DDSLoader.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>

#ifndef ASSETS_DIR
#define ASSETS_DIR "Assets"
#endif

namespace Diamond::TextureCooker {

namespace {

std::string ToLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool Contains(const std::string& haystack, const std::string& needle)
{
    return haystack.find(needle) != std::string::npos;
}

// texconv -f argument for each usage (design doc §4 "Format mapping" table):
// albedo/emissive get the full 3-channel BC7; normals are 2-channel BC5 (Z
// reconstructed in-shader — gbuffer.frag); masks are 1-channel BC4.
const char* TexconvFormatFor(Usage usage)
{
    switch (usage) {
        case Usage::Normal: return "BC5_UNORM";
        case Usage::Mask:   return "BC4_UNORM";
        default:            return "BC7_UNORM";
    }
}

bool IsSourceImage(const std::filesystem::path& p)
{
    const std::string ext = ToLower(p.extension().string());
    return ext == ".png" || ext == ".tga" || ext == ".jpg" || ext == ".jpeg";
}

std::filesystem::file_time_type FileTime(const std::string& path)
{
    std::error_code ec;
    auto t = std::filesystem::last_write_time(path, ec);
    return ec ? std::filesystem::file_time_type::min() : t;
}

} // namespace

Usage ClassifyUsage(const std::string& sourcePath)
{
    const std::string stem = ToLower(std::filesystem::path(sourcePath).stem().string());

    if (Contains(stem, "normal") || Contains(stem, "_nrm") ||
        (stem.size() > 2 && stem.compare(stem.size() - 2, 2, "_n") == 0))
        return Usage::Normal;

    if (Contains(stem, "rough") || Contains(stem, "metal") ||
        Contains(stem, "occlusion") || Contains(stem, "_ao") || Contains(stem, "mask"))
        return Usage::Mask;

    return Usage::Color;
}

bool CookOne(const std::string& sourcePath)
{
#ifndef DIAMOND_TEXCONV
    spdlog::warn("[TextureCooker] texconv not found at configure time — cooking disabled "
                 "(see Sandbox/CMakeLists.txt)");
    return false;
#else
    const Usage usage = ClassifyUsage(sourcePath);

    const std::string cookedPath = DDSLoader::CookedPathFor(sourcePath);
    if (cookedPath.empty()) return false;   // not under an Assets/ tree

    std::error_code ec;
    if (std::filesystem::exists(cookedPath, ec) && !ec &&
        FileTime(cookedPath) >= FileTime(sourcePath)) {
        return false;   // already fresh
    }

    const std::filesystem::path outDir = std::filesystem::path(cookedPath).parent_path();
    std::filesystem::create_directories(outDir, ec);

    const std::string cmd = "\"" DIAMOND_TEXCONV "\" -f " + std::string(TexconvFormatFor(usage)) +
                             " -m 0 -dx10 -y -o \"" + outDir.string() + "\" \"" + sourcePath + "\"";
    // cmd.exe mis-parses a command line starting with a quoted token unless the
    // whole thing is wrapped in one more pair of quotes (same fix RecompileSpirv
    // uses for glslangValidator).
    const int rc = std::system(("\"" + cmd + "\"").c_str());
    if (rc != 0) {
        spdlog::warn("[TextureCooker] cook failed (exit {}): '{}'", rc, sourcePath);
        return false;
    }
    spdlog::info("[TextureCooker] cooked '{}' -> '{}'", sourcePath, cookedPath);
    return true;
#endif
}

int CookAll(const std::string& root)
{
    const std::string dir = root.empty() ? std::string(ASSETS_DIR) : root;

    std::error_code ec;
    if (!std::filesystem::exists(dir, ec) || ec) {
        spdlog::warn("[TextureCooker] CookAll: '{}' does not exist", dir);
        return 0;
    }

    int cooked = 0;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        if (!IsSourceImage(entry.path())) continue;
        if (CookOne(entry.path().string())) ++cooked;
    }
    spdlog::info("[TextureCooker] CookAll: {} texture(s) (re)cooked", cooked);
    return cooked;
}

} // namespace Diamond::TextureCooker
