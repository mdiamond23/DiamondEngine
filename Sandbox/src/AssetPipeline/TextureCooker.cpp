#include "AssetPipeline/TextureCooker.h"

#include <Assets/DDSLoader.h>
#include <Assets/ImageLoader.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <stb_image.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <vector>

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

struct SemanticUse {
    bool  hasFull = false;
    Usage full    = Usage::Color;
    bool  channels[4]{};
};

std::string Normalized(const std::filesystem::path& path)
{
    std::error_code ec;
    auto canonical = std::filesystem::weakly_canonical(path, ec);
    return (ec ? path : canonical).string();
}

// Resolve a glTF texture index through textures[index].source to its external
// image. Embedded/data URI images cannot be cooked into the project cache.
std::string ImageForTexture(const nlohmann::json& gltf, int textureIndex,
                            const std::filesystem::path& baseDir)
{
    if (textureIndex < 0 || !gltf.contains("textures") || !gltf["textures"].is_array()
        || textureIndex >= static_cast<int>(gltf["textures"].size())) return {};
    const auto& texture = gltf["textures"][textureIndex];
    if (!texture.contains("source") || !texture["source"].is_number_integer()) return {};

    const int imageIndex = texture["source"].get<int>();
    if (imageIndex < 0 || !gltf.contains("images") || !gltf["images"].is_array()
        || imageIndex >= static_cast<int>(gltf["images"].size())) return {};
    const auto& image = gltf["images"][imageIndex];
    if (!image.contains("uri") || !image["uri"].is_string()) return {};
    const std::string uri = image["uri"].get<std::string>();
    if (uri.rfind("data:", 0) == 0) return {};
    return Normalized(baseDir / std::filesystem::path(uri));
}

void MarkFull(std::unordered_map<std::string, SemanticUse>& uses,
              const std::string& path, Usage usage)
{
    if (path.empty()) return;
    auto& use = uses[path];
    if (!use.hasFull) {
        use.hasFull = true;
        use.full = usage;
        return;
    }
    // BC7 is the safe representation if an unusual glTF uses one image as
    // both color and normal: it preserves every channel. Normal-only remains
    // BC5 for the smaller/faster representation.
    if (use.full != usage) use.full = Usage::Color;
}

void MarkChannel(std::unordered_map<std::string, SemanticUse>& uses,
                 const std::string& path, int channel)
{
    if (!path.empty() && channel >= 0 && channel < 4)
        uses[path].channels[channel] = true;
}

void ScanGltfUses(const std::filesystem::path& gltfPath,
                  std::unordered_map<std::string, SemanticUse>& uses)
{
    std::ifstream input(gltfPath);
    if (!input) return;

    nlohmann::json gltf;
    try { input >> gltf; }
    catch (const std::exception& e) {
        spdlog::warn("[TextureCooker] could not parse '{}': {}", gltfPath.string(), e.what());
        return;
    }
    if (!gltf.contains("materials") || !gltf["materials"].is_array()) return;

    const auto baseDir = gltfPath.parent_path();
    auto texturePath = [&](const nlohmann::json& info) -> std::string {
        if (!info.is_object() || !info.contains("index") || !info["index"].is_number_integer())
            return {};
        return ImageForTexture(gltf, info["index"].get<int>(), baseDir);
    };

    for (const auto& material : gltf["materials"]) {
        if (material.contains("pbrMetallicRoughness")) {
            const auto& pbr = material["pbrMetallicRoughness"];
            if (pbr.contains("baseColorTexture"))
                MarkFull(uses, texturePath(pbr["baseColorTexture"]), Usage::Color);
            if (pbr.contains("metallicRoughnessTexture")) {
                const std::string path = texturePath(pbr["metallicRoughnessTexture"]);
                MarkChannel(uses, path, 1); // roughness = G
                MarkChannel(uses, path, 2); // metallic  = B
            }
        }
        if (material.contains("normalTexture"))
            MarkFull(uses, texturePath(material["normalTexture"]), Usage::Normal);
        if (material.contains("occlusionTexture"))
            MarkChannel(uses, texturePath(material["occlusionTexture"]), 0); // AO = R
        if (material.contains("emissiveTexture"))
            MarkFull(uses, texturePath(material["emissiveTexture"]), Usage::Color);
    }
}

std::filesystem::file_time_type FileTime(const std::string& path)
{
    std::error_code ec;
    auto t = std::filesystem::last_write_time(path, ec);
    return ec ? std::filesystem::file_time_type::min() : t;
}

std::string TexconvPath()
{
#ifdef DIAMOND_TEXCONV
    if (std::filesystem::is_regular_file(DIAMOND_TEXCONV)) return DIAMOND_TEXCONV;
#endif

    // Runtime fallback keeps the editor action useful when texconv is dropped
    // into Tools after CMake was configured. ASSETS_DIR is absolute in normal
    // builds, so this is independent of the process working directory.
    const std::filesystem::path tools = std::filesystem::path(ASSETS_DIR).parent_path() / "Tools";
#ifdef _WIN32
    const std::filesystem::path bundled = tools / "texconv.exe";
#else
    const std::filesystem::path bundled = tools / "texconv";
#endif
    std::error_code ec;
    return std::filesystem::is_regular_file(bundled, ec) && !ec ? bundled.string() : std::string{};
}

bool CookAs(const std::string& sourcePath, Usage usage, int channel = -1)
{
    const std::string texconv = TexconvPath();
    if (texconv.empty()) return false;

    const std::string cookedPath = channel >= 0
        ? DDSLoader::CookedChannelPathFor(sourcePath, static_cast<uint32_t>(channel))
        : DDSLoader::CookedPathFor(sourcePath);
    if (cookedPath.empty()) return false;

    std::error_code ec;
    if (std::filesystem::exists(cookedPath, ec) && !ec
        && FileTime(cookedPath) >= FileTime(sourcePath)) return false;

    const std::filesystem::path outDir = std::filesystem::path(cookedPath).parent_path();
    std::filesystem::create_directories(outDir, ec);

    std::string extra;
    if (channel >= 0) {
        static constexpr char kNames[] = { 'r', 'g', 'b', 'a' };
        const char c = kNames[channel];
        extra = " -swizzle " + std::string(3, c) + "1 -sx ." + c;
    }
    // Match ImageLoader's runtime VRAM guard in the offline result. stbi_info
    // reads only the image header, so cooking never has to decode once merely
    // to discover the target size. Successive halving preserves aspect ratio
    // and exactly matches the uncooked fallback's dimensions.
    int width = 0, height = 0, components = 0;
    const uint32_t cap = ImageLoader::MaxDimension();
    if (cap > 0 && stbi_info(sourcePath.c_str(), &width, &height, &components)) {
        while ((width > static_cast<int>(cap) || height > static_cast<int>(cap))
               && (width > 1 || height > 1)) {
            width  = std::max(1, width  / 2);
            height = std::max(1, height / 2);
        }
        extra += " -w " + std::to_string(width) + " -h " + std::to_string(height);
    }
    const std::string cmd = "\"" + texconv + "\" -f "
        + std::string(channel >= 0 ? "BC4_UNORM" : TexconvFormatFor(usage))
        + extra + " -m 0 -dx10 -y -o \"" + outDir.string() + "\" \"" + sourcePath + "\"";
    const int rc = std::system(("\"" + cmd + "\"").c_str());
    if (rc != 0) {
        spdlog::warn("[TextureCooker] cook failed (exit {}): '{}'{}", rc, sourcePath,
                     channel >= 0 ? " derived channel" : "");
        return false;
    }
    if (!std::filesystem::exists(cookedPath, ec) || ec) {
        spdlog::warn("[TextureCooker] texconv succeeded but expected output '{}' is missing", cookedPath);
        return false;
    }
    spdlog::info("[TextureCooker] cooked '{}' -> '{}'", sourcePath, cookedPath);
    return true;
}

} // namespace

Usage ClassifyUsage(const std::string& sourcePath)
{
    const std::string stem = ToLower(std::filesystem::path(sourcePath).stem().string());

    if (Contains(stem, "normal") || Contains(stem, "_nrm") ||
        (stem.size() > 2 && stem.compare(stem.size() - 2, 2, "_n") == 0))
        return Usage::Normal;

    // Positive color evidence beats the mask keywords: names like
    // "metal_door_01_BaseColor" or "dirt_decal_mask_..._Opacity" are albedo
    // maps whose *material* name happens to contain a mask keyword. Cooking an
    // albedo as BC4 keeps only the red channel and drops alpha — solid-red
    // geometry in the scene. "opacity" is color because alpha only survives
    // in BC7.
    if (Contains(stem, "basecolor") || Contains(stem, "albedo") ||
        Contains(stem, "diffuse")   || Contains(stem, "_diff") ||
        Contains(stem, "opacity")   || Contains(stem, "emissive"))
        return Usage::Color;

    if (Contains(stem, "rough") || Contains(stem, "metal") ||
        Contains(stem, "occlusion") || Contains(stem, "_ao") || Contains(stem, "mask"))
        return Usage::Mask;

    return Usage::Color;
}

bool CookOne(const std::string& sourcePath)
{
    return CookAs(sourcePath, ClassifyUsage(sourcePath));
}

int CookAll(const std::string& root)
{
    const std::string dir = root.empty() ? std::string(ASSETS_DIR) : root;

    if (TexconvPath().empty()) {
        spdlog::warn("[TextureCooker] texconv not found - place texconv.exe in Tools/; cooking disabled");
        return 0;
    }

    std::error_code ec;
    if (!std::filesystem::exists(dir, ec) || ec) {
        spdlog::warn("[TextureCooker] CookAll: '{}' does not exist", dir);
        return 0;
    }

    std::vector<std::filesystem::path> sources;
    std::vector<std::filesystem::path> gltfs;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        if (IsSourceImage(entry.path())) sources.push_back(entry.path());
        else if (ToLower(entry.path().extension().string()) == ".gltf") gltfs.push_back(entry.path());
    }

    // Material semantics beat filename guesses and identify derived packed
    // channels. This also means MR/ORM images do not get a useless base DDS.
    std::unordered_map<std::string, SemanticUse> uses;
    for (const auto& gltf : gltfs) ScanGltfUses(gltf, uses);

    int cooked = 0;
    for (const auto& source : sources) {
        const std::string path = Normalized(source);
        auto it = uses.find(path);
        if (it == uses.end()) {
            if (CookOne(path)) ++cooked;
            continue;
        }
        const SemanticUse& use = it->second;
        if (use.hasFull && CookAs(path, use.full)) ++cooked;
        for (int channel = 0; channel < 4; ++channel)
            if (use.channels[channel] && CookAs(path, Usage::Mask, channel)) ++cooked;
    }
    spdlog::info("[TextureCooker] CookAll: {} texture(s) (re)cooked", cooked);
    return cooked;
}

} // namespace Diamond::TextureCooker
