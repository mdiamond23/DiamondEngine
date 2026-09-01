#include "Assets/DDSLoader.h"

#include <spdlog/spdlog.h>

#include <cstring>
#include <filesystem>
#include <fstream>

namespace Diamond {

namespace {

// The subset of the DDS file layout we consume (docs.microsoft.com/dds). All
// fields little-endian; the structs mirror the on-disk layout exactly so a
// memcpy from the file bytes fills them.
struct DDSPixelFormat {
    uint32_t size;          // 32
    uint32_t flags;         // DDPF_FOURCC = 0x4
    uint32_t fourCC;        // 'DX10' routes format info to the DX10 header
    uint32_t rgbBitCount;
    uint32_t rMask, gMask, bMask, aMask;
};

struct DDSHeader {
    uint32_t       size;    // 124
    uint32_t       flags;
    uint32_t       height;
    uint32_t       width;
    uint32_t       pitchOrLinearSize;
    uint32_t       depth;
    uint32_t       mipMapCount;
    uint32_t       reserved1[11];
    DDSPixelFormat ddspf;
    uint32_t       caps, caps2, caps3, caps4;
    uint32_t       reserved2;
};

struct DDSHeaderDXT10 {
    uint32_t dxgiFormat;
    uint32_t resourceDimension;   // 3 = TEXTURE2D
    uint32_t miscFlag;
    uint32_t arraySize;
    uint32_t miscFlags2;
};

constexpr uint32_t kDDSMagic          = 0x20534444;   // "DDS "
constexpr uint32_t kFourCCDX10        = 0x30315844;   // "DX10"
constexpr uint32_t kDDPFFourCC        = 0x4;
constexpr uint32_t kDXGIFormatBC4     = 80;           // DXGI_FORMAT_BC4_UNORM
constexpr uint32_t kDXGIFormatBC5     = 83;           // DXGI_FORMAT_BC5_UNORM
constexpr uint32_t kDXGIFormatBC7     = 98;           // DXGI_FORMAT_BC7_UNORM
constexpr uint32_t kResourceTexture2D = 3;

// The only dxgiFormats texconv -dx10 ever hands us (see TextureCooker.cpp's
// usage->format mapping) — anything else is rejected rather than half-supported.
RHIFormat FormatFromDxgi(uint32_t dxgiFormat)
{
    switch (dxgiFormat) {
        case kDXGIFormatBC7: return RHIFormat::BC7;
        case kDXGIFormatBC5: return RHIFormat::BC5;
        case kDXGIFormatBC4: return RHIFormat::BC4;
        default:             return RHIFormat::Undefined;
    }
}

std::filesystem::file_time_type FileTime(const std::string& path)
{
    std::error_code ec;
    auto t = std::filesystem::last_write_time(path, ec);
    return ec ? std::filesystem::file_time_type::min() : t;
}

} // namespace

DDSData DDSLoader::Load(const std::string& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return {};

    const size_t fileSize = static_cast<size_t>(file.tellg());
    constexpr size_t kHeaderBytes = 4 + sizeof(DDSHeader) + sizeof(DDSHeaderDXT10);
    if (fileSize <= kHeaderBytes) {
        spdlog::warn("[DDS] '{}' too small to be a cooked DDS", path);
        return {};
    }

    std::vector<uint8_t> bytes(fileSize);
    file.seekg(0);
    file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(fileSize));
    if (!file) return {};

    uint32_t magic = 0;
    std::memcpy(&magic, bytes.data(), 4);
    DDSHeader header{};
    std::memcpy(&header, bytes.data() + 4, sizeof(DDSHeader));

    if (magic != kDDSMagic || header.size != sizeof(DDSHeader)
        || header.ddspf.size != sizeof(DDSPixelFormat)) {
        spdlog::warn("[DDS] '{}' has a malformed header", path);
        return {};
    }
    // v1 only reads texconv -dx10 output; legacy-header files (fourCC 'DXT1'
    // etc.) are rejected rather than half-supported.
    if (!(header.ddspf.flags & kDDPFFourCC) || header.ddspf.fourCC != kFourCCDX10) {
        spdlog::warn("[DDS] '{}' lacks the DX10 header (re-cook with texconv -dx10)", path);
        return {};
    }

    DDSHeaderDXT10 dx10{};
    std::memcpy(&dx10, bytes.data() + 4 + sizeof(DDSHeader), sizeof(DDSHeaderDXT10));
    const RHIFormat format = FormatFromDxgi(dx10.dxgiFormat);
    if (format == RHIFormat::Undefined || dx10.resourceDimension != kResourceTexture2D
        || dx10.arraySize != 1) {
        spdlog::warn("[DDS] '{}' unsupported (dxgiFormat={}, dim={}, arraySize={}) — v1 reads "
                     "BC7/BC5/BC4_UNORM 2D only", path, dx10.dxgiFormat, dx10.resourceDimension, dx10.arraySize);
        return {};
    }

    DDSData dds;
    dds.Width    = header.width;
    dds.Height   = header.height;
    dds.MipCount = header.mipMapCount > 0 ? header.mipMapCount : 1;
    dds.Format   = format;
    if (dds.Width == 0 || dds.Height == 0) return {};

    uint64_t needed = 0;
    for (uint32_t mip = 0; mip < dds.MipCount; ++mip) {
        const uint32_t w = dds.Width  >> mip ? dds.Width  >> mip : 1;
        const uint32_t h = dds.Height >> mip ? dds.Height >> mip : 1;
        needed += RHIFormatLevelSize(dds.Format, w, h);
    }
    const size_t available = fileSize - kHeaderBytes;
    if (available < needed) {
        spdlog::warn("[DDS] '{}' truncated: {} payload bytes, need {}", path, available, needed);
        return {};
    }
    if (available > needed)
        spdlog::warn("[DDS] '{}' has {} trailing bytes past the mip chain (ignored)",
                     path, available - needed);

    dds.Payload.assign(bytes.begin() + kHeaderBytes,
                       bytes.begin() + kHeaderBytes + static_cast<ptrdiff_t>(needed));
    return dds;
}

std::string DDSLoader::CookedPathFor(const std::string& sourcePath)
{
    namespace fs = std::filesystem;
    const fs::path src(sourcePath);

    // Split into components and find the last "Assets" directory; the cooked
    // mirror inserts "Cache" right after it, preserving the subtree.
    std::vector<fs::path> parts(src.begin(), src.end());
    size_t assetsIdx = parts.size();
    for (size_t i = parts.size(); i-- > 0; ) {
        if (parts[i] == "Assets") { assetsIdx = i; break; }
    }
    if (assetsIdx == parts.size() || assetsIdx + 1 >= parts.size()) return {};

    fs::path cooked;
    for (size_t i = 0; i <= assetsIdx; ++i) cooked /= parts[i];
    cooked /= "Cache";
    for (size_t i = assetsIdx + 1; i < parts.size(); ++i) cooked /= parts[i];
    cooked.replace_extension(".dds");
    return cooked.string();
}

std::string DDSLoader::CookedChannelPathFor(const std::string& sourcePath, uint32_t channel)
{
    if (channel > 3) return {};
    std::filesystem::path cooked(CookedPathFor(sourcePath));
    if (cooked.empty()) return {};

    static constexpr char kNames[] = { 'r', 'g', 'b', 'a' };
    const std::string stem = cooked.stem().string() + "." + kNames[channel];
    cooked.replace_filename(stem + ".dds");
    return cooked.string();
}

DDSData DDSLoader::LoadCookedFor(const std::string& sourcePath)
{
    const std::string cooked = CookedPathFor(sourcePath);
    if (cooked.empty()) return {};

    std::error_code ec;
    if (!std::filesystem::exists(cooked, ec) || ec) return {};
    if (FileTime(cooked) < FileTime(sourcePath)) return {};   // stale — source wins until re-cooked

    return Load(cooked);
}

DDSData DDSLoader::LoadCookedChannelFor(const std::string& sourcePath, uint32_t channel)
{
    const std::string cooked = CookedChannelPathFor(sourcePath, channel);
    if (cooked.empty()) return {};

    std::error_code ec;
    if (!std::filesystem::exists(cooked, ec) || ec) return {};
    if (FileTime(cooked) < FileTime(sourcePath)) return {};

    return Load(cooked);
}

} // namespace Diamond
