#pragma once

#include <spdlog/spdlog.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

// Shared helpers for the Vulkan render passes (Platform/Vulkan/Passes/*). Kept
// header-only and tiny — a place for utilities every pass needs without dragging
// in the device/backend types.
namespace Diamond {

// Reads a compiled SPIR-V module (.spv) emitted by the build's glslang step. 'dir'
// is the baked DIAMOND_VULKAN_SHADER_DIR; 'name' is e.g. "tonemap.frag.spv".
// A missing module is a build-wiring bug, not a recoverable runtime state — abort.
inline std::vector<uint32_t> LoadSpirv(const std::string& dir, const std::string& name)
{
    const std::string path = dir + "/" + name;
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        spdlog::critical("[Vulkan] failed to open SPIR-V '{}'", path);
        std::abort();
    }
    const std::streamsize bytes = file.tellg();
    std::vector<uint32_t> words(static_cast<size_t>(bytes) / sizeof(uint32_t));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(words.data()), bytes);
    return words;
}

} // namespace Diamond
