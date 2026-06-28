cmake_minimum_required(VERSION 3.24)
project(MyEngine LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Vulkan backend. Off on Apple (macOS stays on the OpenGL 4.1 path — no MoltenVK),
# on elsewhere so we can target a clean Vulkan 1.3 baseline. See Docs/vulkan-renderer-design.md.
if(APPLE)
    option(DIAMOND_ENABLE_VULKAN "Build the Vulkan rendering backend" OFF)
else()
    option(DIAMOND_ENABLE_VULKAN "Build the Vulkan rendering backend" ON)
endif()

include(FetchContent)

FetchContent_Declare(
    iconfontcppheaders
    GIT_REPOSITORY https://github.com/juliettef/IconFontCppHeaders.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(iconfontcppheaders)

FetchContent_Declare(
    fontawesome
    URL https://github.com/FortAwesome/Font-Awesome/releases/download/5.15.4/fontawesome-free-5.15.4-web.zip
)
FetchContent_MakeAvailable(fontawesome)

add_subdirectory(Engine)
add_subdirectory(Sandbox)

# Headless validation tool for the glTF importer (no window / OpenGL needed).
add_executable(GltfDump Tools/GltfDump.cpp)
target_link_libraries(GltfDump PRIVATE MyEngine)

# M0 Vulkan smoke test — clears the screen via the Vulkan backend.
if(DIAMOND_ENABLE_VULKAN)
    add_executable(VulkanClear Tools/VulkanClear.cpp)
    target_link_libraries(VulkanClear PRIVATE MyEngine)
endif()