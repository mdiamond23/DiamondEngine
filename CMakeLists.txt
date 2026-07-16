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

# Tracy profiler client. Off by default so normal builds carry no Tracy code;
# enable with -DDIAMOND_TRACY=ON for a build you intend to connect the Tracy
# GUI to. Compiled with TRACY_ON_DEMAND, so even an enabled build collects
# nothing until the GUI attaches. GUI version must match the pinned client tag.
option(DIAMOND_TRACY "Compile in the Tracy profiler client" OFF)

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

# Prebuilt Tracy GUI (Windows release binary) — fetched alongside the client so
# it survives clean builds. Version must stay in lockstep with the client tag
# in Engine/CMakeLists.txt: the wire protocol is version-locked. `Sandbox
# --tracy` spawns it auto-connected (define wired below, after add_subdirectory).
if(DIAMOND_TRACY AND WIN32)
    FetchContent_Declare(
        tracy_gui
        URL https://github.com/wolfpld/tracy/releases/download/v0.13.1/windows-0.13.1.zip
    )
    FetchContent_MakeAvailable(tracy_gui)
endif()

add_subdirectory(Engine)
add_subdirectory(Sandbox)

# Standalone game runtime (Milestone 7) — Vulkan-only: it renders through
# SceneRenderer's swapchain mode, which doesn't exist in a GL-only build
# (macOS keeps the editor; there's no GL shipping path).
if(DIAMOND_ENABLE_VULKAN)
    add_subdirectory(Runtime)
endif()

# PRIVATE and Sandbox-only on purpose: only main.cpp's --tracy handler needs
# the GUI path, and keeping it off MyEngine's PUBLIC defines means changing it
# never rebuilds the engine.
if(DIAMOND_TRACY AND WIN32)
    target_compile_definitions(Sandbox PRIVATE
        DIAMOND_TRACY_GUI="${tracy_gui_SOURCE_DIR}/tracy-profiler.exe")
endif()

# Headless validation tool for the glTF importer (no window / OpenGL needed).
add_executable(GltfDump Tools/GltfDump.cpp)
target_link_libraries(GltfDump PRIVATE MyEngine)

# Vulkan smoke test — draws a textured, lit, depth-tested cube via the RHI.
if(DIAMOND_ENABLE_VULKAN)
    add_executable(VulkanMesh Tools/VulkanMesh.cpp)
    target_link_libraries(VulkanMesh PRIVATE MyEngine)

    # Wiring Slice 1 — drives a real Scene/ECS through SceneRenderer's deferred chain.
    add_executable(VulkanScene Tools/VulkanScene.cpp)
    target_link_libraries(VulkanScene PRIVATE MyEngine)
endif()