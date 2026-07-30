# Standalone game runtime (Milestone 7): loads a .scene and plays it — no
# editor. A separate executable, not an ifdef'd Sandbox: if this target links,
# it's proof no editor code leaked into the shipping path.
add_executable(Runtime
    src/main.cpp
)

target_link_libraries(Runtime PRIVATE MyEngine)

# Runtime includes Scripts/AllScripts.h in one translation unit, just like Sandbox.
# The header-only component/system registrations can exceed MSVC's standard COFF
# section count, so both game-facing targets require the extended object format.
if(MSVC)
    target_compile_options(Runtime PRIVATE /bigobj)
endif()

# Windows subsystem: no console window pops up behind the game. main() stays
# the entry point (main.cpp provides a forwarding WinMain); logs go to
# runtime.log next to the exe, mirrored to the parent terminal when launched
# from one (AttachConsole). Ignored on non-Windows.
set_target_properties(Runtime PROPERTIES WIN32_EXECUTABLE TRUE)

target_include_directories(Runtime PRIVATE
    # VulkanRenderer2D for the in-game UI overlay — an engine-internal header
    # (same private-include access Sandbox has). The header is Vulkan-free
    # (RHI types only); the class is only compiled under DIAMOND_ENABLE_VULKAN,
    # which gates this whole target.
    ${CMAKE_SOURCE_DIR}/Engine/src
    # Scripts/AllScripts.h — game code ships with the game target, not the
    # engine. Still Sandbox-side until a dedicated Game/ module exists.
    ${CMAKE_SOURCE_DIR}/Sandbox/src
)

target_compile_definitions(Runtime PRIVATE
    # Dev-tree asset root, same value as Sandbox/MyEngine. The packager step
    # replaces this with an exe-relative root resolved at runtime.
    ASSETS_DIR="${CMAKE_SOURCE_DIR}/Assets"
    # Must match Engine/CMakeLists.txt's _vk_spv_dir (VulkanRenderer2D loads
    # its shaders from here).
    DIAMOND_VULKAN_SHADER_DIR="${CMAKE_BINARY_DIR}/shaders"
)
