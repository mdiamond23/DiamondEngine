add_executable(Sandbox
    src/main.cpp
    src/Scripts/AllScripts.h
    src/AssetPipeline/TextureCooker.cpp
    src/Editor/EditorLayers.cpp
    src/Editor/GLEditorBackend.cpp
    src/Editor/Command.cpp
    src/Editor/MeshThumbnailRenderer.cpp
    src/Editor/Panels/HierarchyPanel.cpp
    src/Editor/Panels/ViewportPanel.cpp
    src/Editor/Panels/InspectorPanel.cpp
    src/Editor/Panels/ContentPanel.cpp
    src/Editor/Panels/ConsolePanel.cpp
    src/Editor/Panels/GameViewportPanel.cpp
    src/Editor/Panels/AnimatorPanel.cpp
    src/Editor/Panels/ParticlePreviewPanel.cpp
    src/Editor/Panels/MixerPanel.cpp
    src/Editor/Panels/PackagePanel.cpp
    src/Editor/Panels/ProfilerPanel.cpp
)

target_link_libraries(Sandbox PRIVATE MyEngine nlohmann_json::nlohmann_json imguizmo miniz)

# InspectorPanel.cpp's many inline lambdas + template instantiations overflow the
# COFF section limit on MSVC; /bigobj raises it. Harmless on other compilers.
if(MSVC)
    target_compile_options(Sandbox PRIVATE /bigobj)
endif()

target_include_directories(Sandbox PRIVATE
    ${CMAKE_SOURCE_DIR}/Engine/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${iconfontcppheaders_SOURCE_DIR}
)

target_compile_definitions(Sandbox PRIVATE
    ENGINE_SHADERS_DIR="${CMAKE_SOURCE_DIR}/Engine/Shaders"
    ASSETS_DIR="${CMAKE_SOURCE_DIR}/Assets"
    FONTS_DIR="${fontawesome_SOURCE_DIR}/webfonts"
    SCRIPTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/Scripts"
    # The Packager panel shells out to `cmake --build <this>` and copies
    # Runtime.exe + build/shaders/*.spv out of it.
    DIAMOND_BUILD_DIR="${CMAKE_BINARY_DIR}"
)

# texconv (DirectXTex) drives the texture cooker's BC7 shell-out (TextureCooker.cpp),
# same offline-tool pattern as glslangValidator in Engine/CMakeLists.txt. Unlike
# glslangValidator this is optional, not FATAL_ERROR: cooking is a dev-time
# convenience (Assets/Cache is gitignored, per-machine), not required to build or
# run the engine. TextureCooker also checks Tools/ at runtime, so dropping the
# executable there after configuration enables the editor action immediately.
find_program(DIAMOND_TEXCONV_PATH NAMES texconv
    HINTS ${CMAKE_SOURCE_DIR}/Tools)
if(DIAMOND_TEXCONV_PATH)
    target_compile_definitions(Sandbox PRIVATE DIAMOND_TEXCONV="${DIAMOND_TEXCONV_PATH}")
else()
    message(STATUS "texconv not found at configure time; Assets > Cook Textures will "
                   "check Tools/ again at runtime.")
endif()

# Headless serializer tests (no window/GPU). Excluded from ALL so normal
# builds stay fast: cmake --build build --target PrefabTests, then run
# build/Sandbox/<config>/PrefabTests.exe (exit code = failure count).
add_executable(PrefabTests EXCLUDE_FROM_ALL
    tests/PrefabTests.cpp
)
target_link_libraries(PrefabTests PRIVATE MyEngine nlohmann_json::nlohmann_json imguizmo miniz)
target_include_directories(PrefabTests PRIVATE
    ${CMAKE_SOURCE_DIR}/Engine/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(PrefabTests PRIVATE
    ASSETS_DIR="${CMAKE_SOURCE_DIR}/Assets"
)
