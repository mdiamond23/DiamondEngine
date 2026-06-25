add_executable(Sandbox
    src/main.cpp
    src/Scripts/AllScripts.h
    src/Editor/EditorLayers.cpp
    src/Editor/Command.cpp
    src/Editor/SceneSerializer.cpp
    src/Editor/MeshThumbnailRenderer.cpp
    src/Editor/Panels/HierarchyPanel.cpp
    src/Editor/Panels/ViewportPanel.cpp
    src/Editor/Panels/InspectorPanel.cpp
    src/Editor/Panels/ContentPanel.cpp
    src/Editor/Panels/ConsolePanel.cpp
    src/Editor/Panels/GameViewportPanel.cpp
    src/Editor/Panels/AnimatorPanel.cpp
    src/Editor/Panels/ParticlePreviewPanel.cpp
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
)