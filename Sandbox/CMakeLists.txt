add_executable(Sandbox
    src/main.cpp
    src/Editor/EditorLayers.cpp
    src/Editor/Command.cpp
    src/Editor/SceneSerializer.cpp
    src/Editor/MeshThumbnailRenderer.cpp
    src/Editor/Panels/HierarchyPanel.cpp
    src/Editor/Panels/ViewportPanel.cpp
    src/Editor/Panels/InspectorPanel.cpp
    src/Editor/Panels/ContentPanel.cpp
    src/Editor/Panels/ConsolePanel.cpp
)

target_link_libraries(Sandbox PRIVATE MyEngine nlohmann_json::nlohmann_json imguizmo miniz)

target_include_directories(Sandbox PRIVATE
    ${CMAKE_SOURCE_DIR}/Engine/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(Sandbox PRIVATE
    ENGINE_SHADERS_DIR="${CMAKE_SOURCE_DIR}/Engine/Shaders"
    ASSETS_DIR="${CMAKE_SOURCE_DIR}/Assets"
)