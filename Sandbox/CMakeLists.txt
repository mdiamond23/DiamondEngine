add_executable(Sandbox
    src/main.cpp
)

target_link_libraries(Sandbox PRIVATE MyEngine)

target_include_directories(Sandbox PRIVATE
    ${CMAKE_SOURCE_DIR}/Engine/src
)

target_compile_definitions(Sandbox PRIVATE
    ENGINE_SHADERS_DIR="${CMAKE_SOURCE_DIR}/Engine/Shaders"
    ASSETS_DIR="${CMAKE_SOURCE_DIR}/Assets"
)