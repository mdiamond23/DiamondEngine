include(FetchContent)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

FetchContent_Declare(
    entt
    GIT_REPOSITORY https://github.com/skypjack/entt.git
    GIT_TAG v3.15.0
)
FetchContent_MakeAvailable(entt)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.15.3
)
FetchContent_MakeAvailable(spdlog)

FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG glad2
    SOURCE_SUBDIR cmake
)
FetchContent_MakeAvailable(glad)
glad_add_library(glad_gl STATIC REPRODUCIBLE LOADER API gl:core=4.6)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

# imgui_impl_glfw.cpp needs GLFW headers
target_link_libraries(imgui PUBLIC glfw)

find_package(OpenGL REQUIRED)

add_library(MyEngine STATIC
    src/Application.cpp
)

target_include_directories(MyEngine PUBLIC
    Include
)

target_link_libraries(MyEngine PUBLIC
    glfw
    OpenGL::GL
    glm::glm
    EnTT::EnTT
    spdlog::spdlog
    imgui
    glad_gl
)