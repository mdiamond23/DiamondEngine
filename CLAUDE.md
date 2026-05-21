# DiamondEngine

A C++20 game engine with an OpenGL renderer, built with CMake and FetchContent.

## Project Structure

```
DiamondEngine/
├── Engine/          # Core engine library (builds as libMyEngine.a)
│   ├── src/         # Source and private headers
│   └── include/     # Public headers
├── Sandbox/         # Test application that links against Engine
│   └── src/
├── ThirdParty/      # Reserved for vendored/patched dependencies
├── Assets/          # Textures, models, shaders
└── build/           # Generated — never commit this
```

## Building

Always run CMake from the project root using absolute paths (relative paths can misplace the `_deps` download directory):

```bash
cmake -S /path/to/DiamondEngine -B /path/to/DiamondEngine/build
cmake --build /path/to/DiamondEngine/build
```

Or from the project root:

```bash
cmake -B build
cmake --build build
```

## Third-Party Dependencies

All managed via FetchContent — sources download to `build/_deps/` on first configure. Do not move them; they are regenerated on clean builds.

| Library | Version | Target | Include |
|---------|---------|--------|---------|
| GLFW | 3.4 | `glfw` | `<GLFW/glfw3.h>` |
| GLM | 1.0.1 | `glm::glm` | `<glm/glm.hpp>` |
| EnTT | v3.15.0 | `EnTT::EnTT` | `<entt/entt.hpp>` |
| spdlog | v1.15.3 | `spdlog::spdlog` | `<spdlog/spdlog.h>` |
| ImGui | docking branch | `imgui` | `<imgui.h>` |
| GLAD2 | glad2 branch | `glad_gl` | `<glad/gl.h>` |

GLAD2 generates its sources at configure time using Python + jinja2. If generation fails, run `pip3 install jinja2`.

## Platform Notes

- **macOS**: OpenGL 4.1 Core Profile (Apple's maximum). The `GL_SILENCE_DEPRECATION` warning about `glClearColor`/`glClear` is harmless — Apple deprecated OpenGL in 10.14 but it still works.
- **Other platforms**: OpenGL 4.5/4.6 Core Profile.

## CMake Notes

- Root project uses `LANGUAGES C CXX` — C is required by GLAD2.
- ImGui uses the deprecated `FetchContent_Populate` (not `FetchContent_MakeAvailable`) because ImGui has no CMakeLists.txt; its sources are compiled manually into a static library.
- `jinja2` must be installed for GLAD2 source generation: `pip3 install jinja2`.
