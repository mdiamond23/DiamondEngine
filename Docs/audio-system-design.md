# Audio System Design

DiamondEngine's audio layer, built on **miniaudio** (single-header, zero-dependency,
public-domain). miniaudio is compiled into the engine the same way Jolt is wired in:
**linked/compiled PRIVATE, with no miniaudio type ever appearing in a public header.**
Game scripts and the editor talk only to the `Audio::` facade and the ECS components.

See also: [particle-system-design.md](particle-system-design.md) and
[ui-widgets-design.md](ui-widgets-design.md) for the system/component/serialization
conventions this mirrors.

## Why miniaudio

Chosen over SoLoud (effectively dormant) and FMOD/Wwise (proprietary, heavyweight
authoring ecosystems that resist a "wrap-my-own-thin-API" goal). miniaudio gives a
single-translation-unit, fully-owned audio layer with a built-in 3D spatialization
engine and a node graph for custom DSP — the right altitude for this engine.

## Goals

- Modern feel, not "low budget": bus mixer, 3D directional audio, editor audition.
- Lightweight and fully owned behind a thin C++ API (scripting + entt dispatch).
- Architected so a true HRTF backend (Steam Audio / phonon) can drop in later.

## Build integration

- `miniaudio` fetched via FetchContent (pinned `0.11.25`), **populated only** — no
  `MakeAvailable` (no CMakeLists we want). Same manual-compile pattern as ImGui.
- `Engine/src/Audio/miniaudio_impl.c` is the one TU that defines
  `MINIAUDIO_IMPLEMENTATION`. It integrates `extras/stb_vorbis.c` for **OGG/Vorbis**;
  **WAV / MP3 / FLAC** are built into miniaudio natively. Compiled as C, isolated so
  the giant implementation rarely recompiles.
- `Engine/src/Audio/AudioEngine.cpp` includes `miniaudio.h` for *declarations only*
  and holds all miniaudio state behind a PIMPL.
- The miniaudio include dir is added **PRIVATE** to `MyEngine`. Platform link libs:
  macOS frameworks (CoreFoundation/CoreAudio/AudioToolbox); Linux `pthread m dl`;
  Windows needs none.

## Architecture

```
 Game scripts ─┐
 UI / events  ─┼─►  Audio::  (free-function facade)  ──►  AudioEngine (global)
 ECS systems  ─┘                                               │ owns
                                                               ▼
                                            ma_engine + 4 ma_sound_groups (buses)
                                                               │
                              voices (ma_sound pool, fire-and-forget + component-owned)
```

### `AudioEngine`
Owns one `ma_engine`, the four bus groups, the clip table, and the active-voice pool.
A single global instance is created/torn down by the app (mirrors how the Physics
session is the implicit target of the `Physics::` free functions). All `Audio::`
functions are silent no-ops when no engine is active.

### Buses (fixed set)
```cpp
enum class Audio::Bus { Master, Music, SFX, UI, Ambience };
```
`Master` is the `ma_engine` endpoint (engine volume). The other four are
`ma_sound_group`s parented to the endpoint, each with volume + mute (+ solo, surfaced
by the mixer panel in Phase 4). Sources pick a bus; one-shots default to `SFX`,
`Play2D` defaults to `UI` (overridable, e.g. `Music` for a stinger).

### Clips
`Audio::Load(path, stream)` returns an opaque `ClipHandle` (index into a clip table of
`{path, streamFlag}`). Playback uses `ma_sound_init_from_file`, which decodes through
miniaudio's resource manager — repeated plays of the same file share cached data, so a
`ClipHandle` is just a cheap reference, not an owned buffer. `stream` defaults via a
smart heuristic at the asset layer (long → stream, short → decode to RAM), overridable.

### Voices
Each active sound is a heap `ma_sound` kept alive in the engine's voice pool until it
finishes. `Update()` reclaims any voice past its end (`ma_sound_at_end`) — and, in a
later phase, publishes `SoundFinished`. A global **voice cap** (default 32) steals the
oldest voice when exceeded, so runaway emission can't exhaust the device.

### Spatialization — 3D now, HRTF-ready
One-shots and 3D sources use miniaudio's built-in spatializer: attenuation model
(default **inverse**), min/max distance, rolloff, directional **cones** (inner/outer
angle + outer gain), and Doppler. The listener is driven from an
`AudioListenerComponent` (Phase 3), usually on the camera. Routing is
`source → bus group → endpoint`; a Steam Audio HRTF node can later be inserted between
source and bus via the node graph **without touching components**.

## Public API (`Audio::`)

```cpp
namespace Audio {
    // Clips
    ClipHandle Load(const std::string& path, bool stream = false);
    void       Unload(ClipHandle clip);

    // Playback
    void PlayOneShot(ClipHandle clip, const glm::vec3& position,
                     Bus bus = Bus::SFX, float volume = 1.0f, float pitch = 1.0f);
    void Play2D(ClipHandle clip, Bus bus = Bus::UI,  // non-spatial: UI, stingers, narration
                float volume = 1.0f, float pitch = 1.0f);

    // Listener
    void SetListener(const glm::vec3& position, const glm::vec3& forward,
                     const glm::vec3& up, const glm::vec3& velocity = glm::vec3(0));

    // Editor audition — single streamed preview voice, separate from the pool.
    bool PreviewClip(const std::string& path);  void StopPreview();  bool IsPreviewPlaying();

    // Mixer
    void  SetBusVolume(Bus bus, float volume);   float GetBusVolume(Bus bus);
    void  SetBusMuted (Bus bus, bool muted);     bool  IsBusMuted (Bus bus);
}
```

## Editor

- **AudioClip** content-browser asset type (`.wav/.mp3/.flac/.ogg`): magenta note
  icon, drag-drop via `CONTENT_ITEM_PATH` (so the AudioSourceComponent can accept it),
  like fonts and materials. *(Phase 2 — done.)*
- **Audition**: double-click (or context-menu Play/Stop) auditions a clip through
  `Audio::PreviewClip` in edit mode — a streamed preview voice with a pulsing green
  ring + stop badge on the playing clip. *(Phase 2 — done.)* Inspector audition lands
  with the AudioSource inspector in Phase 3.
- **Mixer panel**: DAW-style per-bus strips (vertical volume fader + dB readout +
  mute/solo), Master set off from the four routable groups. Volume is owned by the
  engine (read/written via the facade); mute/solo are resolved in the panel — it keeps
  the user's mute intent and solo set, then each frame pushes the effective mute
  (`userMute || any-solo-and-not-soloed`) to the engine. Solo never touches Master;
  engine mute is lossless so toggling solo can't lose a fader. *(Phase 4 — done.)*
- **Spatial-audio viewport visualization**: under the viewport "Debug Draw" toggle,
  each 3D `AudioSourceComponent` draws a min/max distance sphere pair (selected one
  highlighted) and, when not omnidirectional, its directional cone; each enabled
  `AudioListenerComponent` draws a marker + forward line. Built on `DebugDraw`,
  alongside the IK/collider debug draw. *(Phase 4 — done.)*

## ECS

- `AudioListenerComponent` — syncs position/orientation to the engine listener.
- `AudioSourceComponent` — clip handle, bus, volume, pitch, loop, autoplay, `is3D`,
  attenuation model, min/max distance, cone params, `stream` flag. Position synced
  from `TransformComponent`. Registered via `ComponentRegistry` for free
  serialization + inspector (same path as `ParticleEmitterComponent`).
- Events: `SoundStarted` / `SoundFinished` / `UISoundPlayed` (in `Scene/Events.h`)
  published through `Scene::Events()` (entt dispatcher), matching the UI event-bus
  pattern. **Crucially these are entity-scoped, so they cover only component-owned
  source voices** — fire-and-forget one-shots are reclaimed inside `AudioEngine`,
  which has no entity/Scene context, so they emit nothing. `AudioSystem` publishes
  `SoundStarted` when it starts a source voice and `SoundFinished` when it reclaims a
  finished non-looping one (both `enqueue`d, drained end-of-frame). **UI-bus
  integration:** `AudioSystem` subscribes to `UIButtonFired` in `OnStart`; a fired
  button carrying an `AudioSourceComponent` is played as a one-shot on its bus
  (`Play2D` unless `is3D`), then `UISoundPlayed` is published. Subscriptions are
  dropped when `StopPlay` clears the dispatcher.

## Build phases

1. miniaudio implemented in; `AudioEngine` + buses + `Audio::PlayOneShot`/
   `Play2D` + listener + bus mixer API + voice cap. App init/update/shutdown. No ECS,
   no events yet — goal is "sound comes out."
2. `AudioClip` content-browser asset + audition preview.
3. `AudioListenerComponent` + `AudioSourceComponent` + Transform sync + serialization
   + inspector.
4. Mixer panel + spatial-audio viewport visualization. *(done.)*
5. entt events (`SoundStarted`/`SoundFinished`/`UISoundPlayed`) + UI-bus integration
   + voice-limiting polish (mixer voice meter via `Audio::ActiveVoiceCount`/`VoiceCap`).
   *(done — audio system feature-complete.)*

## Deferred (architected for, not built)

Steam Audio HRTF node, reverb/effect zones, ducking/snapshots, user-defined buses.
