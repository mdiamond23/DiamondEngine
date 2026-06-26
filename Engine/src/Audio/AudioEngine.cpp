#include "Audio/AudioEngine.h"
#include "Audio/AudioAPI.h"

#include "miniaudio.h"   // declarations only — implementation lives in miniaudio_impl.c

#include <spdlog/spdlog.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// All miniaudio state lives in this PIMPL struct, defined only in this TU so no
// miniaudio type escapes into a public header. The Audio:: facade reaches it through
// AudioFacadeAccess (a friend), which holds the single active instance.
// ---------------------------------------------------------------------------
struct AudioEngine::Impl
{
    ma_engine       engine{};
    bool            engineOK = false;

    // The four routable buses (Music, SFX, UI, Ambience). Master is the engine
    // endpoint itself, controlled via ma_engine_set_volume.
    ma_sound_group  groups[Audio::kBusGroupCount]{};

    // Index by (int)Bus — slot 0 (Master) plus the four groups.
    float           busVolume[1 + Audio::kBusGroupCount];
    bool            busMuted [1 + Audio::kBusGroupCount];

    // A loaded clip is just a remembered path + stream flag; miniaudio's resource
    // manager owns the actual decoded data and shares it across plays.
    struct Clip { std::string path; bool stream = false; bool valid = false; };
    std::vector<Clip> clips;

    // ---- clip table -------------------------------------------------------
    Audio::ClipHandle LoadClip(const std::string& path, bool stream)
    {
        if (path.empty()) return Audio::kInvalidClip;

        // Reuse a matching live slot so repeated loads share one handle.
        for (std::size_t i = 0; i < clips.size(); ++i)
            if (clips[i].valid && clips[i].path == path && clips[i].stream == stream)
                return (Audio::ClipHandle)i;

        // Otherwise fill a freed slot, or append.
        Clip clip{ path, stream, true };
        for (std::size_t i = 0; i < clips.size(); ++i)
            if (!clips[i].valid) { clips[i] = clip; return (Audio::ClipHandle)i; }

        clips.push_back(clip);
        return (Audio::ClipHandle)(clips.size() - 1);
    }

    void UnloadClip(Audio::ClipHandle clip)
    {
        if (clip < clips.size()) clips[clip].valid = false;  // in-flight voices unaffected
    }

    // Active fire-and-forget voices, oldest first. Each ma_sound is heap-owned because
    // miniaudio keeps internal pointers to it; it must not move while playing.
    std::vector<ma_sound*> voices;
    int                    voiceCap = Audio::kDefaultVoiceCap;

    // A single editor audition voice, kept apart from the pool so it can always be
    // stopped/replaced instantly and never competes for the voice cap.
    ma_sound* preview = nullptr;

    void StopPreviewVoice()
    {
        if (preview) { ma_sound_uninit(preview); delete preview; preview = nullptr; }
    }

    bool PreviewFile(const std::string& path)
    {
        StopPreviewVoice();
        if (path.empty()) return false;

        // Streamed + non-spatial + no group (master endpoint): instant stop, flat
        // memory regardless of length, independent of game bus volumes.
        auto* s = new ma_sound{};
        const ma_uint32 flags = MA_SOUND_FLAG_STREAM | MA_SOUND_FLAG_NO_SPATIALIZATION;
        if (ma_sound_init_from_file(&engine, path.c_str(), flags, nullptr, nullptr, s)
            != MA_SUCCESS) {
            delete s;
            spdlog::warn("[Audio] failed to preview '{}'", path);
            return false;
        }
        ma_sound_start(s);
        preview = s;
        return true;
    }

    // ---- bus helpers ------------------------------------------------------
    ma_sound_group* GroupFor(Audio::Bus bus)
    {
        // Master routes to the endpoint (NULL group); the rest map to groups[0..3].
        if (bus == Audio::Bus::Master) return nullptr;
        return &groups[(int)bus - 1];
    }

    void ApplyBusGain(Audio::Bus bus)
    {
        const int   i    = (int)bus;
        const float gain = busMuted[i] ? 0.0f : busVolume[i];
        if (bus == Audio::Bus::Master) ma_engine_set_volume(&engine, gain);
        else                           ma_sound_group_set_volume(GroupFor(bus), gain);
    }

    // ---- voice management -------------------------------------------------
    void DestroyVoice(ma_sound* s)
    {
        ma_sound_uninit(s);
        delete s;
    }

    // Spawns and starts a voice for `clip`. Returns the sound, or nullptr on failure.
    // `spatial` toggles 3D positioning; non-spatial plays flat (UI, music, narration).
    ma_sound* StartVoice(Audio::ClipHandle clip, Audio::Bus bus,
                         float volume, float pitch, bool spatial)
    {
        if (clip >= clips.size() || !clips[clip].valid) return nullptr;
        const Clip& c = clips[clip];

        // Evict the oldest voice if we're at the cap, so emission can't exhaust the device.
        while ((int)voices.size() >= voiceCap && !voices.empty()) {
            DestroyVoice(voices.front());
            voices.erase(voices.begin());
        }

        ma_uint32 flags = c.stream ? MA_SOUND_FLAG_STREAM : MA_SOUND_FLAG_DECODE;
        if (!spatial) flags |= MA_SOUND_FLAG_NO_SPATIALIZATION;

        auto* s = new ma_sound{};
        if (ma_sound_init_from_file(&engine, c.path.c_str(), flags, GroupFor(bus), nullptr, s)
            != MA_SUCCESS) {
            delete s;
            spdlog::warn("[Audio] failed to play clip '{}'", c.path);
            return nullptr;
        }

        ma_sound_set_volume(s, volume);
        ma_sound_set_pitch(s, pitch);
        if (spatial) {
            // Sensible directional defaults; per-source overrides arrive with the
            // AudioSourceComponent in a later phase.
            ma_sound_set_attenuation_model(s, ma_attenuation_model_inverse);
            ma_sound_set_min_distance(s, 1.0f);
            ma_sound_set_rolloff(s, 1.0f);
        }
        ma_sound_start(s);
        voices.push_back(s);
        return s;
    }

    // ---- component-owned source voices ------------------------------------
    // Persistent voices keyed by an ever-increasing handle. Unlike the one-shot
    // pool, these are owned by an AudioSourceComponent: never cap-evicted, never
    // auto-reclaimed in Update() — the owner stops them (or Shutdown does).
    std::unordered_map<Audio::VoiceHandle, ma_sound*> sources;
    Audio::VoiceHandle nextSource = 1;

    static ma_attenuation_model ToMaAttenuation(Audio::AttenuationModel m)
    {
        switch (m) {
            case Audio::AttenuationModel::None:        return ma_attenuation_model_none;
            case Audio::AttenuationModel::Linear:      return ma_attenuation_model_linear;
            case Audio::AttenuationModel::Exponential: return ma_attenuation_model_exponential;
            case Audio::AttenuationModel::Inverse:
            default:                                   return ma_attenuation_model_inverse;
        }
    }

    ma_sound* FindSource(Audio::VoiceHandle v)
    {
        auto it = sources.find(v);
        return it == sources.end() ? nullptr : it->second;
    }

    Audio::VoiceHandle StartSource(Audio::ClipHandle clip, const glm::vec3& pos,
                                   const Audio::SourceParams& p)
    {
        if (clip >= clips.size() || !clips[clip].valid) return Audio::kInvalidVoice;
        const Clip& c = clips[clip];

        ma_uint32 flags = (c.stream || p.stream) ? MA_SOUND_FLAG_STREAM : MA_SOUND_FLAG_DECODE;
        if (!p.spatial) flags |= MA_SOUND_FLAG_NO_SPATIALIZATION;

        auto* s = new ma_sound{};
        if (ma_sound_init_from_file(&engine, c.path.c_str(), flags, GroupFor(p.bus), nullptr, s)
            != MA_SUCCESS) {
            delete s;
            spdlog::warn("[Audio] failed to start source '{}'", c.path);
            return Audio::kInvalidVoice;
        }

        ma_sound_set_volume(s, p.volume);
        ma_sound_set_pitch(s, p.pitch);
        ma_sound_set_looping(s, p.loop ? MA_TRUE : MA_FALSE);
        if (p.spatial) {
            ma_sound_set_position(s, pos.x, pos.y, pos.z);
            ma_sound_set_attenuation_model(s, ToMaAttenuation(p.attenuation));
            ma_sound_set_min_distance(s, p.minDistance);
            ma_sound_set_max_distance(s, p.maxDistance);
            ma_sound_set_rolloff(s, p.rolloff);
            ma_sound_set_cone(s, glm::radians(p.coneInnerAngle),
                                 glm::radians(p.coneOuterAngle), p.coneOuterGain);
        }
        ma_sound_start(s);

        Audio::VoiceHandle h = nextSource++;
        sources[h] = s;
        return h;
    }

    void StopSource(Audio::VoiceHandle v)
    {
        auto it = sources.find(v);
        if (it == sources.end()) return;
        DestroyVoice(it->second);
        sources.erase(it);
    }
};

// Grants the namespace-level Audio:: facade access to the active engine's Impl without
// exposing it in the public header. Holds the one active instance.
class AudioFacadeAccess
{
public:
    static AudioEngine::Impl*& Active() { static AudioEngine::Impl* p = nullptr; return p; }
};

// ---------------------------------------------------------------------------
// AudioEngine
// ---------------------------------------------------------------------------
AudioEngine::AudioEngine()  = default;
AudioEngine::~AudioEngine() { Shutdown(); }

bool AudioEngine::Init()
{
    if (m_Impl) return m_Impl->engineOK;
    m_Impl = std::make_unique<Impl>();

    if (ma_engine_init(nullptr, &m_Impl->engine) != MA_SUCCESS) {
        spdlog::error("[Audio] ma_engine_init failed — audio disabled");
        m_Impl.reset();
        return false;
    }
    m_Impl->engineOK = true;

    // Buses: all unity gain, unmuted. groups[i] is parented to the endpoint (NULL).
    for (int i = 0; i < 1 + Audio::kBusGroupCount; ++i) {
        m_Impl->busVolume[i] = 1.0f;
        m_Impl->busMuted[i]  = false;
    }
    for (int i = 0; i < Audio::kBusGroupCount; ++i) {
        if (ma_sound_group_init(&m_Impl->engine, 0, nullptr, &m_Impl->groups[i]) != MA_SUCCESS)
            spdlog::warn("[Audio] sound group {} failed to init", i);
    }

    // Default listener: at the origin, looking down -Z, Y up.
    ma_engine_listener_set_position (&m_Impl->engine, 0, 0.0f, 0.0f,  0.0f);
    ma_engine_listener_set_direction(&m_Impl->engine, 0, 0.0f, 0.0f, -1.0f);
    ma_engine_listener_set_world_up (&m_Impl->engine, 0, 0.0f, 1.0f,  0.0f);

    AudioFacadeAccess::Active() = m_Impl.get();
    spdlog::info("[Audio] miniaudio engine initialized ({} Hz)",
                 ma_engine_get_sample_rate(&m_Impl->engine));
    return true;
}

void AudioEngine::Shutdown()
{
    if (!m_Impl) return;

    if (AudioFacadeAccess::Active() == m_Impl.get())
        AudioFacadeAccess::Active() = nullptr;

    for (auto* s : m_Impl->voices) m_Impl->DestroyVoice(s);
    m_Impl->voices.clear();
    for (auto& [h, s] : m_Impl->sources) m_Impl->DestroyVoice(s);
    m_Impl->sources.clear();
    m_Impl->StopPreviewVoice();

    if (m_Impl->engineOK) {
        for (int i = 0; i < Audio::kBusGroupCount; ++i)
            ma_sound_group_uninit(&m_Impl->groups[i]);
        ma_engine_uninit(&m_Impl->engine);
    }
    m_Impl.reset();
}

void AudioEngine::Update()
{
    if (!m_Impl) return;

    // Reclaim any voice that has reached its end. Swap-erase keeps it O(1) per removal;
    // order among the survivors only matters for cap-eviction, which tolerates the swap.
    auto& voices = m_Impl->voices;
    for (std::size_t i = 0; i < voices.size();) {
        if (ma_sound_at_end(voices[i])) {
            m_Impl->DestroyVoice(voices[i]);
            voices[i] = voices.back();
            voices.pop_back();
        } else {
            ++i;
        }
    }

    // Drop the preview voice once it finishes so IsPreviewPlaying() reflects reality.
    if (m_Impl->preview && ma_sound_at_end(m_Impl->preview))
        m_Impl->StopPreviewVoice();
}

bool AudioEngine::IsActive() const { return m_Impl && m_Impl->engineOK; }

// ---------------------------------------------------------------------------
// Audio:: facade — forwards to the active engine, silent no-op when none exists.
// ---------------------------------------------------------------------------
namespace Audio {

ClipHandle Load(const std::string& path, bool stream)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return kInvalidClip;
    return impl->LoadClip(path, stream);
}

void Unload(ClipHandle clip)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    impl->UnloadClip(clip);
}

void PlayOneShot(ClipHandle clip, const glm::vec3& position, Bus bus, float volume, float pitch)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    if (ma_sound* s = impl->StartVoice(clip, bus, volume, pitch, /*spatial*/true))
        ma_sound_set_position(s, position.x, position.y, position.z);
}

void Play2D(ClipHandle clip, Bus bus, float volume, float pitch)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    impl->StartVoice(clip, bus, volume, pitch, /*spatial*/false);
}

VoiceHandle PlaySource(ClipHandle clip, const glm::vec3& position, const SourceParams& params)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return kInvalidVoice;
    return impl->StartSource(clip, position, params);
}

void StopVoice(VoiceHandle voice)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    impl->StopSource(voice);
}

void SetVoicePosition(VoiceHandle voice, const glm::vec3& position)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    if (ma_sound* s = impl->FindSource(voice))
        ma_sound_set_position(s, position.x, position.y, position.z);
}

void SetVoiceVolume(VoiceHandle voice, float volume)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    if (ma_sound* s = impl->FindSource(voice))
        ma_sound_set_volume(s, volume);
}

void SetVoicePitch(VoiceHandle voice, float pitch)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    if (ma_sound* s = impl->FindSource(voice))
        ma_sound_set_pitch(s, pitch);
}

bool IsVoicePlaying(VoiceHandle voice)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return false;
    ma_sound* s = impl->FindSource(voice);
    return s && !ma_sound_at_end(s);
}

bool PreviewClip(const std::string& path)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return false;
    return impl->PreviewFile(path);
}

void StopPreview()
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    impl->StopPreviewVoice();
}

bool IsPreviewPlaying()
{
    auto* impl = AudioFacadeAccess::Active();
    return impl && impl->preview && !ma_sound_at_end(impl->preview);
}

void SetListener(const glm::vec3& position, const glm::vec3& forward,
                 const glm::vec3& up, const glm::vec3& velocity)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    ma_engine_listener_set_position (&impl->engine, 0, position.x, position.y, position.z);
    ma_engine_listener_set_direction(&impl->engine, 0, forward.x,  forward.y,  forward.z);
    ma_engine_listener_set_world_up (&impl->engine, 0, up.x,       up.y,       up.z);
    ma_engine_listener_set_velocity (&impl->engine, 0, velocity.x, velocity.y, velocity.z);
}

void SetBusVolume(Bus bus, float volume)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    impl->busVolume[(int)bus] = volume < 0.0f ? 0.0f : volume;
    impl->ApplyBusGain(bus);
}

float GetBusVolume(Bus bus)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return 0.0f;
    return impl->busVolume[(int)bus];
}

void SetBusMuted(Bus bus, bool muted)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return;
    impl->busMuted[(int)bus] = muted;
    impl->ApplyBusGain(bus);
}

bool IsBusMuted(Bus bus)
{
    auto* impl = AudioFacadeAccess::Active();
    if (!impl) return false;
    return impl->busMuted[(int)bus];
}

int ActiveVoiceCount()
{
    auto* impl = AudioFacadeAccess::Active();
    return impl ? (int)impl->voices.size() : 0;
}

int VoiceCap()
{
    auto* impl = AudioFacadeAccess::Active();
    return impl ? impl->voiceCap : 0;
}

} // namespace Audio
