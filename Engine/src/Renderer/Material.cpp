#include "Renderer/Material.h"

namespace Diamond {

void PBRMaterial::Bind(const Shader& shader) const
{
    // Surface maps — slots 0-4 match pbr_textured.frag sampler bindings
    if (Albedo)    { Albedo->Bind(0);    shader.SetInt("albedoMap",    0); }
    if (Normal)    { Normal->Bind(1);    shader.SetInt("normalMap",    1); }
    if (Metallic)  { Metallic->Bind(2);  shader.SetInt("metallicMap",  2); }
    if (Roughness) { Roughness->Bind(3); shader.SetInt("roughnessMap", 3); }
    if (AO)        { AO->Bind(4);        shader.SetInt("aoMap",        4); }

    shader.SetFloat("uvScale", UVScale);
    shader.SetVec4("baseColorFactor", BaseColorFactor);
    // 0 disables the alpha test entirely (a < 0.0 never holds), so Opaque and
    // Blend materials skip the discard without a shader variant.
    shader.SetFloat("alphaCutoff", Mode == AlphaMode::Mask ? AlphaCutoff : 0.0f);

    // Emissive map — slot 5; strength=0 disables the contribution even if map is unbound
    shader.SetFloat("emissiveStrength", Emissive ? EmissiveStrength : 0.0f);
    if (Emissive) { Emissive->Bind(5); shader.SetInt("emissiveMap", 5); }

    // IBL maps — slots 6-8
    if (IrradianceMap) { IrradianceMap->Bind(6); shader.SetInt("irradianceMap", 6); }
    if (PrefilterMap)  { PrefilterMap->Bind(7);  shader.SetInt("prefilterMap",  7); }
    if (BrdfLUT)       { BrdfLUT->Bind(8);       shader.SetInt("brdfLUT",       8); }
}

} // namespace Diamond
