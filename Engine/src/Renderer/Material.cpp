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

    // IBL maps — slots 5-7
    if (IrradianceMap) { IrradianceMap->Bind(5); shader.SetInt("irradianceMap", 5); }
    if (PrefilterMap)  { PrefilterMap->Bind(6);  shader.SetInt("prefilterMap",  6); }
    if (BrdfLUT)       { BrdfLUT->Bind(7);       shader.SetInt("brdfLUT",       7); }
}

} // namespace Diamond
