#pragma once

namespace Diamond {

class VulkanRHIDevice;

// Bring-up check for the RHI compute path (the GI prerequisite — see
// Docs/gi-prerequisites, step 1). Dispatches compute_selftest.comp twice
// through the ordinary RHI surface — CreateComputePipeline, a storage image and
// storage buffer in one resource set, Dispatch, StorageBarrier — reads the
// result back on the CPU, and logs PASS/FAIL.
//
// Runs from the device constructor only when DIAMOND_COMPUTE_SELFTEST is set in
// the environment, so it costs nothing on a normal launch. Delete once real
// compute passes (SSGI/DDGI) exercise this ground for us.
void RunComputeSelfTest(VulkanRHIDevice& device, const char* shaderDir);

// The same check one layer up, for the render graph's compute support: builds a
// three-pass graph (produce a gradient → reduce it plus last execute's
// accumulation into a storage buffer → copy the gradient into the persistent
// accumulation), runs it TWICE, and verifies the second run saw the first run's
// accumulation. Covers compute pass nodes, graph storage buffers and their
// barriers, persistent textures, and ReadHistory's ordering. Same env gate.
void RunGraphSelfTest(VulkanRHIDevice& device, const char* shaderDir);

} // namespace Diamond
