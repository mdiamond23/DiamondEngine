#pragma once

#include "Renderer/RHI/RHIResources.h"
#include "Platform/Vulkan/VulkanBuffer.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"

#include <vector>

namespace Diamond {

// A Vulkan acceleration structure — the BVH ray traversal walks. One class
// serves both levels because the build path is identical apart from the geometry
// description: size query → storage allocation → vkCreateAccelerationStructureKHR
// → scratch allocation → vkCmdBuildAccelerationStructuresKHR.
//
// Builds run through VulkanContext::ImmediateSubmit, so they BLOCK. That is fine
// for slice 2: BLASes are built once at mesh registration and the TLAS only
// rebuilds when the static set changes (the same hash that drives static
// shadow-map invalidation). Moving the build into the frame command buffer is a
// slice-5 concern.
class VulkanRHIAccelStruct : public RHIAccelStruct {
public:
    // Bottom level: one mesh's triangles in local space.
    VulkanRHIAccelStruct(VulkanRHIDevice* device, const RHIBLASDesc& desc);
    // Top level: instances of BLASes with world transforms.
    VulkanRHIAccelStruct(VulkanRHIDevice* device,
                         const std::vector<RHITLASInstance>& instances);
    ~VulkanRHIAccelStruct() override;

    // False when construction bailed (empty geometry). Nothing else on the
    // object is meaningful in that case.
    bool Valid() const { return m_Handle != VK_NULL_HANDLE; }

    // TLAS only: re-point at a new instance set. False when the new count
    // exceeds what this structure was sized for — the caller must then recreate
    // the TLAS *and* every resource set binding it.
    bool Rebuild(const std::vector<RHITLASInstance>& instances);

    VkAccelerationStructureKHR Handle()  const { return m_Handle; }
    VkDeviceAddress            Address() const { return m_Address; }

private:
    // Shared tail of both constructors: allocates storage + scratch, creates the
    // structure, records the build. 'buildInfo' arrives with type/flags/geometry
    // filled in and no dst/scratch.
    void Build(VkAccelerationStructureBuildGeometryInfoKHR& buildInfo,
               uint32_t primitiveCount, const char* debugName);
    // Fills m_InstanceBuffer from 'instances'. Returns false if it doesn't fit.
    bool WriteInstances(const std::vector<RHITLASInstance>& instances);

    VulkanRHIDevice* m_Device;
    bool             m_TopLevel = false;

    VulkanBuffer m_Storage;   // backing memory of the structure itself
    VulkanBuffer m_Scratch;   // build workspace — freed after a BLAS build, kept
                              // for a TLAS so rebuilds don't re-allocate
    VulkanBuffer m_Instances; // TLAS only: host-visible VkAccelerationStructureInstanceKHR[]
    void*        m_InstancesMapped   = nullptr;
    uint32_t     m_InstanceCapacity  = 0;
    uint32_t     m_InstanceCount     = 0;
    VkDeviceSize m_ScratchAddress    = 0;   // already aligned

    VkAccelerationStructureKHR m_Handle  = VK_NULL_HANDLE;
    VkDeviceAddress            m_Address = 0;
};

// GPU address of a buffer. The buffer must carry ShaderDeviceAddress usage.
VkDeviceAddress BufferDeviceAddress(VkDevice device, VkBuffer buffer);

} // namespace Diamond
