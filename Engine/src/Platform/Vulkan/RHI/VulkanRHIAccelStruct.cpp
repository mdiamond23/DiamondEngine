#include "Platform/Vulkan/RHI/VulkanRHIAccelStruct.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"

#include <algorithm>
#include <cstring>

namespace Diamond {

VkDeviceAddress BufferDeviceAddress(VkDevice device, VkBuffer buffer) {
    VkBufferDeviceAddressInfo info{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
    info.buffer = buffer;
    return vkGetBufferDeviceAddress(device, &info);
}

namespace {

constexpr VkBufferUsageFlags kASStorageUsage =
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

constexpr VkBufferUsageFlags kScratchUsage =
    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

constexpr VkBufferUsageFlags kInstanceUsage =
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

VkDeviceSize AlignUp(VkDeviceSize value, VkDeviceSize alignment) {
    return alignment == 0 ? value : (value + alignment - 1) & ~(alignment - 1);
}

} // namespace

// ── Bottom level ─────────────────────────────────────────────────────────────

VulkanRHIAccelStruct::VulkanRHIAccelStruct(VulkanRHIDevice* device, const RHIBLASDesc& desc)
    : m_Device(device) {
    if (!desc.vertexBuffer || !desc.indexBuffer || desc.indexCount < 3 || desc.vertexCount == 0)
        return;   // degenerate mesh — Valid() stays false and the TLAS skips it

    VkDevice dev = m_Device->Ctx().Device();
    auto* vb = static_cast<VulkanRHIBuffer*>(desc.vertexBuffer);
    auto* ib = static_cast<VulkanRHIBuffer*>(desc.indexBuffer);

    // Static buffers ignore the frame index; a dynamic mesh buffer would give a
    // different address per slot, which a once-built BLAS could not represent.
    VkAccelerationStructureGeometryKHR geom{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
    geom.flags        = VK_GEOMETRY_OPAQUE_BIT_KHR;   // no any-hit; alpha-mask is a later problem

    auto& tris = geom.geometry.triangles;
    tris.sType         = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
    tris.vertexFormat  = VK_FORMAT_R32G32B32_SFLOAT;   // position at offset 0
    tris.vertexData.deviceAddress = BufferDeviceAddress(dev, vb->Handle(0));
    tris.vertexStride  = desc.vertexStride;
    tris.maxVertex     = desc.vertexCount - 1;
    tris.indexType     = VK_INDEX_TYPE_UINT32;
    tris.indexData.deviceAddress = BufferDeviceAddress(dev, ib->Handle(0));

    VkAccelerationStructureBuildGeometryInfoKHR build{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    build.type          = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    build.flags         = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    build.mode          = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    build.geometryCount = 1;
    build.pGeometries   = &geom;

    Build(build, desc.indexCount / 3, desc.debugName);

    // A BLAS is built exactly once, so its scratch is dead the moment the build
    // retires — and ImmediateSubmit already waited for that. Holding one scratch
    // per mesh would waste hundreds of MB on a scene like Sponza.
    DestroyBuffer(m_Device->Ctx().Allocator(), m_Scratch);
}

// ── Top level ────────────────────────────────────────────────────────────────

VulkanRHIAccelStruct::VulkanRHIAccelStruct(VulkanRHIDevice* device,
                                           const std::vector<RHITLASInstance>& instances)
    : m_Device(device), m_TopLevel(true) {
    if (instances.empty()) return;   // nothing to trace against

    VmaAllocator allocator = m_Device->Ctx().Allocator();

    // Headroom so ordinary scene edits (adding a few props) rebuild in place
    // instead of forcing every resource set that binds the TLAS to be recreated.
    m_InstanceCapacity = static_cast<uint32_t>(instances.size()) + 64;

    VmaAllocationInfo info{};
    m_Instances = CreateBuffer(
        allocator, VkDeviceSize(m_InstanceCapacity) * sizeof(VkAccelerationStructureInstanceKHR),
        kInstanceUsage, VMA_MEMORY_USAGE_AUTO,
        VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
        VMA_ALLOCATION_CREATE_MAPPED_BIT,
        &info);
    m_InstancesMapped = info.pMappedData;

    if (!WriteInstances(instances)) return;

    VkAccelerationStructureGeometryKHR geom{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geom.flags        = VK_GEOMETRY_OPAQUE_BIT_KHR;
    geom.geometry.instances.sType =
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    geom.geometry.instances.arrayOfPointers  = VK_FALSE;
    geom.geometry.instances.data.deviceAddress =
        BufferDeviceAddress(m_Device->Ctx().Device(), m_Instances.handle);

    // ALLOW_UPDATE is what makes Refit legal later. It costs a little traversal
    // performance and storage against a build-only structure, which is a good
    // trade for not doing a full rebuild every frame something moves.
    VkAccelerationStructureBuildGeometryInfoKHR build{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    build.type          = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    build.flags         = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
                        | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    build.mode          = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    build.geometryCount = 1;
    build.pGeometries   = &geom;
    m_Updatable         = true;

    // Sized for the full capacity, not the current count, so a rebuild that adds
    // instances still fits the storage and scratch allocated here.
    Build(build, m_InstanceCapacity, "TLAS");
}

bool VulkanRHIAccelStruct::WriteInstances(const std::vector<RHITLASInstance>& instances) {
    if (instances.size() > m_InstanceCapacity || !m_InstancesMapped) return false;

    auto* dst = static_cast<VkAccelerationStructureInstanceKHR*>(m_InstancesMapped);
    for (size_t i = 0; i < instances.size(); ++i) {
        const RHITLASInstance& src = instances[i];
        auto* blas = static_cast<VulkanRHIAccelStruct*>(src.blas);
        if (!blas || !blas->Valid()) return false;

        VkAccelerationStructureInstanceKHR inst{};
        std::memcpy(&inst.transform, src.transform, sizeof(inst.transform));
        inst.instanceCustomIndex = src.customIndex & 0xFFFFFFu;   // 24-bit field
        inst.mask                = 0xFF;
        inst.instanceShaderBindingTableRecordOffset = 0;   // one hit group for now
        inst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
        inst.accelerationStructureReference = blas->Address();
        dst[i] = inst;
    }
    m_InstanceCount = static_cast<uint32_t>(instances.size());

    // The allocation is very likely HOST_COHERENT, but VMA only guarantees that
    // when it says so — flushing an already-coherent range is a no-op.
    vmaFlushAllocation(m_Device->Ctx().Allocator(), m_Instances.allocation, 0, VK_WHOLE_SIZE);
    return true;
}

bool VulkanRHIAccelStruct::Rebuild(const std::vector<RHITLASInstance>& instances) {
    if (!m_TopLevel || !Valid()) return false;
    if (instances.empty()) { m_InstanceCount = 0; return true; }
    if (!WriteInstances(instances)) return false;

    VkAccelerationStructureGeometryKHR geom{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geom.flags        = VK_GEOMETRY_OPAQUE_BIT_KHR;
    geom.geometry.instances.sType =
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    geom.geometry.instances.arrayOfPointers  = VK_FALSE;
    geom.geometry.instances.data.deviceAddress =
        BufferDeviceAddress(m_Device->Ctx().Device(), m_Instances.handle);

    // A full rebuild rather than an UPDATE: an update cannot change topology,
    // and callers only reach Rebuild when it did. ALLOW_UPDATE carries over so
    // the refit path keeps working against the rebuilt structure, and because
    // the flags must match the ones the storage was sized against.
    VkAccelerationStructureBuildGeometryInfoKHR build{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    build.type                     = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    build.flags                    = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
                                   | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    build.mode                     = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    build.geometryCount            = 1;
    build.pGeometries              = &geom;
    build.dstAccelerationStructure = m_Handle;
    build.scratchData.deviceAddress = m_ScratchAddress;

    VkAccelerationStructureBuildRangeInfoKHR range{};
    range.primitiveCount = m_InstanceCount;
    const VkAccelerationStructureBuildRangeInfoKHR* pRange = &range;

    // Nothing may be reading the structure while it is rewritten in place.
    m_Device->WaitIdle();
    m_Device->Ctx().ImmediateSubmit([&](VkCommandBuffer cmd) {
        vkCmdBuildAccelerationStructuresKHR(cmd, 1, &build, &pRange);
    });
    return true;
}

bool VulkanRHIAccelStruct::Refit(const std::vector<RHITLASInstance>& instances) {
    if (!m_TopLevel || !Valid() || !m_Updatable) return false;
    // An update must keep the primitive count its source was built with, so a
    // changed instance count is a Rebuild, not a Refit.
    if (instances.empty() || instances.size() != m_InstanceCount) return false;
    if (!WriteInstances(instances)) return false;

    VkAccelerationStructureGeometryKHR geom{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geom.flags        = VK_GEOMETRY_OPAQUE_BIT_KHR;
    geom.geometry.instances.sType =
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    geom.geometry.instances.arrayOfPointers  = VK_FALSE;
    geom.geometry.instances.data.deviceAddress =
        BufferDeviceAddress(m_Device->Ctx().Device(), m_Instances.handle);

    // src == dst is an in-place update, which is explicitly permitted and saves
    // keeping a second structure to ping-pong between.
    VkAccelerationStructureBuildGeometryInfoKHR build{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    build.type                      = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    build.flags                     = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
                                    | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    build.mode                      = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
    build.geometryCount             = 1;
    build.pGeometries               = &geom;
    build.srcAccelerationStructure  = m_Handle;
    build.dstAccelerationStructure  = m_Handle;
    build.scratchData.deviceAddress = m_ScratchAddress;

    VkAccelerationStructureBuildRangeInfoKHR range{};
    range.primitiveCount = m_InstanceCount;
    const VkAccelerationStructureBuildRangeInfoKHR* pRange = &range;

    // Same in-place hazard as Rebuild: nothing may be tracing the structure, and
    // nothing may be mid-build off the instance buffer, while both are rewritten.
    m_Device->WaitIdle();
    m_Device->Ctx().ImmediateSubmit([&](VkCommandBuffer cmd) {
        vkCmdBuildAccelerationStructuresKHR(cmd, 1, &build, &pRange);
    });
    return true;
}

// ── Shared build ─────────────────────────────────────────────────────────────

void VulkanRHIAccelStruct::Build(VkAccelerationStructureBuildGeometryInfoKHR& build,
                                 uint32_t primitiveCount, const char* debugName) {
    VulkanContext& ctx       = m_Device->Ctx();
    VkDevice       dev       = ctx.Device();
    VmaAllocator   allocator = ctx.Allocator();

    VkAccelerationStructureBuildSizesInfoKHR sizes{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
    vkGetAccelerationStructureBuildSizesKHR(
        dev, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &build, &primitiveCount, &sizes);

    m_Storage = CreateBuffer(allocator, sizes.accelerationStructureSize,
                             kASStorageUsage, VMA_MEMORY_USAGE_AUTO);

    VkAccelerationStructureCreateInfoKHR ci{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
    ci.buffer = m_Storage.handle;
    ci.size   = sizes.accelerationStructureSize;
    ci.type   = build.type;
    VK_CHECK(vkCreateAccelerationStructureKHR(dev, &ci, nullptr, &m_Handle));

    if (debugName)
        ctx.SetObjectName(VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_KHR,
                          reinterpret_cast<uint64_t>(m_Handle), debugName);

    // The scratch OFFSET has a device-specific alignment that is not the
    // allocation's own alignment — over-allocate and align the address by hand.
    const VkDeviceSize scratchAlign =
        ctx.AccelStructProps().minAccelerationStructureScratchOffsetAlignment;
    // Sized for whichever mode needs more. updateScratchSize is usually the
    // smaller of the two, but the spec does not require it, and a refit reusing
    // this buffer would then run off the end.
    const VkDeviceSize scratchSize =
        std::max(sizes.buildScratchSize, sizes.updateScratchSize);
    m_Scratch = CreateBuffer(allocator, scratchSize + scratchAlign,
                             kScratchUsage, VMA_MEMORY_USAGE_AUTO);
    m_ScratchAddress = AlignUp(BufferDeviceAddress(dev, m_Scratch.handle), scratchAlign);

    build.dstAccelerationStructure  = m_Handle;
    build.scratchData.deviceAddress = m_ScratchAddress;

    VkAccelerationStructureBuildRangeInfoKHR range{};
    range.primitiveCount = m_TopLevel ? m_InstanceCount : primitiveCount;
    const VkAccelerationStructureBuildRangeInfoKHR* pRange = &range;

    ctx.ImmediateSubmit([&](VkCommandBuffer cmd) {
        vkCmdBuildAccelerationStructuresKHR(cmd, 1, &build, &pRange);
    });

    VkAccelerationStructureDeviceAddressInfoKHR addrInfo{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
    addrInfo.accelerationStructure = m_Handle;
    m_Address = vkGetAccelerationStructureDeviceAddressKHR(dev, &addrInfo);
}

VulkanRHIAccelStruct::~VulkanRHIAccelStruct() {
    VmaAllocator allocator = m_Device->Ctx().Allocator();
    if (m_Handle)
        vkDestroyAccelerationStructureKHR(m_Device->Ctx().Device(), m_Handle, nullptr);
    DestroyBuffer(allocator, m_Storage);
    DestroyBuffer(allocator, m_Scratch);
    DestroyBuffer(allocator, m_Instances);
}

} // namespace Diamond
