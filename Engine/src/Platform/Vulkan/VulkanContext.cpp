#include "Platform/Vulkan/VulkanContext.h"

#define GLFW_INCLUDE_NONE          // we provide Vulkan via volk, not GLFW's copy
#include <GLFW/glfw3.h>

#include <vector>
#include <set>
#include <cstring>
#include <array>

namespace Diamond {

namespace {

// Validation is a pure development aid with non-trivial CPU cost, so it is
// compiled out entirely in release builds.
#ifdef NDEBUG
constexpr bool kEnableValidationByDefault = false;
#else
constexpr bool kEnableValidationByDefault = true;
#endif

constexpr const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

// Required device extensions. Presentation is the only hard requirement at M0;
// dynamic rendering / sync2 / timeline semaphores are core in Vulkan 1.3 and so
// are requested as *features*, not extensions, below.
constexpr std::array kRequiredDeviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
};

VKAPI_ATTR VkBool32 VKAPI_CALL DebugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT      severity,
    VkDebugUtilsMessageTypeFlagsEXT             /*type*/,
    const VkDebugUtilsMessengerCallbackDataEXT* data,
    void*                                       /*user*/) {
    if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
        spdlog::error("[Vulkan] {}", data->pMessage);
    else if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
        spdlog::warn("[Vulkan] {}", data->pMessage);
    else
        spdlog::info("[Vulkan] {}", data->pMessage);
    return VK_FALSE; // never abort the offending Vulkan call
}

void PopulateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& ci) {
    ci = {};
    ci.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    ci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
                       | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    ci.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
                   | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
                   | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    ci.pfnUserCallback = DebugCallback;
}

bool ValidationLayerAvailable() {
    uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    for (const auto& l : layers)
        if (std::strcmp(l.layerName, kValidationLayer) == 0) return true;
    return false;
}

QueueFamilyIndices FindQueueFamilies(VkPhysicalDevice device, VkSurfaceKHR surface) {
    QueueFamilyIndices indices;

    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());

    for (uint32_t i = 0; i < count; ++i) {
        const VkQueueFlags flags = families[i].queueFlags;

        if ((flags & VK_QUEUE_GRAPHICS_BIT) && indices.graphics == UINT32_MAX)
            indices.graphics = i;

        // Prefer a transfer-capable family that is NOT graphics — a dedicated
        // DMA queue lets uploads overlap rendering later.
        if ((flags & VK_QUEUE_TRANSFER_BIT) && !(flags & VK_QUEUE_GRAPHICS_BIT))
            indices.transfer = i;

        VkBool32 presentSupport = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);
        if (presentSupport && indices.present == UINT32_MAX)
            indices.present = i;
    }

    if (indices.transfer == UINT32_MAX) indices.transfer = indices.graphics;
    return indices;
}

bool DeviceSupportsExtensions(VkPhysicalDevice device) {
    uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> available(count);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, available.data());

    for (const char* required : kRequiredDeviceExtensions) {
        bool found = false;
        for (const auto& ext : available)
            if (std::strcmp(ext.extensionName, required) == 0) { found = true; break; }
        if (!found) return false;
    }
    return true;
}

// Vulkan 1.3 / 1.2 features the renderer relies on. Queried during selection and
// re-enabled (identically) at device creation.
bool DeviceSupportsRequiredFeatures(VkPhysicalDevice device) {
    VkPhysicalDeviceVulkan13Features f13{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES };
    VkPhysicalDeviceVulkan12Features f12{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES };
    f12.pNext = &f13;
    VkPhysicalDeviceFeatures2 f2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
    f2.pNext = &f12;
    vkGetPhysicalDeviceFeatures2(device, &f2);

    return f13.dynamicRendering && f13.synchronization2
        && f12.timelineSemaphore && f12.bufferDeviceAddress;
}

bool DeviceSuitable(VkPhysicalDevice device, VkSurfaceKHR surface) {
    return FindQueueFamilies(device, surface).IsComplete()
        && DeviceSupportsExtensions(device)
        && DeviceSupportsRequiredFeatures(device);
}

} // namespace

void VulkanContext::Init(GLFWwindow* window) {
    VK_CHECK(volkInitialize());          // loads vkGetInstanceProcAddr + global entry points

    CreateInstance();
    volkLoadInstance(m_Instance);        // instance-level entry points

    if (m_ValidationEnabled) CreateDebugMessenger();

    CreateSurface(window);
    SelectPhysicalDevice();
    CreateLogicalDevice();
    volkLoadDevice(m_Device);            // device-level entry points — direct dispatch, no trampoline

    CreateAllocator();
    CreateImmediateContext();

    spdlog::info("[Vulkan] Context ready on '{}' (Vulkan {}.{}.{})",
                 m_DeviceProps.deviceName,
                 VK_API_VERSION_MAJOR(m_DeviceProps.apiVersion),
                 VK_API_VERSION_MINOR(m_DeviceProps.apiVersion),
                 VK_API_VERSION_PATCH(m_DeviceProps.apiVersion));
}

void VulkanContext::CreateInstance() {
    m_ValidationEnabled = kEnableValidationByDefault && ValidationLayerAvailable();
    if (kEnableValidationByDefault && !m_ValidationEnabled)
        spdlog::warn("[Vulkan] validation requested but {} is not installed — running without it",
                     kValidationLayer);

    VkApplicationInfo app{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
    app.pApplicationName = "DiamondEngine";
    app.pEngineName      = "DiamondEngine";
    app.apiVersion       = VK_API_VERSION_1_3;

    // GLFW tells us which instance extensions it needs for surface creation.
    uint32_t glfwExtCount = 0;
    const char** glfwExt = glfwGetRequiredInstanceExtensions(&glfwExtCount);
    std::vector<const char*> extensions(glfwExt, glfwExt + glfwExtCount);
    if (m_ValidationEnabled) extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

    VkInstanceCreateInfo ci{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
    ci.pApplicationInfo        = &app;
    ci.enabledExtensionCount   = static_cast<uint32_t>(extensions.size());
    ci.ppEnabledExtensionNames = extensions.data();

    // A messenger chained into pNext captures errors during instance creation
    // itself (before the persistent messenger exists).
    VkDebugUtilsMessengerCreateInfoEXT dbg{};
    if (m_ValidationEnabled) {
        ci.enabledLayerCount   = 1;
        ci.ppEnabledLayerNames = &kValidationLayer;
        PopulateDebugMessengerCreateInfo(dbg);
        ci.pNext = &dbg;
    }

    VK_CHECK(vkCreateInstance(&ci, nullptr, &m_Instance));
}

void VulkanContext::CreateDebugMessenger() {
    VkDebugUtilsMessengerCreateInfoEXT ci{};
    PopulateDebugMessengerCreateInfo(ci);
    VK_CHECK(vkCreateDebugUtilsMessengerEXT(m_Instance, &ci, nullptr, &m_DebugMessenger));
}

void VulkanContext::CreateSurface(GLFWwindow* window) {
    VK_CHECK(glfwCreateWindowSurface(m_Instance, window, nullptr, &m_Surface));
}

void VulkanContext::SelectPhysicalDevice() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(m_Instance, &count, nullptr);
    if (count == 0) {
        spdlog::critical("[Vulkan] no Vulkan-capable GPUs found");
        std::abort();
    }
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(m_Instance, &count, devices.data());

    // Prefer a discrete GPU; fall back to the first suitable device of any type.
    VkPhysicalDevice fallback = VK_NULL_HANDLE;
    for (VkPhysicalDevice dev : devices) {
        if (!DeviceSuitable(dev, m_Surface)) continue;

        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(dev, &props);
        if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            m_PhysicalDevice = dev;
            m_DeviceProps    = props;
            break;
        }
        if (fallback == VK_NULL_HANDLE) { fallback = dev; m_DeviceProps = props; }
    }
    if (m_PhysicalDevice == VK_NULL_HANDLE) m_PhysicalDevice = fallback;

    if (m_PhysicalDevice == VK_NULL_HANDLE) {
        spdlog::critical("[Vulkan] no GPU meets the renderer's requirements "
                         "(swapchain, dynamic rendering, sync2, timeline semaphores)");
        std::abort();
    }
    m_Queues = FindQueueFamilies(m_PhysicalDevice, m_Surface);
}

void VulkanContext::CreateLogicalDevice() {
    // One create-info per *unique* family (graphics/present/transfer may overlap).
    const std::set<uint32_t> uniqueFamilies = {
        m_Queues.graphics, m_Queues.present, m_Queues.transfer
    };
    const float priority = 1.0f;
    std::vector<VkDeviceQueueCreateInfo> queueInfos;
    for (uint32_t family : uniqueFamilies) {
        VkDeviceQueueCreateInfo qi{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
        qi.queueFamilyIndex = family;
        qi.queueCount       = 1;
        qi.pQueuePriorities = &priority;
        queueInfos.push_back(qi);
    }

    // Re-enable exactly the features checked in DeviceSupportsRequiredFeatures.
    VkPhysicalDeviceVulkan13Features f13{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES };
    f13.dynamicRendering = VK_TRUE;
    f13.synchronization2 = VK_TRUE;

    VkPhysicalDeviceVulkan12Features f12{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES };
    f12.timelineSemaphore   = VK_TRUE;
    f12.bufferDeviceAddress = VK_TRUE;   // enables VMA buffer-device-address path
    f12.pNext = &f13;

    VkPhysicalDeviceFeatures2 f2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
    f2.pNext = &f12;

    VkDeviceCreateInfo ci{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    ci.pNext                   = &f2;    // features via pNext chain; pEnabledFeatures stays null
    ci.queueCreateInfoCount    = static_cast<uint32_t>(queueInfos.size());
    ci.pQueueCreateInfos       = queueInfos.data();
    ci.enabledExtensionCount   = static_cast<uint32_t>(kRequiredDeviceExtensions.size());
    ci.ppEnabledExtensionNames = kRequiredDeviceExtensions.data();

    VK_CHECK(vkCreateDevice(m_PhysicalDevice, &ci, nullptr, &m_Device));

    vkGetDeviceQueue(m_Device, m_Queues.graphics, 0, &m_GraphicsQueue);
    vkGetDeviceQueue(m_Device, m_Queues.present,  0, &m_PresentQueue);
    vkGetDeviceQueue(m_Device, m_Queues.transfer, 0, &m_TransferQueue);
}

void VulkanContext::CreateAllocator() {
    VmaAllocatorCreateInfo ci{};
    ci.physicalDevice   = m_PhysicalDevice;
    ci.device           = m_Device;
    ci.instance         = m_Instance;
    ci.vulkanApiVersion = VK_API_VERSION_1_3;
    ci.flags            = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;

    // We built with VK_NO_PROTOTYPES, so VMA can't reference Vulkan symbols
    // directly — hand it volk's loaded function pointers.
    VmaVulkanFunctions vkFns{};
    VK_CHECK(vmaImportVulkanFunctionsFromVolk(&ci, &vkFns));
    ci.pVulkanFunctions = &vkFns;

    VK_CHECK(vmaCreateAllocator(&ci, &m_Allocator));
}

void VulkanContext::CreateImmediateContext() {
    VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    poolInfo.flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    poolInfo.queueFamilyIndex = m_Queues.graphics;
    VK_CHECK(vkCreateCommandPool(m_Device, &poolInfo, nullptr, &m_ImmediatePool));

    VkFenceCreateInfo fenceInfo{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    VK_CHECK(vkCreateFence(m_Device, &fenceInfo, nullptr, &m_ImmediateFence));
}

void VulkanContext::ImmediateSubmit(const std::function<void(VkCommandBuffer)>& record) {
    VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    allocInfo.commandPool        = m_ImmediatePool;
    allocInfo.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VK_CHECK(vkAllocateCommandBuffers(m_Device, &allocInfo, &cmd));

    VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(cmd, &begin));
    record(cmd);
    VK_CHECK(vkEndCommandBuffer(cmd));

    VkCommandBufferSubmitInfo cmdInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
    cmdInfo.commandBuffer = cmd;
    VkSubmitInfo2 submit{ VK_STRUCTURE_TYPE_SUBMIT_INFO_2 };
    submit.commandBufferInfoCount = 1;
    submit.pCommandBufferInfos    = &cmdInfo;
    VK_CHECK(vkQueueSubmit2(m_GraphicsQueue, 1, &submit, m_ImmediateFence));

    VK_CHECK(vkWaitForFences(m_Device, 1, &m_ImmediateFence, VK_TRUE, UINT64_MAX));
    VK_CHECK(vkResetFences(m_Device, 1, &m_ImmediateFence));
    // Frees the command buffer back to the pool for the next immediate submit.
    VK_CHECK(vkResetCommandPool(m_Device, m_ImmediatePool, 0));
}

void VulkanContext::Shutdown() {
    // Reverse creation order. Caller is responsible for having destroyed all
    // swapchain/frame resources first (vkDeviceWaitIdle before this).
    if (m_ImmediateFence) { vkDestroyFence(m_Device, m_ImmediateFence, nullptr); m_ImmediateFence = VK_NULL_HANDLE; }
    if (m_ImmediatePool)  { vkDestroyCommandPool(m_Device, m_ImmediatePool, nullptr); m_ImmediatePool = VK_NULL_HANDLE; }
    if (m_Allocator) { vmaDestroyAllocator(m_Allocator); m_Allocator = VK_NULL_HANDLE; }
    if (m_Device)    { vkDestroyDevice(m_Device, nullptr); m_Device = VK_NULL_HANDLE; }
    if (m_Surface)   { vkDestroySurfaceKHR(m_Instance, m_Surface, nullptr); m_Surface = VK_NULL_HANDLE; }
    if (m_DebugMessenger) {
        vkDestroyDebugUtilsMessengerEXT(m_Instance, m_DebugMessenger, nullptr);
        m_DebugMessenger = VK_NULL_HANDLE;
    }
    if (m_Instance)  { vkDestroyInstance(m_Instance, nullptr); m_Instance = VK_NULL_HANDLE; }
}

} // namespace Diamond
