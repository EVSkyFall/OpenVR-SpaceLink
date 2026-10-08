// SPDX-License-Identifier: AGPL-3.0-only

#ifdef _WIN32
#include <Windows.h>
#endif

#include "ImGuiWindow.h"

#include <imgui.h>
#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_vulkan.h>

#include "imgui_impl_openvr.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>

ImGuiWindow::ImGuiWindow()
{
    window_ = nullptr;
    window_data_ = {};
    width_ = 0;
    height_ = 0;
    window_shown_ = false;
    window_minimized_ = false;
}

auto ImGuiWindow::Initialize(VulkanRenderer*& renderer, VrOverlay*& overlay, const char* name, int width, int height) -> void
{
    width_ = width;
    height_ = height;

    auto sdl_window_flags = SDL_WINDOW_VULKAN | SDL_WINDOW_HIDDEN | SDL_WINDOW_MOUSE_FOCUS;
    window_ = SDL_CreateWindow(name, width, height, sdl_window_flags);
    if (window_ == nullptr) {
#ifdef _WIN32
        MessageBoxA(NULL, SDL_GetError(), "SpaceLink", MB_OK);
#else
        printf("SDL_CreateWindow(): %s\n", SDL_GetError());
#endif
        return;
    }

    VkSurfaceKHR surface = {};
    if (SDL_Vulkan_CreateSurface(window_, renderer->Instance(), renderer->Allocator(), &surface) == 0) {
#ifdef _WIN32
        MessageBoxA(NULL, SDL_GetError(), "SpaceLink", MB_OK);
#else
        printf("SDL_Vulkan_CreateSurface(): %s\n", SDL_GetError());
#endif
        return;
    }

    Vulkan_Window* window = &window_data_;
    renderer->SetupWindow(window, surface, width, height);

    SDL_SetWindowPosition(window_, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_CaptureMouse(false);

    window_shown_ = false;

    IMGUI_CHECKVERSION();

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_IsSRGB; // NOTE: ImGuiConfigFlags_IsSRGB is not used by ImGui, used to communicate state.

    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(static_cast<float>(width), static_cast<float>(height));

    m_userInterface_.Setup();

    VkSurfaceFormatKHR render_format =
    {
        .format = VK_FORMAT_R8G8B8A8_SRGB,
        .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
    };

    VkPipelineRenderingCreateInfoKHR pipeline_rendering_create_info =
    {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO_KHR,
        .viewMask = 0,
        .colorAttachmentCount = 1,
        .pColorAttachmentFormats = &render_format.format,
        .depthAttachmentFormat = VK_FORMAT_UNDEFINED,
        .stencilAttachmentFormat = VK_FORMAT_UNDEFINED,
    };

    ImGui_ImplVulkan_InitInfo init_info = {
        .ApiVersion = VK_API_VERSION_1_3,
        .Instance = renderer->Instance(),
        .PhysicalDevice = renderer->PhysicalDevice(),
        .Device = renderer->Device(),
        .QueueFamily = renderer->QueueFamily(),
        .Queue = renderer->Queue(),
        .DescriptorPool = renderer->DescriptorPool(),
        .MinImageCount = renderer->MinimumConcurrentImageCount(),
        .ImageCount = window->image_count,
        .PipelineCache = renderer->PipelineCache(),
        .PipelineInfoMain = {
            .RenderPass = VK_NULL_HANDLE,
            .Subpass = 0,
            .MSAASamples = VK_SAMPLE_COUNT_1_BIT,
            .PipelineRenderingCreateInfo = pipeline_rendering_create_info,
        },
        .UseDynamicRendering = true,
        .Allocator = renderer->Allocator(),
        .CheckVkResultFn = nullptr,
    };

    ImGui_ImplSDL3_InitForVulkan(window_);
    ImGui_ImplVulkan_Init(&init_info);

    ImGui_ImplOpenVR_InitInfo openvr_init_info =
    {
        .handle = overlay->Handle(),
        .width = width,
        .height = height
    };

    ImGui_ImplOpenVR_Init(&openvr_init_info);

    renderer->SetupRenderTarget(width, height, render_format);
}

auto ImGuiWindow::Show() -> void
{
    SDL_ShowWindow(window_);
    SDL_RestoreWindow(window_);

    window_shown_ = true;
}

auto ImGuiWindow::SetMinimizedFromEvent(bool state) -> void
{
    window_minimized_ = state;
}

auto ImGuiWindow::Hide() -> void
{
    SDL_HideWindow(window_);

    window_shown_ = false;
}

auto ImGuiWindow::Draw(bool dashboardVisible) -> void
{
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui_ImplOpenVR_NewFrame();

    ImGui::GetIO().DisplaySize = ImVec2(static_cast<float>(width_), static_cast<float>(height_));

    ImGui::NewFrame();

    m_userInterface_.Render(dashboardVisible);

    ImGui::Render();
}

auto ImGuiWindow::Destroy(VulkanRenderer*& renderer) -> void
{
    ImGui_ImplOpenVR_Shutdown();
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    SDL_DestroyWindow(window_);
}
