// SPDX-License-Identifier: AGPL-3.0-only

// Draws the overlay UI in a set of representative states and saves each as a PNG,
// without SteamVR, the driver or a GPU.
// Usage: UiPreview <output folder> [scene...] writes <output folder>/<scene>.png.
// UiPreview --check clicks through every control instead.

#include "PreviewState.h"
#include "SoftwareRenderer.h"
#include "UserInterface.h"

#include <imgui.h>
#include <SDL3/SDL.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

int RunChecks();

namespace
{
	const int Width = 1200, Height = 800;
	// Enough frames for popups to open and the modal dimming to finish fading in.
	const int SettleFrames = 40;

	bool SavePng(const std::vector<uint8_t> &pixels, const std::filesystem::path &path)
	{
		SDL_Surface *surface = SDL_CreateSurfaceFrom(Width, Height, SDL_PIXELFORMAT_RGBA32, const_cast<uint8_t *>(pixels.data()), Width * 4);
		if (!surface)
			return false;
		bool saved = SDL_SavePNG(surface, path.string().c_str());
		SDL_DestroySurface(surface);
		return saved;
	}

	bool RenderScene(const preview::Scene &scene, const std::filesystem::path &path)
	{
		preview::Reset();
		scene.setup();

		ImGui::CreateContext();
		ImGuiIO &io = ImGui::GetIO();
		io.IniFilename = nullptr;
		io.ConfigFlags |= ImGuiConfigFlags_IsSRGB;
		io.DisplaySize = ImVec2(static_cast<float>(Width), static_cast<float>(Height));
		io.DeltaTime = 1.0f / 90.0f;
		SoftwareRenderer::Install();

		UserInterface ui;
		ui.Setup();
		ui.ShowPage(scene.page);
		if (scene.progress)
			ui.ShowCalibrationProgress();

		for (int frame = 0; frame < SettleFrames; ++frame)
		{
			ImGui::NewFrame();
			ui.Render(scene.overlay);
			ImGui::Render();
			SoftwareRenderer::UpdateTextures(ImGui::GetDrawData());
		}

		SoftwareRenderer renderer(Width, Height);
		renderer.Render(ImGui::GetDrawData());
		bool saved = SavePng(renderer.Pixels(), path);
		if (!saved)
			fprintf(stderr, "%s: %s\n", path.string().c_str(), SDL_GetError());

		SoftwareRenderer::Uninstall();
		ImGui::DestroyContext();
		return saved;
	}
}

int main(int argc, char **argv)
{
	if (argc > 1 && std::string(argv[1]) == "--check")
		return RunChecks();

	std::filesystem::path output = argc > 1 ? argv[1] : "ui-preview";
	std::vector<std::string> only(argv + std::min(argc, 2), argv + argc);
	std::filesystem::create_directories(output);

	int failed = 0;
	for (const auto &scene : preview::Scenes())
	{
		if (!only.empty() && std::find(only.begin(), only.end(), scene.name) == only.end())
			continue;
		auto path = output / (scene.name + ".png");
		if (RenderScene(scene, path))
			printf("%s\n", path.string().c_str());
		else
			++failed;
	}
	return failed ? 1 : 0;
}
