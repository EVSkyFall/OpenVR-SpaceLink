// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <imgui.h>

#include <cstdint>
#include <vector>

// Rasterizes Dear ImGui draw data on the CPU the way the overlay's Vulkan path shows it:
// vertex colors and texels are linear, blending happens in linear space, and the result
// is stored as sRGB, like the R8G8B8A8_SRGB render target the overlay draws into.
class SoftwareRenderer
{
public:
	SoftwareRenderer(int width, int height);

	static void Install();
	static void Uninstall();
	static void UpdateTextures(ImDrawData *drawData);

	void Render(ImDrawData *drawData);
	std::vector<uint8_t> Pixels() const;

private:
	struct Vertex
	{
		float x, y, u, v;
		float r, g, b, a;
	};

	struct Clip
	{
		int x0, y0, x1, y1;
	};

	void DrawTriangle(Vertex v0, Vertex v1, Vertex v2, const Clip &clip, ImTextureData *texture);

	int width, height;
	std::vector<float> color;
};
