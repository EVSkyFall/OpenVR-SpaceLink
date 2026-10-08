// SPDX-License-Identifier: AGPL-3.0-only

#include "SoftwareRenderer.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
	struct Texel
	{
		float r, g, b, a;
	};

	bool NearestSampling = false;

	void ResetRenderState(const ImDrawList *, const ImDrawCmd *) { NearestSampling = false; }
	void SetSamplerLinear(const ImDrawList *, const ImDrawCmd *) { NearestSampling = false; }
	void SetSamplerNearest(const ImDrawList *, const ImDrawCmd *) { NearestSampling = true; }

	Texel Fetch(ImTextureData *texture, int x, int y)
	{
		x = std::clamp(x, 0, texture->Width - 1);
		y = std::clamp(y, 0, texture->Height - 1);
		const auto *pixel = static_cast<const unsigned char *>(texture->GetPixelsAt(x, y));
		if (texture->Format == ImTextureFormat_Alpha8)
			return { 1.0f, 1.0f, 1.0f, pixel[0] / 255.0f };
		return { pixel[0] / 255.0f, pixel[1] / 255.0f, pixel[2] / 255.0f, pixel[3] / 255.0f };
	}

	Texel Mix(const Texel &a, const Texel &b, float t)
	{
		return { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t };
	}

	// Clamp-to-edge bilinear filtering, as the Vulkan backend's default sampler does.
	Texel Sample(ImTextureData *texture, float u, float v)
	{
		float x = u * texture->Width - 0.5f, y = v * texture->Height - 0.5f;
		if (NearestSampling)
			return Fetch(texture, static_cast<int>(std::floor(x + 0.5f)), static_cast<int>(std::floor(y + 0.5f)));
		int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
		float fx = x - x0, fy = y - y0;
		return Mix(Mix(Fetch(texture, x0, y0), Fetch(texture, x0 + 1, y0), fx),
			Mix(Fetch(texture, x0, y0 + 1), Fetch(texture, x0 + 1, y0 + 1), fx), fy);
	}

	// Positions snap to 1/256 pixel like a GPU's subpixel grid, so edge tests are exact integers
	// and an edge shared by two triangles covers each pixel exactly once.
	constexpr int64_t Subpixel = 256;

	struct Fixed
	{
		int64_t x, y;
	};

	Fixed Snap(float x, float y)
	{
		return { std::llround(x * Subpixel), std::llround(y * Subpixel) };
	}

	int64_t Edge(const Fixed &a, const Fixed &b, int64_t x, int64_t y)
	{
		return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
	}

	// Top-left fill rule for the orientation where interior edge values are positive.
	bool TopLeft(const Fixed &a, const Fixed &b)
	{
		return (a.y == b.y && b.x > a.x) || b.y < a.y;
	}

	uint8_t EncodeSrgb(float linear)
	{
		linear = std::clamp(linear, 0.0f, 1.0f);
		float encoded = linear <= 0.0031308f ? linear * 12.92f : 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
		return static_cast<uint8_t>(std::lround(encoded * 255.0f));
	}
}

SoftwareRenderer::SoftwareRenderer(int width, int height)
	: width(width), height(height), color(static_cast<size_t>(width) * height * 4)
{
}

void SoftwareRenderer::Install()
{
	ImGuiIO &io = ImGui::GetIO();
	io.BackendRendererName = "spacelink_ui_preview";
	io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset | ImGuiBackendFlags_RendererHasTextures;
	ImGuiPlatformIO &platform = ImGui::GetPlatformIO();
	platform.DrawCallback_ResetRenderState = ResetRenderState;
	platform.DrawCallback_SetSamplerLinear = SetSamplerLinear;
	platform.DrawCallback_SetSamplerNearest = SetSamplerNearest;
}

void SoftwareRenderer::Uninstall()
{
	for (ImTextureData *texture : ImGui::GetPlatformIO().Textures)
		if (texture->RefCount == 1)
		{
			texture->SetTexID(ImTextureID_Invalid);
			texture->SetStatus(ImTextureStatus_Destroyed);
		}
	ImGuiIO &io = ImGui::GetIO();
	io.BackendRendererName = nullptr;
	io.BackendFlags &= ~(ImGuiBackendFlags_RendererHasVtxOffset | ImGuiBackendFlags_RendererHasTextures);
}

// Textures are sampled straight from Dear ImGui's own pixel copy, so updates need no upload.
void SoftwareRenderer::UpdateTextures(ImDrawData *drawData)
{
	if (!drawData->Textures)
		return;
	for (ImTextureData *texture : *drawData->Textures)
	{
		if (texture->Status == ImTextureStatus_WantCreate)
		{
			texture->SetTexID(static_cast<ImTextureID>(reinterpret_cast<intptr_t>(texture)));
			texture->SetStatus(ImTextureStatus_OK);
		}
		else if (texture->Status == ImTextureStatus_WantUpdates)
			texture->SetStatus(ImTextureStatus_OK);
		else if (texture->Status == ImTextureStatus_WantDestroy)
		{
			texture->SetTexID(ImTextureID_Invalid);
			texture->SetStatus(ImTextureStatus_Destroyed);
		}
	}
}

void SoftwareRenderer::Render(ImDrawData *drawData)
{
	UpdateTextures(drawData);
	for (size_t i = 0; i < color.size(); i += 4)
	{
		color[i + 0] = color[i + 1] = color[i + 2] = 0.0f;
		color[i + 3] = 1.0f;
	}

	const ImVec2 origin = drawData->DisplayPos;
	for (const ImDrawList *list : drawData->CmdLists)
	{
		for (const ImDrawCmd &cmd : list->CmdBuffer)
		{
			if (cmd.UserCallback)
			{
				cmd.UserCallback(list, &cmd);
				continue;
			}

			// Same scissor rounding as the Vulkan backend.
			float minX = std::max(cmd.ClipRect.x - origin.x, 0.0f), minY = std::max(cmd.ClipRect.y - origin.y, 0.0f);
			float maxX = std::min(cmd.ClipRect.z - origin.x, static_cast<float>(width));
			float maxY = std::min(cmd.ClipRect.w - origin.y, static_cast<float>(height));
			if (maxX <= minX || maxY <= minY)
				continue;
			Clip clip;
			clip.x0 = static_cast<int>(minX);
			clip.y0 = static_cast<int>(minY);
			clip.x1 = clip.x0 + static_cast<int>(maxX - minX);
			clip.y1 = clip.y0 + static_cast<int>(maxY - minY);

			auto *texture = reinterpret_cast<ImTextureData *>(static_cast<intptr_t>(cmd.GetTexID()));
			auto load = [&](unsigned int index) {
				const ImDrawVert &vertex = list->VtxBuffer[cmd.VtxOffset + list->IdxBuffer[cmd.IdxOffset + index]];
				auto channel = [&](int shift) { return ((vertex.col >> shift) & 0xFF) / 255.0f; };
				return Vertex{ vertex.pos.x - origin.x, vertex.pos.y - origin.y, vertex.uv.x, vertex.uv.y,
					channel(IM_COL32_R_SHIFT), channel(IM_COL32_G_SHIFT), channel(IM_COL32_B_SHIFT), channel(IM_COL32_A_SHIFT) };
			};
			for (unsigned int i = 0; i + 2 < cmd.ElemCount; i += 3)
				DrawTriangle(load(i), load(i + 1), load(i + 2), clip, texture);
		}
	}
}

void SoftwareRenderer::DrawTriangle(Vertex v0, Vertex v1, Vertex v2, const Clip &clip, ImTextureData *texture)
{
	Fixed p0 = Snap(v0.x, v0.y), p1 = Snap(v1.x, v1.y), p2 = Snap(v2.x, v2.y);
	int64_t area = Edge(p0, p1, p2.x, p2.y);
	if (area == 0)
		return;
	if (area < 0)
	{
		std::swap(v1, v2);
		std::swap(p1, p2);
		area = -area;
	}

	int x0 = std::max(clip.x0, static_cast<int>(std::floor(std::min({ v0.x, v1.x, v2.x }))));
	int y0 = std::max(clip.y0, static_cast<int>(std::floor(std::min({ v0.y, v1.y, v2.y }))));
	int x1 = std::min(clip.x1, static_cast<int>(std::ceil(std::max({ v0.x, v1.x, v2.x }))) + 1);
	int y1 = std::min(clip.y1, static_cast<int>(std::ceil(std::max({ v0.y, v1.y, v2.y }))) + 1);
	const bool topLeft0 = TopLeft(p1, p2), topLeft1 = TopLeft(p2, p0), topLeft2 = TopLeft(p0, p1);

	for (int y = y0; y < y1; ++y)
	{
		for (int x = x0; x < x1; ++x)
		{
			const int64_t px = x * Subpixel + Subpixel / 2, py = y * Subpixel + Subpixel / 2;
			int64_t w0 = Edge(p1, p2, px, py), w1 = Edge(p2, p0, px, py), w2 = Edge(p0, p1, px, py);
			if (w0 < 0 || w1 < 0 || w2 < 0)
				continue;
			if ((w0 == 0 && !topLeft0) || (w1 == 0 && !topLeft1) || (w2 == 0 && !topLeft2))
				continue;

			float b0 = static_cast<float>(static_cast<double>(w0) / area);
			float b1 = static_cast<float>(static_cast<double>(w1) / area);
			float b2 = static_cast<float>(static_cast<double>(w2) / area);
			auto interpolate = [&](float Vertex::*field) { return v0.*field * b0 + v1.*field * b1 + v2.*field * b2; };
			Texel texel = texture ? Sample(texture, interpolate(&Vertex::u), interpolate(&Vertex::v)) : Texel{ 1.0f, 1.0f, 1.0f, 1.0f };
			float r = interpolate(&Vertex::r) * texel.r, g = interpolate(&Vertex::g) * texel.g;
			float b = interpolate(&Vertex::b) * texel.b, a = interpolate(&Vertex::a) * texel.a;

			float *dst = &color[(static_cast<size_t>(y) * width + x) * 4];
			dst[0] = r * a + dst[0] * (1.0f - a);
			dst[1] = g * a + dst[1] * (1.0f - a);
			dst[2] = b * a + dst[2] * (1.0f - a);
			dst[3] = a + dst[3] * (1.0f - a);
		}
	}
}

std::vector<uint8_t> SoftwareRenderer::Pixels() const
{
	std::vector<uint8_t> pixels(color.size());
	for (size_t i = 0; i < color.size(); i += 4)
	{
		pixels[i + 0] = EncodeSrgb(color[i + 0]);
		pixels[i + 1] = EncodeSrgb(color[i + 1]);
		pixels[i + 2] = EncodeSrgb(color[i + 2]);
		pixels[i + 3] = static_cast<uint8_t>(std::lround(std::clamp(color[i + 3], 0.0f, 1.0f) * 255.0f));
	}
	return pixels;
}
