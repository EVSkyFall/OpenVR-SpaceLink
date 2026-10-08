// SPDX-License-Identifier: AGPL-3.0-only

#include "SystemFonts.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

#include <cstdint>
#include <cstring>
#include <string>

namespace
{
#ifdef _WIN32
	bool MapFont(const std::wstring &path, SystemFont &font)
	{
		HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
		if (file == INVALID_HANDLE_VALUE)
			return false;
		LARGE_INTEGER size{};
		HANDLE mapping = GetFileSizeEx(file, &size) && size.QuadPart > 0 ? CreateFileMappingW(file, nullptr, PAGE_READONLY, 0, 0, nullptr) : nullptr;
		CloseHandle(file);
		if (!mapping)
			return false;
		// The view is never unmapped: Dear ImGui reads glyphs from it on demand for as long as the UI runs,
		// and Windows only pages in the parts it reads.
		const void *view = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0);
		CloseHandle(mapping);
		if (!view)
			return false;
		font.data = static_cast<const unsigned char *>(view);
		font.size = static_cast<std::size_t>(size.QuadPart);
		return true;
	}
#endif
}

const std::vector<SystemFont> &SystemFallbackFonts()
{
	static const std::vector<SystemFont> fonts = [] {
		std::vector<SystemFont> found;
#ifdef _WIN32
		wchar_t windows[MAX_PATH] = {};
		const UINT length = GetSystemWindowsDirectoryW(windows, MAX_PATH);
		if (length == 0 || length >= MAX_PATH)
			return found;
		const std::wstring directory = std::wstring(windows, length) + L"\\Fonts\\";
		// One font per script, the first installed file of each group: Segoe UI (Latin, Greek and Cyrillic extensions),
		// Malgun Gothic (Korean), Yu Gothic or Meiryo (Japanese), Microsoft YaHei (Chinese).
		const std::vector<std::vector<const wchar_t *>> groups = {
			{ L"segoeui.ttf" },
			{ L"malgun.ttf" },
			{ L"YuGothR.ttc", L"meiryo.ttc" },
			{ L"msyh.ttc" },
		};
		for (const auto &group : groups)
			for (const wchar_t *name : group)
			{
				SystemFont font;
				if (MapFont(directory + name, font))
				{
					found.push_back(font);
					break;
				}
			}
#endif
		return found;
	}();
	return fonts;
}

float LineHeightInEms(const unsigned char *data, std::size_t size)
{
	auto u16 = [&](std::size_t at) -> uint32_t { return at + 2 <= size ? (data[at] << 8) | data[at + 1] : 0; };
	auto u32 = [&](std::size_t at) -> uint32_t { return at + 4 <= size ? (u16(at) << 16) | u16(at + 2) : 0; };
	// A font collection lists its faces first; the first face is the regular one.
	std::size_t face = size >= 16 && std::memcmp(data, "ttcf", 4) == 0 ? u32(12) : 0;
	const uint32_t tables = u16(face + 4);
	int unitsPerEm = 0, ascender = 0, descender = 0;
	for (uint32_t i = 0; i < tables; ++i)
	{
		const std::size_t record = face + 12 + i * 16;
		if (record + 16 > size)
			break;
		const std::size_t offset = u32(record + 8);
		if (std::memcmp(data + record, "head", 4) == 0)
			unitsPerEm = static_cast<int>(u16(offset + 18));
		else if (std::memcmp(data + record, "hhea", 4) == 0)
		{
			ascender = static_cast<int16_t>(u16(offset + 4));
			descender = static_cast<int16_t>(u16(offset + 6));
		}
	}
	return unitsPerEm > 0 && ascender > descender ? static_cast<float>(ascender - descender) / unitsPerEm : 0.0f;
}
