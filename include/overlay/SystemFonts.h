// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstddef>
#include <vector>

// A font file that ships with Windows, mapped read-only for the life of the process.
struct SystemFont
{
	const unsigned char *data = nullptr;
	std::size_t size = 0;
};

// Windows fonts for text the bundled fonts cannot show, such as the Korean, Japanese or
// Chinese error messages Windows writes, in fallback order. Fonts that are not installed are left out.
const std::vector<SystemFont> &SystemFallbackFonts();

// Ascender minus descender in ems, which is what Dear ImGui scales a font's pixel size to; 0 if unreadable.
float LineHeightInEms(const unsigned char *data, std::size_t size);
