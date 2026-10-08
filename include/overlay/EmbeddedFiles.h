// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstddef>

// Built from resources/fonts by CMake (cmake/EmbedFile.cmake).
#define EMBEDDED_FONT(name) extern const unsigned char name[]; extern const std::size_t name##_size;
EMBEDDED_FONT(Font_Geist_Regular)
EMBEDDED_FONT(Font_Geist_Medium)
EMBEDDED_FONT(Font_Geist_SemiBold)
EMBEDDED_FONT(Font_GeistMono_Regular)
EMBEDDED_FONT(Font_Phosphor)
#undef EMBEDDED_FONT
