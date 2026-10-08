// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <filesystem>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string_view>

namespace overlaylog
{

constexpr bool ShouldRotate(uintmax_t bytes) noexcept { return bytes > 2 * 1024 * 1024; }
void Prepare(const std::filesystem::path &path) noexcept;
void Initialize() noexcept;
bool Append(const std::filesystem::path &path, std::string_view text) noexcept;
void WriteLine(std::string_view text) noexcept;
std::string WindowsError(unsigned long code) noexcept;

template<typename... Args>
void Write(const Args &...args) noexcept
{
	try
	{
		std::ostringstream line;
		line.imbue(std::locale::classic());
		line << std::boolalpha << std::fixed << std::setprecision(3);
		(line << ... << args);
		WriteLine(line.str());
	}
	catch (...) { }
}

}
