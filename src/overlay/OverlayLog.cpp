// SPDX-License-Identifier: AGPL-3.0-only

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

#include "OverlayLog.h"

#include <fstream>
#include <vector>

namespace overlaylog
{

static std::filesystem::path LogPath;

void Prepare(const std::filesystem::path &path) noexcept
{
	try
	{
		std::error_code error;
		auto size = std::filesystem::file_size(path, error);
		if (!error && ShouldRotate(size))
		{
			auto old = path.parent_path() / "overlay.old.log";
			MoveFileExW(path.c_str(), old.c_str(), MOVEFILE_REPLACE_EXISTING);
		}
	}
	catch (...) { }
}

void Initialize() noexcept
{
	try
	{
		DWORD size = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);
		std::vector<wchar_t> directory(size);
		if (size && GetEnvironmentVariableW(L"LOCALAPPDATA", directory.data(), size))
		{
			LogPath = std::filesystem::path(directory.data()) / "OpenVR-SpaceOverride" / "overlay.log";
			Prepare(LogPath);
		}
	}
	catch (...) { }
}

bool Append(const std::filesystem::path &path, std::string_view text) noexcept
{
	try
	{
		std::error_code error;
		std::filesystem::create_directories(path.parent_path(), error);
		std::ofstream file(path, std::ios::binary | std::ios::app);
		SYSTEMTIME now{};
		GetLocalTime(&now);
		file << std::setfill('0') << std::setw(2) << now.wHour << ':' << std::setw(2) << now.wMinute
			<< ':' << std::setw(2) << now.wSecond << '.' << std::setw(3) << now.wMilliseconds << ' ';
		for (char ch : text)
		{
			if (ch == '\n') file << "\\n";
			else if (ch == '\r') file << "\\r";
			else file.put(ch);
		}
		file.put('\n');
		file.close();
		return !file.fail();
	}
	catch (...) { return false; }
}

void WriteLine(std::string_view text) noexcept
{
	if (LogPath.empty()) Initialize();
	Append(LogPath, text);
}

std::string WindowsError(unsigned long code) noexcept
{
	wchar_t *message = nullptr;
	try
	{
		DWORD count = FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_IGNORE_INSERTS,
			nullptr, code, 0, reinterpret_cast<wchar_t *>(&message), 0, nullptr);
		int size = WideCharToMultiByte(CP_UTF8, 0, message, static_cast<int>(count), nullptr, 0, nullptr, nullptr);
		std::string result(size, '\0');
		if (size) WideCharToMultiByte(CP_UTF8, 0, message, static_cast<int>(count), result.data(), size, nullptr, nullptr);
		if (!size) result = std::to_string(code);
		LocalFree(message);
		message = nullptr;
		return result;
	}
	catch (...)
	{
		LocalFree(message);
		return {};
	}
}

}
