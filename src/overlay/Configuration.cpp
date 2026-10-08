// SPDX-License-Identifier: AGPL-3.0-only

#include "Configuration.h"
#include "OverlayLog.h"
#include "ProfileCodec.h"
#include "AttemptLifecycle.h"

#include <Windows.h>

#include <iostream>
#include <optional>

static const char *RegistryKey = "Software\\OpenVR-SpaceOverride";
static PersistenceState ProfileStorage, SettingsStorage;
static std::optional<CalibrationContext> BeforeProfileRead;

static void LogRegistryResult(const char *operation, const char *value, LSTATUS result)
{
	char *message = nullptr;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_IGNORE_INSERTS,
		0, result, LANG_USER_DEFAULT, reinterpret_cast<LPSTR>(&message), 0, nullptr);
	std::cerr << operation << " " << value << ": " << (message ? message : "Windows registry error") << " (" << result << ")\n";
	overlaylog::Write("configuration-error operation=", std::quoted(operation), " value=", std::quoted(value),
		" code=", result, " error=", std::quoted(overlaylog::WindowsError(result)));
	if (message)
		LocalFree(message);
}

static ReadResult ReadRegistryValue(const char *value, std::string &text)
{
	DWORD size = 0;
	LSTATUS result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, value, RRF_RT_REG_SZ, nullptr, nullptr, &size);
	if (result == ERROR_FILE_NOT_FOUND || result == ERROR_PATH_NOT_FOUND)
		return ReadResult::Absent;
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult("Reading", value, result);
		return ReadResult::Error;
	}
	text.resize(size);
	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, value, RRF_RT_REG_SZ, nullptr, text.data(), &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult("Reading", value, result);
		return ReadResult::Error;
	}
	text.resize(size);
	if (!text.empty() && text.back() == '\0')
		text.pop_back();
	return ReadResult::Present;
}

static bool WriteRegistryValue(const char *value, const std::string &text)
{
	HKEY key;
	LSTATUS result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_SET_VALUE, nullptr, &key, nullptr);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult("Opening for write", value, result);
		return false;
	}
	result = RegSetValueExA(key, value, 0, REG_SZ, reinterpret_cast<const BYTE *>(text.c_str()), static_cast<DWORD>(text.size() + 1));
	RegCloseKey(key);
	if (result != ERROR_SUCCESS)
		LogRegistryResult("Writing", value, result);
	return result == ERROR_SUCCESS;
}

void LoadProfile(CalibrationContext &ctx)
{
	if (!ProfileStorage.resolved)
	{
		if (!BeforeProfileRead)
			BeforeProfileRead = ctx;
		std::string text, error;
		auto result = ReadRegistryValue("Config", text);
		if (ApplyProfileRead(result, text, ctx, *BeforeProfileRead, ProfileStorage, error))
		{
			BeforeProfileRead.reset();
			overlaylog::Write("profile-read result=", result == ReadResult::Absent ? "absent" : "present",
				" binding=", ctx.trackerSerial.empty() ? "unbound" : "bound", " serial=", std::quoted(ctx.trackerSerial), " error=", std::quoted(error));
			if (!error.empty())
				std::cerr << "Reading Config: " << error << '\n';
		}
	}
	if (!SettingsStorage.resolved)
	{
		std::string text, error;
		auto result = ReadRegistryValue("Settings", text);
		if (ApplySettingsRead(result, text, ctx, SettingsStorage, error))
		{
			overlaylog::Write("settings-read result=", result == ReadResult::Absent ? "absent" : "present", " auto=", ctx.autoAcquire, " error=", std::quoted(error));
			if (!error.empty())
				std::cerr << "Reading Settings: " << error << '\n';
		}
	}
}

void SaveProfile(CalibrationContext &ctx, bool exiting)
{
	// An unresolved startup read must not replace an existing binding with default memory on exit.
	if (exiting && !ProfileStorage.resolved)
		return;
	ProfileStorage.Changed();
	ProfileStorage.Saved(WriteRegistryValue("Config", EncodeProfile(ctx)));
}

void SaveAppSettings(const CalibrationContext &ctx)
{
	SettingsStorage.Changed();
	SettingsStorage.Saved(WriteRegistryValue("Settings", EncodeSettings(ctx.autoAcquire)));
}

void RetryConfiguration(CalibrationContext &ctx)
{
	LoadProfile(ctx);
	if (ProfileStorage.dirty)
		ProfileStorage.Saved(WriteRegistryValue("Config", EncodeProfile(ctx)));
	if (SettingsStorage.dirty)
		SettingsStorage.Saved(WriteRegistryValue("Settings", EncodeSettings(ctx.autoAcquire)));
}

bool ProfileReadSucceeded()
{
	return ProfileStorage.resolved;
}

bool SettingsReadSucceeded()
{
	return SettingsStorage.resolved;
}
