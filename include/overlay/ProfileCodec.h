// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"

struct PersistenceState;
enum class ReadResult { Present, Absent, Error };

bool DecodeProfile(const std::string &json, CalibrationContext &profile, std::string &error);
void PreserveChangedSettings(CalibrationContext &loaded, const CalibrationContext &current, const CalibrationContext &beforeRead);
std::string EncodeProfile(const CalibrationContext &profile);
bool DecodeSettings(const std::string &json, bool &autoAcquire, std::string &error);
std::string EncodeSettings(bool autoAcquire);
bool ApplyProfileRead(ReadResult result, const std::string &json, CalibrationContext &profile,
	const CalibrationContext &beforeRead, PersistenceState &storage, std::string &error);
bool ApplySettingsRead(ReadResult result, const std::string &json, CalibrationContext &profile,
	PersistenceState &storage, std::string &error);
