// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx, bool exiting = false);
void RetryConfiguration(CalibrationContext &ctx);
void SaveAppSettings(const CalibrationContext &ctx);
bool ProfileReadSucceeded();
bool SettingsReadSucceeded();
