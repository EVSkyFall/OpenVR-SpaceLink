// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"
#include "SpaceMemory.h"
#include "ProfileCodec.h"

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx, bool exiting = false);
void RetryConfiguration(CalibrationContext &ctx);
void SaveAppSettings(const CalibrationContext &ctx);
bool ProfileReadSucceeded();
bool SettingsReadSucceeded();
ReadResult LoadLastSpace(std::optional<spacememory::StoredAlignment> &stored);
bool SaveLastSpace(const spacememory::StoredAlignment &stored);
bool DeleteLastSpace();
