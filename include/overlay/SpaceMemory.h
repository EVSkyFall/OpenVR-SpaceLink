// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"
#include <optional>

namespace spacememory
{

struct Alignment
{
	Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
	Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct Basis
{
	vr::HmdQuaternion_t calibrationRotation{}, offsetRotation{};
	vr::HmdVector3d_t calibrationTranslation{}, offsetTranslation{};
	double calibrationScale = 1, hmdScale = 1;
	std::string trackerSerial;
};

struct StoredAlignment
{
	Alignment alignment;
	Basis basis;
	uint64_t session = 0;
};

enum class RestoreResult { Applied, SameSession, BasisMismatch, NotBound, Absent };
struct Restoration
{
	RestoreResult result = RestoreResult::Absent;
	Alignment alignment;
};

Alignment Fold(const protocol::DriftState &state);
std::optional<StoredAlignment> Capture(const protocol::DriftState &state, const CalibrationContext &profile);
Restoration Restore(const std::optional<StoredAlignment> &stored, const CalibrationContext &profile, uint64_t currentSession);
const char *ResultName(RestoreResult result);
std::string Encode(const StoredAlignment &stored);
std::optional<StoredAlignment> Decode(const std::string &json);
bool ShouldWrite(const std::optional<StoredAlignment> &latest, const std::optional<StoredAlignment> &written,
	double now, double lastWrite, bool exiting = false);

}
