// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "AcquisitionMath.h"
#include "Calibration.h"

#include <functional>

namespace acquisition
{

struct CalibrationSolution
{
	Eigen::Vector3d rotation = Eigen::Vector3d::Zero(), translation = Eigen::Vector3d::Zero();
	double scale = 1.0, targetModelScale = 1.0, hmdScale = 1.0;
	vr::HmdQuaternion_t relativeRotation{ 1, 0, 0, 0 };
	vr::HmdVector3d_t relativeTranslation{};
	ErrorStats holdout, rotationError;
};

CalibrationSolution SolveCalibration(const std::vector<Sample> &raw, double targetModelScale,
	const std::function<void(const std::string &)> &log = {});
bool AutomaticCalibrationSucceeded(const CalibrationSolution &solution);
bool AutomaticCalibrationAbandoned(size_t newerKeyframes, double rotationMedian);

}
