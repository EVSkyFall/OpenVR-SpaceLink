// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "DesiredState.h"
#include "AcquisitionMath.h"

class ManualDetection
{
public:
	enum class Event { Waiting, Collecting, HeadsetPaused, TrackerPaused, Selected };
	struct Result { Event event; uint32_t id = vr::k_unTrackedDeviceIndexInvalid; std::string serial; };
	Result Observe(double time, const std::vector<DeviceSnapshot> &devices, const vr::TrackedDevicePose_t *poses);
	void Resolve(const std::vector<DeviceSnapshot> &devices);
	bool Started() const { return started; }
	size_t Progress() const { return hmdSpeeds.size(); }
	const std::vector<std::string> &Serials() const { return serials; }
	const std::vector<double> &Speeds(size_t candidate) const { return candidateSpeeds[candidate]; }

private:
	std::vector<std::string> serials;
	std::vector<uint32_t> candidates;
	std::vector<std::vector<double>> candidateSpeeds;
	std::vector<double> hmdSpeeds;
	std::vector<Eigen::Matrix3d> prevRot;
	std::vector<bool> candidateHavePrev;
	bool started = false, havePrev = false;
	double prevTime = 0;
	void RestartEvidence();
};
