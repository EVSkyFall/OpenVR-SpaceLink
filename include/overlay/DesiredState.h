// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"
#include <array>

struct DeviceSnapshot
{
	uint32_t id = 0;
	vr::ETrackedDeviceClass deviceClass = vr::TrackedDeviceClass_Invalid;
	vr::ETrackedControllerRole role = vr::TrackedControllerRole_Invalid;
	std::string serial, trackingSystem;
	bool connected = false, poseValid = false;
	double modelScale = 1.0;
	std::string controllerType;
};

uint32_t ResolveSerial(const std::vector<DeviceSnapshot> &devices, const std::string &serial);
std::vector<uint32_t> ManualCandidates(const std::vector<DeviceSnapshot> &devices, const vr::TrackedDevicePose_t *poses);
std::vector<protocol::Request> DesiredState(const std::vector<DeviceSnapshot> &devices,
	const CalibrationContext &profile, bool sampling, uint32_t samplingTrackerID = vr::k_unTrackedDeviceIndexInvalid);

class ObservationSpace
{
public:
	void Applied(const protocol::Request &command);
	bool HasConfiguration() const { return haveConfiguration; }
	std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> RawPoses(const vr::TrackedDevicePose_t *observed) const;

private:
	struct Transform
	{
		bool known = false, enabled = false;
		vr::HmdQuaternion_t rotation{ 1, 0, 0, 0 };
		vr::HmdVector3d_t translation{};
		double scale = 1;
	};
	std::array<Transform, vr::k_unMaxTrackedDeviceCount> transforms{};
	bool haveConfiguration = false, native = false, hmdOverride = false;
};
