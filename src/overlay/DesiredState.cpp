// SPDX-License-Identifier: AGPL-3.0-only

#include "DesiredState.h"

#include <Geometry>
#include <algorithm>
#include <cstring>

uint32_t ResolveSerial(const std::vector<DeviceSnapshot> &devices, const std::string &serial)
{
	for (const auto &device : devices)
		if (!serial.empty() && device.serial == serial)
			return device.id;
	return vr::k_unTrackedDeviceIndexInvalid;
}

std::vector<uint32_t> ManualCandidates(const std::vector<DeviceSnapshot> &devices, const vr::TrackedDevicePose_t *poses)
{
	std::vector<uint32_t> candidates;
	for (const auto &device : devices)
		if (device.deviceClass == vr::TrackedDeviceClass_GenericTracker && device.id < vr::k_unMaxTrackedDeviceCount && poses[device.id].bPoseIsValid)
			candidates.push_back(device.id);
	return candidates;
}

static protocol::Request Packet(protocol::RequestType type)
{
	protocol::Request request;
	std::memset(&request, 0, sizeof request);
	request.type = type;
	return request;
}

std::vector<protocol::Request> DesiredState(const std::vector<DeviceSnapshot> &devices,
	const CalibrationContext &profile, bool sampling, uint32_t samplingTrackerID)
{
	std::vector<protocol::Request> commands;
	const uint32_t tracker = ResolveSerial(devices, profile.trackerSerial);
	const bool enabled = profile.validProfile && !profile.trackerSerial.empty() && profile.validRelativeOffset
		&& tracker < vr::k_unMaxTrackedDeviceCount && !sampling;
	Eigen::Vector3d radians = profile.calibratedRotation * EIGEN_PI / 180.0;
	Eigen::Quaterniond rotation = Eigen::AngleAxisd(radians(0), Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(radians(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(radians(2), Eigen::Vector3d::UnitX());
	vr::HmdQuaternion_t q{ rotation.w(), rotation.x(), rotation.y(), rotation.z() };
	vr::HmdVector3d_t t{ profile.calibratedTranslation.x() * 0.01, profile.calibratedTranslation.y() * 0.01, profile.calibratedTranslation.z() * 0.01 };
	// The old driver clears slamSync on disable, so apply the complete HMD packet first.
	auto hmd = Packet(protocol::RequestSetHmdTracker);
	hmd.setHmdTracker = { vr::k_unTrackedDeviceIndex_Hmd, enabled ? tracker : 0, enabled,
		profile.enableNative, profile.fallbackToSlam, profile.enableAngularVelocity, profile.predictionTime,
		profile.relativeRotation, profile.relativeTranslation, q, t, profile.calibratedScale, profile.hmdScale };
	commands.push_back(hmd);
	for (const auto &device : devices)
	{
		if (device.id >= vr::k_unMaxTrackedDeviceCount || device.deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;
		bool hmdDevice = device.id == vr::k_unTrackedDeviceIndex_Hmd || device.deviceClass == vr::TrackedDeviceClass_HMD;
		bool transform = !hmdDevice && profile.validProfile && device.trackingSystem == profile.targetTrackingSystem && device.id != tracker
			&& !(sampling && device.id == samplingTrackerID);
		auto request = Packet(protocol::RequestSetDeviceTransform);
		request.setDeviceTransform = { device.id, transform, transform ? t : vr::HmdVector3d_t{},
			transform ? q : vr::HmdQuaternion_t{ 1, 0, 0, 0 },
			transform ? profile.calibratedScale * device.modelScale / profile.targetModelScale : 1.0 };
		commands.push_back(request);
		auto sync = Packet(protocol::RequestSetSlamSync);
		sync.setSlamSync = { device.id, enabled && profile.continuousSync && !hmdDevice
			&& device.deviceClass != vr::TrackedDeviceClass_TrackingReference && device.trackingSystem != profile.targetTrackingSystem };
		commands.push_back(sync);
	}
	auto filter = Packet(protocol::RequestSetOneEuro);
	filter.setOneEuro = { profile.headFilterEnabled, profile.headFilterParams, profile.driftFilterParams };
	commands.push_back(filter);
	return commands;
}

void ObservationSpace::Applied(const protocol::Request &command)
{
	if (command.type == protocol::RequestSetHmdTracker)
	{
		haveConfiguration = true;
		native = command.setHmdTracker.native;
		hmdOverride = command.setHmdTracker.enabled;
	}
	else if (command.type == protocol::RequestSetDeviceTransform && command.setDeviceTransform.openVRID < transforms.size())
	{
		const auto &source = command.setDeviceTransform;
		auto &target = transforms[source.openVRID];
		target.known = true;
		target.enabled = source.enabled;
		if (source.updateRotation) target.rotation = source.rotation;
		if (source.updateTranslation) target.translation = source.translation;
		if (source.updateScale) target.scale = source.scale;
	}
}

std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> ObservationSpace::RawPoses(const vr::TrackedDevicePose_t *observed) const
{
	// For old unbound profiles with scale != 1, this inverse is approximate because the client cannot observe the driver's internal world offset.
	// Device model ratios differ by about 0.6%, so the remaining position error is typically a centimeter or two.
	std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> poses;
	std::copy_n(observed, poses.size(), poses.begin());
	for (size_t id = 0; id < poses.size(); ++id)
	{
		auto &pose = poses[id];
		const auto &transform = transforms[id];
		if (!haveConfiguration || hmdOverride || (!native && !transform.known))
		{
			pose.bPoseIsValid = false;
			continue;
		}
		if (native || !transform.enabled || !pose.bPoseIsValid)
			continue;
		const auto &q = transform.rotation;
		Eigen::Matrix3d inverse = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix().transpose();
		Eigen::Matrix3d rotation;
		Eigen::Vector3d position;
		for (int row = 0; row < 3; ++row)
		{
			for (int column = 0; column < 3; ++column)
				rotation(row, column) = pose.mDeviceToAbsoluteTracking.m[row][column];
			position(row) = pose.mDeviceToAbsoluteTracking.m[row][3] - transform.translation.v[row];
		}
		rotation = inverse * rotation;
		position = inverse * position / transform.scale;
		for (int row = 0; row < 3; ++row)
		{
			for (int column = 0; column < 3; ++column)
				pose.mDeviceToAbsoluteTracking.m[row][column] = static_cast<float>(rotation(row, column));
			pose.mDeviceToAbsoluteTracking.m[row][3] = static_cast<float>(position(row));
		}
	}
	return poses;
}
