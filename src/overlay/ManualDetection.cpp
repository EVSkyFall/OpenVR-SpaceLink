// SPDX-License-Identifier: AGPL-3.0-only

#include "ManualDetection.h"

#include <algorithm>
#include <cmath>

static double AngularSpeedBetween(const Eigen::Matrix3d &cur, const Eigen::Matrix3d &prev, double dt)
{
	Eigen::Matrix3d delta = cur * prev.transpose();
	double c = (delta(0,0) + delta(1,1) + delta(2,2) - 1.0) / 2.0;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	return acos(c) / dt;
}

static double PearsonCorrelation(const std::vector<double> &a, const std::vector<double> &b)
{
	if (a.size() != b.size() || a.empty())
		return 0.0;

	double meanA = 0, meanB = 0;
	for (size_t i = 0; i < a.size(); i++) { meanA += a[i]; meanB += b[i]; }
	meanA /= a.size();
	meanB /= b.size();

	double cov = 0, varA = 0, varB = 0;
	for (size_t i = 0; i < a.size(); i++)
	{
		double da = a[i] - meanA, db = b[i] - meanB;
		cov += da * db;
		varA += da * da;
		varB += db * db;
	}

	if (varA < 1e-9 || varB < 1e-9)
		return 0.0;

	return cov / std::sqrt(varA * varB);
}

void ManualDetection::RestartEvidence()
{
	candidateSpeeds.assign(candidates.size(), {});
	hmdSpeeds.clear();
	prevRot.clear();
	candidateHavePrev.assign(candidates.size(), false);
	havePrev = false;
}

void ManualDetection::Resolve(const std::vector<DeviceSnapshot> &devices)
{
	for (size_t i = 0; i < serials.size(); ++i)
		candidates[i] = ResolveSerial(devices, serials[i]);
}

ManualDetection::Result ManualDetection::Observe(double time, const std::vector<DeviceSnapshot> &devices, const vr::TrackedDevicePose_t *poses)
{
	acquisition::Pose hmd(poses[0].mDeviceToAbsoluteTracking);
	if (!poses[0].bPoseIsValid || !hmd.Finite())
	{
		havePrev = false;
		std::fill(candidateHavePrev.begin(), candidateHavePrev.end(), false);
		return { Event::HeadsetPaused };
	}
	if (!started)
	{
		candidates = ManualCandidates(devices, poses);
		if (candidates.empty())
			return { Event::Waiting };
		for (uint32_t id : candidates)
			for (const auto &device : devices)
				if (device.id == id)
					serials.push_back(device.serial);
		started = true;
		if (candidates.size() == 1)
			return { Event::Selected, candidates.front(), serials.front() };
		RestartEvidence();
		return { Event::Collecting };
	}
	std::vector<Eigen::Matrix3d> rotations(candidates.size() + 1, Eigen::Matrix3d::Identity());
	rotations[0] = hmd.rot;
	std::vector<bool> valid(candidates.size(), false);
	for (size_t i = 0; i < candidates.size(); ++i)
	{
		uint32_t id = candidates[i];
		if (id >= vr::k_unMaxTrackedDeviceCount || !poses[id].bPoseIsValid)
			continue;
		acquisition::Pose pose(poses[id].mDeviceToAbsoluteTracking);
		valid[i] = pose.Finite();
		if (valid[i])
			rotations[i + 1] = pose.rot;
	}
	double dt = time - prevTime;
	if (havePrev && dt > 1e-4)
	{
		hmdSpeeds.push_back(AngularSpeedBetween(rotations[0], prevRot[0], dt));
		for (size_t i = 0; i < candidates.size(); ++i)
			candidateSpeeds[i].push_back(valid[i] && candidateHavePrev[i] ? AngularSpeedBetween(rotations[i + 1], prevRot[i + 1], dt) : 0.0);
	}
	prevRot = std::move(rotations);
	candidateHavePrev = std::move(valid);
	prevTime = time;
	havePrev = true;
	if (hmdSpeeds.size() < 40)
		return { Event::Collecting };
	double peak = *std::max_element(hmdSpeeds.begin(), hmdSpeeds.end());
	double best = -2, second = -2;
	size_t bestIndex = 0;
	for (size_t i = 0; i < candidates.size(); ++i)
	{
		double correlation = PearsonCorrelation(hmdSpeeds, candidateSpeeds[i]);
		if (correlation > best)
		{
			second = best;
			best = correlation;
			bestIndex = i;
		}
		else
			second = std::max(second, correlation);
	}
	if (peak >= 0.5 && best >= 0.7 && best - second >= 0.1)
		return { Event::Selected, candidates[bestIndex], serials[bestIndex] };
	RestartEvidence();
	return { Event::Collecting };
}
