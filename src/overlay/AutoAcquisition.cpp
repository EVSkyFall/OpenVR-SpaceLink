// SPDX-License-Identifier: AGPL-3.0-only

#include "AutoAcquisition.h"

#include <algorithm>
#include <map>
#include <set>

namespace acquisition
{

Evaluation Evaluate(const std::vector<Hypothesis> &hypotheses)
{
	Evaluation evaluation;
	for (const auto &hypothesis : hypotheses)
	{
		SyncResult sync = CombineHandPairs(hypothesis.pairs);
		evaluation.handPairs += sync.handPairs;
		if (!sync.held)
			continue;
		for (const auto &candidate : hypothesis.candidates)
		{
			RigidityResult rigidity;
			if (IsHeadTracker(candidate.samples, sync, &rigidity))
			{
				auto passed = candidate.candidate;
				passed.rotation = rigidity.rotation;
				passed.heightCertain = sync.heightCertain;
				evaluation.passing.push_back(std::move(passed));
			}
		}
	}
	return evaluation;
}

std::optional<CandidateEvidence> Confirmation::Update(const Evaluation &evaluation)
{
	if (evaluation.passing.size() != 1 || !evaluation.passing.front().heightCertain)
	{
		Reset();
		return {};
	}
	const auto &candidate = evaluation.passing.front();
	if (previous && previous->serial == candidate.serial && previous->system == candidate.system
		&& candidate.count >= previous->count && candidate.count - previous->count >= 20)
		return candidate;
	if (!previous || previous->serial != candidate.serial || previous->system != candidate.system || candidate.count < previous->count)
		previous = candidate;
	return {};
}

static bool Compatible(vr::ETrackedControllerRole a, vr::ETrackedControllerRole b)
{
	auto known = [](vr::ETrackedControllerRole role) {
		return role == vr::TrackedControllerRole_LeftHand || role == vr::TrackedControllerRole_RightHand;
	};
	return !known(a) || !known(b) || a == b;
}

void AutoAcquisition::Refresh(const std::vector<DeviceSnapshot> &devices)
{
	std::string hmdSystem;
	for (const auto &device : devices)
		if (device.id == vr::k_unTrackedDeviceIndex_Hmd)
			hmdSystem = device.trackingSystem;
	std::set<std::string> systems;
	for (const auto &device : devices)
		if (device.deviceClass == vr::TrackedDeviceClass_GenericTracker && device.trackingSystem != hmdSystem)
			systems.insert(device.trackingSystem);
	std::vector<PairStore> nextPairs;
	std::vector<CandidateStore> nextCandidates;
	for (const auto &system : systems)
	{
		for (const auto &l : devices)
		{
			if (l.trackingSystem != system)
				continue;
			if (l.deviceClass == vr::TrackedDeviceClass_GenericTracker)
			{
				auto found = std::find_if(candidates.begin(), candidates.end(), [&](const CandidateStore &store) {
					return store.candidate.id == l.id && store.candidate.serial == l.serial && store.candidate.system == system;
				});
				if (found != candidates.end())
					nextCandidates.push_back(std::move(*found));
				else
					nextCandidates.push_back({ { l.id, l.serial, system }, {} });
			}
			if (l.deviceClass != vr::TrackedDeviceClass_Controller)
				continue;
			for (const auto &s : devices)
			{
				bool hand = s.deviceClass == vr::TrackedDeviceClass_Controller
					|| s.role == vr::TrackedControllerRole_LeftHand || s.role == vr::TrackedControllerRole_RightHand;
				if (!hand || s.trackingSystem == system || s.deviceClass == vr::TrackedDeviceClass_HMD
					|| s.deviceClass == vr::TrackedDeviceClass_TrackingReference || !Compatible(s.role, l.role))
					continue;
				auto found = std::find_if(pairs.begin(), pairs.end(), [&](const PairStore &store) {
					return store.system == system && store.hand == s.id && store.controller == l.id
						&& store.handSerial == s.serial && store.controllerSerial == l.serial;
				});
				if (found != pairs.end())
					nextPairs.push_back(std::move(*found));
				else
					nextPairs.push_back({ system, s.serial, l.serial, s.id, l.id, {} });
			}
		}
	}
	pairs = std::move(nextPairs);
	candidates = std::move(nextCandidates);
}

bool AutoAcquisition::ObserveConfigured(double time, const vr::TrackedDevicePose_t *observed, const ObservationSpace &space)
{
	if (!space.HasConfiguration())
		return false;
	auto poses = space.RawPoses(observed);
	Observe(time, poses.data());
	return true;
}

void AutoAcquisition::Observe(double time, const vr::TrackedDevicePose_t *poses)
{
	for (auto &pair : pairs)
		pair.sampler.ExpireBefore(time - 180.0);
	for (auto &candidate : candidates)
		candidate.sampler.ExpireBefore(time - 180.0, true);
	const auto &hmd = poses[vr::k_unTrackedDeviceIndex_Hmd];
	Pose head(hmd.mDeviceToAbsoluteTracking);
	bool headValid = hmd.bPoseIsValid && head.Finite();
	if (!headValid && !headsetLost)
	{
		++epoch;
		confirmation.Reset();
	}
	if (headValid && headsetLost)
	{
		for (auto &pair : pairs)
			pair.sampler.Clear();
		for (auto &candidate : candidates)
			candidate.sampler.Clear();
		++epoch;
		confirmation.Reset();
	}
	headsetLost = !headValid;
	for (auto &pair : pairs)
	{
		const auto &s = poses[pair.hand];
		const auto &l = poses[pair.controller];
		pair.sampler.Observe(time, Pose(s.mDeviceToAbsoluteTracking), s.bPoseIsValid,
			Pose(l.mDeviceToAbsoluteTracking), l.bPoseIsValid, 400, false, false);
	}
	for (auto &candidate : candidates)
	{
		const auto &tracker = poses[candidate.candidate.id];
		candidate.sampler.Observe(time, head, headValid, Pose(tracker.mDeviceToAbsoluteTracking),
			tracker.bPoseIsValid, 400, true, false, true);
	}
}

std::vector<Hypothesis> AutoAcquisition::Snapshot() const
{
	std::map<std::string, Hypothesis> grouped;
	for (const auto &pair : pairs)
	{
		auto &hypothesis = grouped[pair.system];
		hypothesis.system = pair.system;
		hypothesis.pairs.push_back({ pair.hand, pair.controller, pair.sampler.Store().Samples() });
	}
	for (const auto &store : candidates)
	{
		auto &hypothesis = grouped[store.candidate.system];
		hypothesis.system = store.candidate.system;
		auto candidate = store.candidate;
		candidate.count = store.sampler.Store().Count();
		hypothesis.candidates.push_back({ candidate, store.sampler.Store().Samples() });
	}
	std::vector<Hypothesis> result;
	for (auto &entry : grouped)
		result.push_back(std::move(entry.second));
	return result;
}

void AutoAcquisition::Clear()
{
	++epoch;
	headsetLost = false;
	pairs.clear();
	candidates.clear();
	confirmation.Reset();
}

}
