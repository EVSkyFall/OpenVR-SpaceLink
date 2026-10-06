// SPDX-License-Identifier: AGPL-3.0-only

#include "SamplerPolicy.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

namespace acquisition
{

static double PoseDistance(const Pose &a, const Pose &b)
{
	return std::max(RotationAngle(a.rot * b.rot.transpose()) / (3 * Degrees), (a.trans - b.trans).norm() / 0.01);
}

static double Distance(const Sample &a, const Sample &b, bool eitherPose)
{
	double distance = PoseDistance(a.target, b.target);
	return eitherPose ? std::max(distance, PoseDistance(a.ref, b.ref)) : distance;
}

bool KeyframeStore::Add(Sample sample, size_t capacity, bool eitherPose)
{
	SetCapacity(capacity, eitherPose);
	if (!sample.ref.Finite() || !sample.target.Finite())
		return false;
	std::vector<double> distances;
	double closest = INFINITY;
	uint64_t neighbour = 0;
	for (const auto &stored : samples)
	{
		double distance = Distance(sample, stored, eitherPose);
		if (distance < 1.0)
			return false;
		distances.push_back(distance);
		if (distance < closest)
		{
			closest = distance;
			neighbour = stored.sequence;
		}
	}
	sample.sequence = accepted + 1;
	for (size_t i = 0; i < samples.size(); ++i)
		if (distances[i] < nearest[i])
		{
			nearest[i] = distances[i];
			neighbours[i] = sample.sequence;
		}
	samples.push_back(sample);
	nearest.push_back(closest);
	neighbours.push_back(neighbour);
	uint64_t removed = 0;
	if (samples.size() > capacity)
	{
		size_t redundant = static_cast<size_t>(std::min_element(nearest.begin(), nearest.end()) - nearest.begin());
		removed = samples[redundant].sequence;
		Remove(redundant, eitherPose);
	}
	if (removed == sample.sequence)
		return false;
	++accepted;
	return true;
}

void KeyframeStore::RefreshNearest(size_t index, bool eitherPose)
{
	nearest[index] = INFINITY;
	neighbours[index] = 0;
	for (size_t j = 0; j < samples.size(); ++j)
		if (index != j)
		{
			double distance = Distance(samples[index], samples[j], eitherPose);
			if (distance < nearest[index])
			{
				nearest[index] = distance;
				neighbours[index] = samples[j].sequence;
			}
		}
}

void KeyframeStore::Remove(size_t index, bool eitherPose)
{
	uint64_t removed = samples[index].sequence;
	samples.erase(samples.begin() + index);
	nearest.erase(nearest.begin() + index);
	neighbours.erase(neighbours.begin() + index);
	for (size_t i = 0; i < samples.size(); ++i)
		if (neighbours[i] == removed)
			RefreshNearest(i, eitherPose);
}

void KeyframeStore::SetCapacity(size_t capacity, bool eitherPose)
{
	while (samples.size() > capacity)
		Remove(static_cast<size_t>(std::min_element(nearest.begin(), nearest.end()) - nearest.begin()), eitherPose);
}

bool KeyframeStore::ExpireBefore(double time, bool eitherPose)
{
	std::unordered_set<uint64_t> removed;
	size_t kept = 0;
	for (size_t i = 0; i < samples.size(); ++i)
	{
		if (samples[i].time < time)
			removed.insert(samples[i].sequence);
		else
		{
			samples[kept] = samples[i];
			nearest[kept] = nearest[i];
			neighbours[kept++] = neighbours[i];
		}
	}
	samples.resize(kept);
	nearest.resize(kept);
	neighbours.resize(kept);
	for (size_t i = 0; i < kept; ++i)
		if (removed.count(neighbours[i]))
			RefreshNearest(i, eitherPose);
	return !removed.empty();
}

void KeyframeStore::Clear()
{
	samples.clear();
	nearest.clear();
	neighbours.clear();
	accepted = 0;
}

SampleEvent Sampler::Observe(double time, const Pose &hmd, bool hmdValid, const Pose &tracker, bool trackerValid,
	size_t capacity, bool referenceRotation, bool segmentBreaks, bool eitherPose)
{
	store.SetCapacity(capacity, eitherPose);
	hmdValid = hmdValid && hmd.Finite();
	trackerValid = trackerValid && tracker.Finite();
	double dt = time - previousTime;
	bool headPrevious = haveHmd, trackerPrevious = haveTracker;
	Pose oldHmd = previousHmd, oldTracker = previousTracker;
	haveHmd = hmdValid;
	haveTracker = trackerValid;
	previousTime = time;
	previousHmd = hmd;
	previousTracker = tracker;
	if (!hmdValid)
	{
		breakPending = segmentBreaks;
		return SampleEvent::HeadsetPaused;
	}
	if (!trackerValid)
		return SampleEvent::TrackerPaused;
	bool consecutive = headPrevious && trackerPrevious && dt > 0 && dt <= 0.15;
	bool jump = segmentBreaks && consecutive
		&& ((hmd.trans - oldHmd.trans).norm() > 0.10 || RotationAngle(hmd.rot * oldHmd.rot.transpose()) > 15 * Degrees)
		&& (tracker.trans - oldTracker.trans).norm() < 0.02 && RotationAngle(tracker.rot * oldTracker.rot.transpose()) < 3 * Degrees;
	if (breakPending || jump)
	{
		store.Clear();
		++segment;
		breakPending = false;
		return SampleEvent::SegmentRestarted;
	}
	if (!consecutive)
		return SampleEvent::Skipped;
	if (dt > 0)
	{
		if (headPrevious && ((hmd.trans - oldHmd.trans).norm() / dt > 0.4
			|| (referenceRotation && RotationAngle(hmd.rot * oldHmd.rot.transpose()) / dt > 1.2)))
			return SampleEvent::Skipped;
		if (trackerPrevious && ((tracker.trans - oldTracker.trans).norm() / dt > 0.4
			|| RotationAngle(tracker.rot * oldTracker.rot.transpose()) / dt > 1.2))
			return SampleEvent::Skipped;
	}
	Sample sample{ hmd, tracker };
	sample.time = time;
	return store.Add(sample, capacity, eitherPose) ? SampleEvent::Accepted : SampleEvent::Skipped;
}

void Sampler::Clear()
{
	store.Clear();
	haveHmd = haveTracker = breakPending = false;
	++segment;
}

}
