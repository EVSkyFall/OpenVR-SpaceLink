// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "AcquisitionMath.h"

namespace acquisition
{

enum class SampleEvent { Skipped, Accepted, TrackerPaused, HeadsetPaused, SegmentRestarted };

class KeyframeStore
{
public:
	bool Add(Sample sample, size_t capacity, bool eitherPose = false);
	void Clear();
	void SetCapacity(size_t capacity, bool eitherPose = false);
	bool ExpireBefore(double time, bool eitherPose = false);
	const std::vector<Sample> &Samples() const { return samples; }
	uint64_t Count() const { return accepted; }

private:
	std::vector<Sample> samples;
	std::vector<double> nearest;
	std::vector<uint64_t> neighbours;
	uint64_t accepted = 0;
	void RefreshNearest(size_t index, bool eitherPose);
	void Remove(size_t index, bool eitherPose);
};

class Sampler
{
public:
	SampleEvent Observe(double time, const Pose &hmd, bool hmdValid, const Pose &tracker, bool trackerValid,
		size_t capacity, bool referenceRotation = true, bool segmentBreaks = true, bool eitherPose = false);
	void Clear();
	bool ExpireBefore(double time, bool eitherPose = false) { return store.ExpireBefore(time, eitherPose); }
	std::vector<Sample> Snapshot() const { return store.Samples(); }
	const KeyframeStore &Store() const { return store; }
	uint64_t Segment() const { return segment; }

private:
	KeyframeStore store;
	Pose previousHmd, previousTracker;
	double previousTime = 0;
	bool haveHmd = false, haveTracker = false, breakPending = false;
	uint64_t segment = 0;
};

}
