// SPDX-License-Identifier: AGPL-3.0-only

#include "AcquisitionMath.h"
#include "SamplerPolicy.h"
#include "AutoAcquisition.h"
#include "CalibrationMath.h"
#include "AttemptLifecycle.h"
#include "DesiredState.h"
#include "ProfileCodec.h"
#include "ManualDetection.h"
#include "SpaceMemory.h"
#include "OverlayLog.h"
#include <fstream>
#include <limits>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <stdexcept>
#include <string>
#include <functional>
#include <picojson.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

using namespace acquisition;

static int CurrentScenario = 0;

static void Check(bool condition, const char *message)
{
	if (!condition)
		throw std::runtime_error("Scenario" + std::to_string(CurrentScenario) + " FAIL: " + message);
}

static Eigen::Matrix3d Rotation(double x, double y, double z = 0)
{
	return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

static const Eigen::Matrix3d SyncRotation = Rotation(0, 60 * Degrees);
static const Eigen::Vector3d SyncTranslation(1.2, 0.3, -0.7);

static Pose Controller(double time, int hand, bool yawOnly = false)
{
	Pose pose;
	pose.trans = Eigen::Vector3d(0.45 * std::sin(time * 0.61) + 0.15 * hand,
		1.1 + 0.16 * std::sin(time * 0.37), 0.4 * std::cos(time * 0.49));
	pose.rot = Rotation(yawOnly ? 0 : 0.65 * std::sin(time * 0.43),
		0.8 * std::sin(time * 0.71), yawOnly ? 0 : 0.55 * std::cos(time * 0.53));
	return pose;
}

static HandPair MakePair(int hand, size_t count = 240, bool outliers = false, bool yawOnly = false, bool noise = true, double outlierSize = 0.2)
{
	HandPair pair;
	pair.hand = static_cast<uint32_t>(1 + hand);
	pair.controller = static_cast<uint32_t>(3 + hand);
	std::mt19937 random(41 + hand);
	std::normal_distribution<double> normal;
	Eigen::Vector3d offset(0.03 + hand * 0.04, 0.04 - hand * 0.07, -0.06 + hand * 0.03);
	for (size_t i = 0; i < count; ++i)
	{
		Sample sample;
		sample.target = Controller(i * 0.15, hand, yawOnly);
		sample.ref.trans = SyncRotation * (sample.target.trans + sample.target.rot * offset) + SyncTranslation;
		// No orientation from the S hand participates in the fit.
		sample.ref.rot = Rotation(normal(random), normal(random), normal(random));
		if (noise)
			for (int axis = 0; axis < 3; ++axis)
			{
				sample.ref.trans(axis) += 0.01 * normal(random);
				sample.target.trans(axis) += 0.002 * normal(random);
			}
		if (outliers && (i % 100 == 0 || (i % 10 == 2 && i % 100 != 92)))
			sample.ref.trans += outlierSize * Eigen::Vector3d(0.16, -0.10, 0.065).normalized();
		sample.sequence = i + 1;
		pair.samples.push_back(sample);
	}
	return pair;
}

static void CheckSync(const SyncResult &fit, bool height = true)
{
	Check(fit.held, "hand pair must be held");
	Check(RotationAngle(fit.rotation * SyncRotation.transpose()) < 2 * Degrees, "yaw error exceeds 2 degrees");
	Eigen::Vector3d error = fit.translation - SyncTranslation;
	Check((height ? error.norm() : std::hypot(error.x(), error.z())) < 0.03, "translation error exceeds 3 cm");
}

static void Scenario1()
{
	auto first = MakePair(0), second = MakePair(1);
	auto fit = CombineHandPairs({ first, second });
	CheckSync(fit);
	Check(fit.heightCertain && fit.handPairs == 2, "both distinct offsets must participate in joint fit");
	for (auto &sample : first.samples)
		sample.ref.rot = Eigen::Matrix3d::Identity();
	CheckSync(CombineHandPairs({ first, second }));
	second.hand = first.hand;
	Check(CombineHandPairs({ first, second }).handPairs == 1, "one hand cannot appear in two pairs");
	auto projection = MakePair(0, 300, false, false, false);
	Eigen::Vector3d meanL = Eigen::Vector3d::Zero(), meanS = meanL;
	for (auto &sample : projection.samples)
	{
		sample.target.trans.x() *= 0.5;
		sample.target.trans.z() *= 0.5;
		sample.ref.trans = SyncRotation * (sample.target.trans + sample.target.rot * Eigen::Vector3d(0.6, 0.1, -0.4)) + SyncTranslation;
		meanL += sample.target.trans;
		meanS += sample.ref.trans;
	}
	meanL /= static_cast<double>(projection.samples.size());
	meanS /= static_cast<double>(projection.samples.size());
	double dot = 0, cross = 0;
	for (const auto &sample : projection.samples)
	{
		Eigen::Vector3d l = sample.target.trans - meanL, h = sample.ref.trans - meanS;
		dot += l.x() * h.x() + l.z() * h.z();
		cross += l.z() * h.x() - l.x() * h.z();
	}
	Check(std::abs(std::atan2(cross, dot) - 60 * Degrees) > 10 * Degrees, "projection fixture must start with a clearly wrong position-only yaw");
	CheckSync(FitHandPairs({ projection }));
}

static void Scenario2()
{
	for (double magnitude : { 0.2, 0.4 })
	{
		auto fit = CombineHandPairs({ MakePair(0, 400, true, false, true, magnitude), MakePair(1, 400, true, false, true, magnitude) });
		CheckSync(fit);
		Check((fit.translation - SyncTranslation).norm() < 0.01, "Huber fit must keep translation within 1 cm despite training and holdout outliers");
	}
}

static void Scenario3()
{
	auto pair = MakePair(0, 160, false, false, false);
	Pose fixed = pair.samples.front().target;
	for (auto &sample : pair.samples)
		sample.target = fixed;
	Check(!FitHandPairs({ pair }).held, "static controller must not be held by a moving hand");
	// Even a tiny moving hand that fits the residual bound lacks controller position spread.
	for (size_t i = 0; i < pair.samples.size(); ++i)
		pair.samples[i].ref.trans = fixed.trans + Eigen::Vector3d(0.005 * std::sin(i * 0.1), 0, 0);
	Check(!FitHandPairs({ pair }).held, "static controller must fail the horizontal spread test");
}

static void Scenario4()
{
	auto fit = FitHandPairs({ MakePair(0, 300, false, true) });
	CheckSync(fit, false);
	Check(!fit.heightCertain, "yaw-only rotations must mark height uncertain");
	auto tilted = MakePair(0, 300, false, false, false);
	Eigen::Vector3d axis = Eigen::Vector3d(1, 1, 0).normalized();
	for (size_t i = 0; i < tilted.samples.size(); ++i)
	{
		auto &sample = tilted.samples[i];
		sample.target.rot = Eigen::AngleAxisd(0.6 * std::sin(i * 0.15), axis).toRotationMatrix();
		sample.ref.trans = SyncRotation * (sample.target.trans + sample.target.rot * Eigen::Vector3d(0.03, 0.04, -0.06)) + SyncTranslation;
	}
	auto tiltedFit = FitHandPairs({ tilted });
	Check(tiltedFit.held && !tiltedFit.heightCertain, "tilted single-axis motion cannot separate height from the hand offset");
	auto constant = MakePair(0, 240, false, false, false);
	for (auto &sample : constant.samples)
	{
		sample.target.rot = Rotation(0.1, 0.3);
		sample.ref.trans = SyncRotation * (sample.target.trans + sample.target.rot * Eigen::Vector3d(0.03, 0.04, -0.06)) + SyncTranslation;
	}
	auto constantFit = FitHandPairs({ constant });
	Check(constantFit.held && !constantFit.heightCertain && (constantFit.translation - SyncTranslation).norm() < 0.15,
		"constant controller orientation must leave translation within the physical hand offset");
	auto linear = constant;
	for (size_t i = 0; i < linear.samples.size(); ++i)
	{
		auto &sample = linear.samples[i];
		sample.target.trans = Eigen::Vector3d(i * 1e-9, 0, 0);
		sample.ref.trans = SyncRotation * sample.target.trans + SyncTranslation;
	}
	auto weak = FitHandPairs({ linear });
	Check(weak.rotation.allFinite() && weak.translation.allFinite() && !weak.held, "unobservable near-linear motion must remain finite and not held");
	Check(DampedYawStep(1e100, 1e-30) == 0.2 && DampedYawStep(-1e100, 1e-30) == -0.2, "yaw step must clamp huge positive and negative ratios");
	Check(std::isfinite(DampedYawStep(0, 0)) && DampedYawStep(0, 0) == 0,
		"yaw damping must keep a zero-information step finite");
	Check(std::abs(DampedYawStep(1e-20, 1e-30)) < 1e-6, "tiny-information yaw step must retain damping below the clamp");
}

static std::vector<Sample> HeadSamples(double distance, bool chest = false, bool neckMotion = true, size_t count = 180)
{
	std::vector<Sample> samples;
	Eigen::Matrix3d mounting = Rotation(0.12, -0.24, 0.15);
	for (size_t i = 0; i < count; ++i)
	{
		double time = i * 0.15;
		Eigen::Matrix3d body = Rotation(0.23 * std::sin(time * 0.19), 0.55 * std::sin(time * 0.27), 0.17 * std::cos(time * 0.31));
		Eigen::Matrix3d neck = neckMotion ? Rotation(0.45 * std::sin(time * 0.71), 0.50 * std::sin(time * 0.89)) : Eigen::Matrix3d::Identity();
		Sample sample;
		sample.ref.rot = body * neck;
		sample.ref.trans = Eigen::Vector3d(1.0 + 0.2 * std::sin(time * 0.23), 1.65, -0.2 + 0.2 * std::cos(time * 0.41));
		Eigen::Matrix3d candidateRotation = chest ? body : sample.ref.rot;
		sample.target.rot = SyncRotation.transpose() * candidateRotation * mounting;
		Eigen::Vector3d offset = chest ? Eigen::Vector3d(0, -distance, 0) : Eigen::Vector3d(0, distance, 0);
		sample.target.trans = SyncRotation.transpose() * (sample.ref.trans + candidateRotation * offset - SyncTranslation);
		sample.sequence = i + 1;
		samples.push_back(sample);
	}
	return samples;
}

static SyncResult KnownSync()
{
	SyncResult result;
	result.held = result.heightCertain = true;
	result.rotation = SyncRotation;
	result.translation = SyncTranslation;
	return result;
}

static Evaluation Detect(const std::vector<std::vector<Sample>> &candidates)
{
	Evaluation evaluation;
	for (size_t i = 0; i < candidates.size(); ++i)
	{
		RigidityResult rigidity;
		if (IsHeadTracker(candidates[i], KnownSync(), &rigidity))
			evaluation.passing.push_back({ static_cast<uint32_t>(5 + i), "tracker" + std::to_string(i), "system-Y", candidates[i].size(), rigidity.rotation, true });
	}
	return evaluation;
}

static void Scenario5()
{
	auto evaluation = Detect({ HeadSamples(0.08), HeadSamples(0.27, true) });
	Check(FitRigidity(HeadSamples(0.27, true)).error.median > 3 * Degrees, "chest neck motion must fail rotational rigidity");
	Check(evaluation.passing.size() == 1 && evaluation.passing.front().id == 5, "head must be chosen and moving-neck chest rejected");
	Confirmation confirmation;
	Check(!confirmation.Update(evaluation), "first pass must not confirm");
	for (int i = 0; i < 8; ++i)
		Check(!confirmation.Update(evaluation), "repeated buffer evaluation is not grown evidence");
	evaluation.passing.front().count += 4;
	Check(!confirmation.Update(evaluation), "4 new keyframes are insufficient");
	++evaluation.passing.front().count;
	Check(confirmation.Update(evaluation).has_value(), "5 new keyframes must confirm");
	auto ambiguous = Detect({ HeadSamples(0.08, false, false), HeadSamples(0.12, false, false) });
	Confirmation fresh;
	auto single = evaluation;
	Check(!fresh.Update(single), "fresh uniqueness check must start unconfirmed");
	ambiguous.passing[0].count = single.passing.front().count + 20;
	ambiguous.passing[1].count = single.passing.front().count + 20;
	Check(ambiguous.passing.size() == 2 && !fresh.Update(ambiguous), "two passing candidates must never confirm");
	single.passing.front().count += 20;
	Check(!fresh.Update(single), "ambiguity must reset earlier single-candidate evidence");
	Confirmation uncertain;
	single.passing.front().heightCertain = false;
	Check(!uncertain.Update(single), "uncertain height still needs a second pass");
	single.passing.front().count += 20;
	Check(uncertain.Update(single).has_value(), "grown evidence must confirm without height certainty");
	Check(!IsHeadTracker(HeadSamples(0.08), [&] { auto s = KnownSync(); s.rotation = Rotation(0, 80 * Degrees); return s; }()), "sync rotation disagreement must reject");
}

static void Scenario6()
{
	Check(Detect({ HeadSamples(0.28, true) }).passing.empty(), "only a chest tracker must yield no candidate");
	auto rigidChest = HeadSamples(0.27, true, false);
	Check(FitRigidity(rigidChest).error.median < Degrees, "fixed-neck chest fixture must be rotationally rigid");
	Check(!IsHeadTracker(rigidChest, KnownSync()), "rigid chest below the HMD must fail the vertical prior");
	for (const Eigen::Vector3d &offset : { Eigen::Vector3d(0, 0.08, 0), Eigen::Vector3d(0, 0, 0.1), Eigen::Vector3d(0, -0.05, -0.08) })
	{
		auto head = HeadSamples(0.08, false, false);
		for (auto &sample : head)
			sample.target.trans = SyncRotation.transpose() * (sample.ref.trans + sample.ref.rot * offset - SyncTranslation);
		Check(IsHeadTracker(head, KnownSync()), "top, front and back-strap head trackers must pass");
	}
}

static void Scenario7()
{
	auto evaluation = Detect({ HeadSamples(0.15) });
	Confirmation confirmation;
	Check(evaluation.passing.size() == 1, "rigid headphone tracker at 15 cm must pass");
	Check(!confirmation.Update(evaluation), "headphone candidate starts with unconfirmed evidence");
	evaluation.passing.front().count += 20;
	Check(confirmation.Update(evaluation).has_value(), "headphone tracker must be chosen after grown evidence");
	Check(!IsHeadTracker(HeadSamples(0.45), KnownSync()), "distant rigid device must fail proximity");
	auto scattered = HeadSamples(0.08);
	for (size_t i = 0; i < scattered.size(); i += 4)
		scattered[i].target.trans += SyncRotation.transpose() * scattered[i].ref.rot * Eigen::Vector3d(0.16, 0, 0);
	Check(!IsHeadTracker(scattered, KnownSync()), "p90 offset spread must reject a positionally loose tracker");
	auto pitching = HeadSamples(0.08);
	for (size_t i = 0; i < pitching.size(); ++i)
	{
		auto &sample = pitching[i];
		sample.ref.rot = Rotation(70 * Degrees + 0.55 * std::sin(i * 0.1), 0.4 * std::sin(i * 0.17));
		sample.target.rot = SyncRotation.transpose() * sample.ref.rot;
		sample.target.trans = SyncRotation.transpose() * (sample.ref.trans + sample.ref.rot * Eigen::Vector3d(0.2, 0.08, 0) - SyncTranslation);
	}
	auto uncertainSync = FitHandPairs({ MakePair(0, 300, false, true, false) });
	Check(uncertainSync.held && !uncertainSync.heightCertain, "height-uncertain fixture must come from yaw-only hand motion");
	uncertainSync.translation.y() = SyncTranslation.y() + 0.03;
	Check(IsHeadTracker(pitching, uncertainSync), "the prior-bounded vertical estimate must be used as is while pitching");
	for (size_t i = 0; i < pitching.size(); i += 4)
		pitching[i].target.trans += SyncRotation.transpose() * pitching[i].ref.rot * Eigen::Vector3d(0.17, 0, 0);
	Check(!IsHeadTracker(pitching, uncertainSync), "uncertain-height spread must still reject lateral looseness");
	uncertainSync.translation.y() += 0.6;
	Check(!IsHeadTracker(HeadSamples(0.08), uncertainSync), "uncertain height must still enforce full 3D proximity");
}

static Pose LatencyController(double time)
{
	Pose pose = Controller(time * 0.38, 0);
	// Alternate genuinely fast motion with slow, informative motion.
	double burst = std::sin(time * 0.27);
	if (burst > 0)
	{
		pose.trans.x() += 0.12 * burst * std::sin(time * 7);
		pose.rot = pose.rot * Rotation(0.25 * burst * std::sin(time * 6), 0);
	}
	return pose;
}

static void Scenario8()
{
	Sampler sampler;
	HandPair pair;
	pair.hand = 1; pair.controller = 3;
	size_t skipped = 0;
	Eigen::Vector3d offset(0.03, 0.04, -0.06);
	for (int i = 0; i < 3600; ++i)
	{
		double time = i * 0.05;
		Pose target = LatencyController(time), delayed = LatencyController(time - 0.08), reference;
		reference.trans = SyncRotation * (delayed.trans + delayed.rot * offset) + SyncTranslation;
		auto event = sampler.Observe(time, reference, true, target, true, 400, false, false);
		if (event == SampleEvent::Skipped)
			++skipped;
	}
	pair.samples = sampler.Store().Samples();
	Check(pair.samples.size() >= 40 && skipped > 100, "latency stream must retain slow informative frames");
	CheckSync(FitHandPairs({ pair }));
	Sampler fast;
	Pose reference, target;
	fast.Observe(0, reference, true, target, true, 400);
	target.trans.x() = 0.04; reference.trans.x() = 0.04;
	Check(fast.Observe(0.05, reference, true, target, true, 400) == SampleEvent::Skipped, "finite differences must reject fast frames even without driver velocity");
}

static void Scenario9()
{
	std::vector<Eigen::Matrix3d> twoAxes, yawOnly;
	for (int i = -20; i <= 20; ++i)
	{
		twoAxes.push_back(Rotation(i * Degrees, 0));
		twoAxes.push_back(Rotation(0, i * Degrees));
		yawOnly.push_back(Rotation(0, 3 * i * Degrees));
	}
	Check(RotationInformation(twoAxes) >= 0.01, "plus/minus 20 degree X/Y motion must pass");
	Check(RotationInformation(yawOnly) < 0.01, "single-axis plus/minus 60 degree yaw must fail");
}

static void Scenario10()
{
	Sampler sampler;
	Pose hmd, tracker;
	for (int i = 0; i < 100; ++i)
		sampler.Observe(i * 0.05, hmd, true, tracker, true, 400);
	Check(sampler.Store().Samples().size() == 1, "stationary stream must add only its first keyframe");
	Check(sampler.Observe(5, hmd, true, tracker, false, 400) == SampleEvent::TrackerPaused, "invalid tracker must pause");
	Check(sampler.Store().Samples().size() == 1, "tracker pause must preserve samples");
	Check(sampler.Observe(5.05, hmd, false, tracker, true, 400) == SampleEvent::HeadsetPaused, "invalid HMD must pause");
	Check(sampler.Observe(5.1, hmd, true, tracker, true, 400) == SampleEvent::SegmentRestarted, "HMD recovery must start a new segment");
	sampler.Observe(5.15, hmd, true, tracker, true, 400);
	auto segment = sampler.Segment();
	hmd.trans.x() += 0.2;
	Check(sampler.Observe(5.2, hmd, true, tracker, true, 400) == SampleEvent::SegmentRestarted, "unmatched HMD jump must restart samples");
	Check(sampler.Segment() > segment && sampler.Store().Samples().empty(), "jump must clear the previous segment");
	Sampler stored;
	auto observations = HeadSamples(0.08, false, true, 240);
	for (size_t i = 0; i < observations.size(); ++i)
	{
		auto sample = observations[i];
		sample.ref.trans *= 1.035;
		stored.Observe(i * 0.1, sample.ref, true, sample.target, true, 200);
	}
	auto original = stored.Snapshot();
	Check(original.size() >= 40, "worker-input fixture must collect real sampler keyframes");
	auto first = SolveCalibration(stored.Snapshot(), 1.0034);
	auto second = SolveCalibration(stored.Snapshot(), 1.0034);
	Check(std::abs(first.hmdScale - 1.035) < 0.005 && std::abs(second.hmdScale - 1.035) < 0.005, "worker snapshots must retain non-unit raw headset scale on repeated solves");
	Check(first.holdout.rms < 0.001 && second.holdout.rms < 0.001, "repeated worker-input solves must retain calibration accuracy");
	Check(stored.Store().Samples().size() == original.size(), "preparing worker inputs must preserve the sampler store");
	for (size_t i = 0; i < original.size(); ++i)
		Check(stored.Store().Samples()[i].ref.trans == original[i].ref.trans && stored.Store().Samples()[i].target.trans == original[i].target.trans,
			"sampler store changed while preparing repeated worker inputs");
	Sampler gaps;
	Pose a, b;
	Check(gaps.Observe(0, a, true, b, true, 100) == SampleEvent::Skipped, "first observation must only establish speed baselines");
	Check(gaps.Observe(0.15, a, true, b, true, 100) == SampleEvent::Accepted, "dt of exactly 0.15 seconds must be allowed");
	a.trans.x() = 0.2;
	auto oldSegment = gaps.Segment();
	Check(gaps.Observe(0.31, a, true, b, true, 100) == SampleEvent::Skipped && gaps.Segment() == oldSegment,
		"a jump across a gap must only refresh previous poses");
	gaps.Observe(0.36, a, true, b, false, 100);
	b.trans.x() = 0.2;
	Check(gaps.Observe(0.41, a, true, b, true, 100) == SampleEvent::Skipped, "first re-acquired pose must not bypass the speed gate");
	Check(gaps.Observe(0.46, a, true, b, true, 100) == SampleEvent::Accepted, "the next consecutive valid tick must resume sampling");
	Pose bad = b;
	bad.trans.x() = std::numeric_limits<double>::quiet_NaN();
	Check(gaps.Observe(0.51, a, true, bad, true, 100) == SampleEvent::TrackerPaused, "non-finite tracker translation must be invalid");
	bad = a;
	bad.rot(1, 2) = std::numeric_limits<double>::infinity();
	Check(gaps.Observe(0.56, bad, true, b, true, 100) == SampleEvent::HeadsetPaused, "non-finite HMD rotation must be invalid");
	for (const auto &sample : gaps.Store().Samples())
		Check(sample.ref.Finite() && sample.target.Finite(), "non-finite poses must never enter the store");
	Sample still, referenceMoved;
	referenceMoved.ref.trans.x() = 0.02;
	KeyframeStore targetNovelty, eitherNovelty;
	targetNovelty.Add(still, 100);
	eitherNovelty.Add(still, 100, true);
	Check(!targetNovelty.Add(referenceMoved, 100) && eitherNovelty.Add(referenceMoved, 100, true), "eitherPose novelty must retain reference-only movement");
	KeyframeStore reservoir;
	for (int i = 0; i < 40; ++i)
	{
		Sample sample;
		sample.target.trans.x() = i * 0.011;
		if (i == 0)
			sample.target.trans.y() = 2;
		reservoir.Add(sample, 12);
	}
	Check(reservoir.Samples().size() == 12, "reservoir must remain bounded");
	Check(std::any_of(reservoir.Samples().begin(), reservoir.Samples().end(), [](const Sample &s) { return s.target.trans.y() == 2; }), "redundancy eviction must preserve informative oldest keyframe");
	reservoir.SetCapacity(5);
	Check(reservoir.Samples().size() == 5, "a smaller calibration capacity must take effect without a new accepted frame");
	CalibrationSolution quality;
	quality.holdout = { 0, 0.15, 0.10 };
	quality.rotationError.median = 3 * Degrees;
	Check(AutomaticCalibrationSucceeded(quality), "automatic success thresholds are inclusive");
	quality.holdout.rms = std::nextafter(0.10, 1.0);
	Check(!AutomaticCalibrationSucceeded(quality), "automatic RMS above 10 cm must fail");
	quality.holdout.rms = 0.10;
	quality.holdout.p90 = std::nextafter(0.15, 1.0);
	Check(!AutomaticCalibrationSucceeded(quality), "automatic p90 above 15 cm must fail");
	quality.holdout.p90 = 0.15;
	quality.rotationError.median = std::nextafter(3 * Degrees, 1.0);
	Check(!AutomaticCalibrationSucceeded(quality), "automatic rotation median above 3 degrees must fail");
	Check(!AutomaticCalibrationAbandoned(19, 9 * Degrees) && !AutomaticCalibrationAbandoned(20, 8 * Degrees)
		&& AutomaticCalibrationAbandoned(20, std::nextafter(8 * Degrees, 1.0)), "automatic abandon requires 20 newer frames and strictly more than 8 degrees");
}

static CalibrationContext Profile()
{
	CalibrationContext profile;
	profile.validProfile = profile.validRelativeOffset = true;
	profile.trackerSerial = "committed-A";
	profile.targetTrackingSystem = "system-Y";
	profile.hmdSerial = "hmd-A";
	profile.calibratedRotation = Eigen::Vector3d(1, 60, 2);
	profile.calibratedTranslation = Eigen::Vector3d(120, 30, -70);
	profile.relativeTranslation = { 0.01, -0.08, 0.02 };
	return profile;
}

static std::vector<DeviceSnapshot> Devices()
{
	return {
		{ 0, vr::TrackedDeviceClass_HMD, vr::TrackedControllerRole_Invalid, "hmd-A", "system-S", true, true },
		{ 1, vr::TrackedDeviceClass_Controller, vr::TrackedControllerRole_LeftHand, "hand-L", "system-S", true, true },
		{ 2, vr::TrackedDeviceClass_Controller, vr::TrackedControllerRole_RightHand, "hand-R", "system-S", true, true },
		{ 3, vr::TrackedDeviceClass_Controller, vr::TrackedControllerRole_LeftHand, "controller-L", "system-Y", true, true },
		{ 4, vr::TrackedDeviceClass_Controller, vr::TrackedControllerRole_RightHand, "controller-R", "system-Y", true, true },
		{ 5, vr::TrackedDeviceClass_GenericTracker, vr::TrackedControllerRole_Invalid, "committed-A", "system-Y", false, false },
		{ 6, vr::TrackedDeviceClass_GenericTracker, vr::TrackedControllerRole_Invalid, "chest", "system-Y", true, true },
		{ 7, vr::TrackedDeviceClass_TrackingReference, vr::TrackedControllerRole_Invalid, "reference", "system-Y", true, true }
	};
}

static vr::TrackedDevicePose_t TrackedPose(const Pose &pose);

static void Scenario11()
{
	auto devices = Devices();
	auto profile = Profile();
	auto commands = DesiredState(devices, profile, false);
	int transforms[vr::k_unMaxTrackedDeviceCount]{};
	int syncs[vr::k_unMaxTrackedDeviceCount]{};
	for (const auto &command : commands)
	{
		if (command.type == protocol::RequestSetDeviceTransform)
		{
			const auto &t = command.setDeviceTransform;
			Check(t.openVRID < vr::k_unMaxTrackedDeviceCount, "transform index out of range");
			++transforms[t.openVRID];
			Check(t.enabled == (t.openVRID == 3 || t.openVRID == 4 || t.openVRID == 6 || t.openVRID == 7), "head tracker must be excluded and only target system transformed");
			Check(t.updateTranslation && t.updateRotation && t.updateScale, "each transform must be complete");
			Check(std::isfinite(t.scale) && std::isfinite(t.rotation.w), "transform fields must be initialized");
		}
		if (command.type == protocol::RequestSetSlamSync)
		{
			const auto &s = command.setSlamSync;
			Check(s.openVRID < vr::k_unMaxTrackedDeviceCount, "sync index out of range");
			++syncs[s.openVRID];
			Check(s.enabled == (s.openVRID == 1 || s.openVRID == 2), "SLAM sync roles must be preserved");
		}
	}
	for (const auto &device : devices)
		Check(transforms[device.id] == 1 && syncs[device.id] == 1, "one command per device without reset-then-set");
	Check(commands.front().setHmdTracker.enabled && commands.front().setHmdTracker.trackerID == 5, "disconnected serial must retain override binding");
	Check(!DesiredState(devices, profile, true).front().setHmdTracker.enabled, "Sampling must disable the HMD override even when its serial resolves");
	for (bool sampling : { false, true })
	{
		if (!sampling) devices.erase(devices.begin() + 5);
		auto disabled = DesiredState(devices, profile, sampling).front().setHmdTracker;
		Check(!disabled.enabled && disabled.trackerID == 0 && disabled.hmdID < vr::k_unMaxTrackedDeviceCount, "disabled HMD packet must use trackerID zero");
	}
	Check(commands.back().type == protocol::RequestSetOneEuro, "reconnect replay must contain smoothing parameters");
	auto hmdDevices = Devices();
	hmdDevices[0].trackingSystem = "system-Y";
	hmdDevices[1].deviceClass = vr::TrackedDeviceClass_HMD;
	hmdDevices[1].trackingSystem = "system-Y";
	hmdDevices[2].deviceClass = vr::TrackedDeviceClass_HMD;
	for (const auto &command : DesiredState(hmdDevices, profile, false))
	{
		if (command.type == protocol::RequestSetDeviceTransform && command.setDeviceTransform.openVRID <= 2)
			Check(!command.setDeviceTransform.enabled, "HMD index and HMD class must never receive transforms");
		if (command.type == protocol::RequestSetSlamSync && command.setSlamSync.openVRID <= 2)
			Check(!command.setSlamSync.enabled, "HMD class at another index must never receive SLAM sync");
	}
	for (bool bound : { true, false })
	{
		auto legacy = Profile();
		if (!bound)
			legacy.trackerSerial.clear();
		auto before = DesiredState(Devices(), legacy, false);
		auto during = DesiredState(Devices(), legacy, true, 6);
		auto after = DesiredState(Devices(), legacy, false);
		int counts[vr::k_unMaxTrackedDeviceCount]{};
		for (size_t i = 0; i < during.size(); ++i)
			if (during[i].type == protocol::RequestSetDeviceTransform)
			{
				const auto &t = during[i].setDeviceTransform;
				++counts[t.openVRID];
				Check(t.enabled == (t.openVRID == 6 ? false : before[i].setDeviceTransform.enabled), "only the sampling target transform must be disabled");
				Check(after[i].setDeviceTransform.enabled == before[i].setDeviceTransform.enabled, "ending an attempt must restore the committed transform");
			}
		for (const auto &device : Devices())
			Check(counts[device.id] == 1, "sampling still requires exactly one transform command per device");
	}
	auto legacy = Profile();
	legacy.trackerSerial.clear();
	legacy.calibratedRotation = Eigen::Vector3d(30, 12, -15);
	legacy.calibratedScale = 1.035;
	auto legacyCommands = DesiredState(Devices(), legacy, false);
	ObservationSpace space;
	for (const auto &command : legacyCommands)
		space.Applied(command);
	auto pair = MakePair(0, 180, false, false, false);
	auto head = HeadSamples(0.08);
	std::vector<Sample> recoveredHead;
	HandPair recoveredPair{ 1, 3, {} };
	vr::TrackedDevicePose_t observed[vr::k_unMaxTrackedDeviceCount]{};
	for (size_t i = 0; i < pair.samples.size(); ++i)
	{
		observed[0] = TrackedPose(head[i].ref);
		observed[1] = TrackedPose(pair.samples[i].ref);
		observed[3] = TrackedPose(pair.samples[i].target);
		observed[5] = TrackedPose(head[i].target);
		for (const auto &command : legacyCommands)
			if (command.type == protocol::RequestSetDeviceTransform && command.setDeviceTransform.enabled)
			{
				const auto &t = command.setDeviceTransform;
				const auto &q = t.rotation;
				Eigen::Matrix3d rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
				Pose pose(observed[t.openVRID].mDeviceToAbsoluteTracking);
				pose.rot = rotation * pose.rot;
				pose.trans = t.scale * rotation * pose.trans + Eigen::Vector3d(t.translation.v[0], t.translation.v[1], t.translation.v[2]);
				observed[t.openVRID] = TrackedPose(pose);
			}
		auto raw = space.RawPoses(observed);
		recoveredPair.samples.push_back({ Pose(raw[1].mDeviceToAbsoluteTracking), Pose(raw[3].mDeviceToAbsoluteTracking) });
		recoveredHead.push_back({ Pose(raw[0].mDeviceToAbsoluteTracking), Pose(raw[5].mDeviceToAbsoluteTracking) });
	}
	auto recoveredSync = FitHandPairs({ recoveredPair });
	CheckSync(recoveredSync);
	Check(IsHeadTracker(recoveredHead, recoveredSync), "legacy roll/pitch transforms must not prevent yaw-only auto acquisition");
	ObservationSpace capturedAtPoseRead = space;
	legacy.enableNative = true;
	for (const auto &command : DesiredState(Devices(), legacy, false))
		space.Applied(command);
	auto previousFrame = capturedAtPoseRead.RawPoses(observed);
	Check((Pose(previousFrame[5].mDeviceToAbsoluteTracking).trans - head.back().target.trans).norm() < 1e-5,
		"pose acquisition must use its captured applied state rather than a later command");
	observed[5] = TrackedPose(head.back().target);
	auto native = space.RawPoses(observed);
	Check(Pose(native[5].mDeviceToAbsoluteTracking).rot == Pose(observed[5].mDeviceToAbsoluteTracking).rot
		&& Pose(native[5].mDeviceToAbsoluteTracking).trans == Pose(observed[5].mDeviceToAbsoluteTracking).trans,
		"Native mode must preserve already raw poses without a second transform");
}

static vr::TrackedDevicePose_t TrackedPose(const Pose &pose)
{
	vr::TrackedDevicePose_t result{};
	result.bDeviceIsConnected = result.bPoseIsValid = true;
	for (int row = 0; row < 3; ++row)
	{
		for (int column = 0; column < 3; ++column)
			result.mDeviceToAbsoluteTracking.m[row][column] = static_cast<float>(pose.rot(row, column));
		result.mDeviceToAbsoluteTracking.m[row][3] = static_cast<float>(pose.trans(row));
	}
	return result;
}

static void Scenario12()
{
	auto profile = Profile();
	const auto saved = EncodeProfile(profile);
	std::optional<CalibrationAttempt> attempt(std::in_place);
	attempt->serial = "uncommitted-B";
	attempt->result.relativeTranslation = { 2, 3, 4 };
	profile.state = CalibrationState::Sampling;
	Check(EncodeProfile(profile) == saved, "exit save must never contain attempt values");
	DiscardAttempt(profile, attempt);
	Check(!attempt && profile.state == CalibrationState::None && EncodeProfile(profile) == saved, "cancel must preserve committed serial and offsets");
	attempt.emplace();
	attempt->serial = "uncommitted-B";
	attempt->automatic = true;
	attempt->result.relativeTranslation = { 4, 5, 6 };
	DiscardAttempt(profile, attempt);
	Check(EncodeProfile(profile) == saved, "failed auto attempt must preserve committed profile");
	attempt.emplace();
	attempt->serial = "committed-A";
	attempt->trackingSystem = "system-Y";
	attempt->hmdSerial = "hmd-A";
	attempt->result.relativeTranslation = { 0.02, -0.07, 0.03 };
	CommitAttempt(profile, *attempt);
	Check(profile.validProfile && profile.trackerSerial == "committed-A" && profile.relativeTranslation.v[0] == 0.02, "successful attempt must commit its result");
	DiscardAttempt(profile, attempt);
	AutoAcquisition acquisition;
	acquisition.Refresh(Devices());
	Evaluation evidence;
	evidence.passing.push_back({ 5, "committed-A", "system-Y", 100, Eigen::Matrix3d::Identity(), true });
	Check(!acquisition.Confirm(evidence, acquisition.Epoch()), "confirmation must start with one pass");
	profile.continuousSync = false;
	RemoveCalibrationState(profile, attempt, acquisition);
	Check(!profile.continuousSync, "Remove must preserve the Relative Calibration setting");
	Check(profile.trackerSerial.empty() && !profile.validProfile && !profile.validRelativeOffset, "Remove must clear the binding");
	Check(!attempt && acquisition.Snapshot().empty(), "Remove must discard the attempt and all reservoirs");
	evidence.passing.front().count += 20;
	Check(!acquisition.Confirm(evidence, acquisition.Epoch()), "Remove must reset confirmations too");
	profile.validProfile = true;
	Check(AutoAcquisitionEligible(profile, true, true), "a valid profile without a tracker serial is unbound");
	Check(!AutoAcquisitionEligible(profile, false, true), "unresolved profile read must pause auto acquisition");
	Check(!AutoAcquisitionEligible(profile, true, false), "unresolved Settings read must pause auto acquisition");
	attempt.emplace();
	attempt->automatic = true;
	profile.state = CalibrationState::Sampling;
	Check(ApplyAutoAcquireState(profile, attempt, acquisition, false), "late-loaded auto off must end an automatic attempt");
	Check(!attempt && !profile.autoAcquire && profile.state == CalibrationState::None && !AutoAcquisitionEligible(profile, true, true), "loaded off must take effect immediately");
	attempt.emplace();
	profile.state = CalibrationState::Sampling;
	Check(!ApplyAutoAcquireState(profile, attempt, acquisition, false) && attempt.has_value(), "auto off must preserve a manual attempt");
	DiscardAttempt(profile, attempt);
	auto noticeProfile = Profile();
	auto found = FoundNotice("candidate-B");
	auto ready = ReadyNotice(noticeProfile);
	Check(found.type == vr::EVRNotificationType_Persistent && found.text == "Head tracker found: candidate-B. Look around naturally for a few seconds to finish.", "found notification must remain persistent");
	Check(ready.type == vr::EVRNotificationType_Transient && ready.text == "Head tracker ready: committed-A.", "ready notification must be transient and name the committed serial");
	Check(WaitingForTracker("bound-A") == "Waiting for head tracker bound-A.", "initial waiting status must name the tracker without reporting tracking loss");
	for (auto speed : { CalibrationContext::FAST, CalibrationContext::SLOW, CalibrationContext::VERY_SLOW })
	{
		noticeProfile.calibrationSpeed = speed;
		Check(CalibrationCapacity(noticeProfile) == (speed == CalibrationContext::FAST ? 200 : speed == CalibrationContext::SLOW ? 500 : 1000), "calibration and rigidity stores must use twice SampleCount");
	}
	CalibrationAttempt manual;
	profile.autoAcquire = false;
	Check(DescribeAcquisition(profile, &manual, false, false, false, 0) == AcquireState::Paused, "unbound manual calibration must report Paused even if auto is off or reads are pending");
	manual.automatic = true;
	profile.autoAcquire = true;
	Check(DescribeAcquisition(profile, &manual, true, true, true, 2) == AcquireState::Calibrating, "only an automatic attempt reports Calibrating");
	auto editing = Profile();
	editing.trackerSerial.clear();
	for (auto state : { CalibrationState::Editing, CalibrationState::Begin, CalibrationState::Detect, CalibrationState::WaitForTracker, CalibrationState::Sampling })
	{
		editing.state = state;
		Check(DescribeAcquisition(editing, nullptr, true, true, false, 0) == AcquireState::Paused,
			"unbound Editing and manual states without an automatic attempt must report Paused");
	}
	editing.autoAcquire = false;
	Check(DescribeAcquisition(editing, nullptr, false, false, false, 0) == AcquireState::Paused,
		"serial-less Editing must report Paused before off or unreadable status");
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
	for (const auto &device : Devices())
		poses[device.id] = TrackedPose(Pose{});
	auto devices = Devices();
	ManualDetection detection;
	Check(detection.Observe(0, devices, poses).event == ManualDetection::Event::Collecting && detection.Serials().size() == 2, "manual Begin must capture its first nonempty candidate list");
	detection.Observe(0.05, devices, poses);
	detection.Observe(0.10, devices, poses);
	auto progress = detection.Progress();
	poses[5].bPoseIsValid = false;
	Check(detection.Observe(0.15, devices, poses).event == ManualDetection::Event::Collecting && detection.Progress() == progress + 1
		&& detection.Speeds(0).back() == 0, "invalid manual candidate must contribute zero without pausing detection");
	poses[6].bPoseIsValid = false;
	Check(detection.Observe(0.20, devices, poses).event == ManualDetection::Event::Collecting && detection.Started()
		&& detection.Speeds(0).back() == 0 && detection.Speeds(1).back() == 0, "all missing candidates must preserve fixed membership and contribute zeros");
	poses[6].bPoseIsValid = true;
	Check(detection.Observe(0.25, devices, poses).event == ManualDetection::Event::Collecting && detection.Serials().size() == 2,
		"one returning candidate must not enable the singleton shortcut");
	Pose recovered;
	recovered.rot = Rotation(0, 1.0);
	poses[5] = TrackedPose(recovered);
	detection.Observe(0.30, devices, poses);
	Check(detection.Speeds(0).back() == 0, "first valid candidate tick must reset its baseline with zero speed");
	detection.Observe(0.35, devices, poses);
	Check(detection.Speeds(0).back() < 0.01, "candidate recovery must not leave a delayed rotation spike");
	Check(detection.Speeds(0).size() == detection.Progress() && detection.Speeds(1).size() == detection.Progress(),
		"all manual speed histories must remain aligned with HMD evidence");
	poses[5].mDeviceToAbsoluteTracking.m[0][0] = std::numeric_limits<float>::quiet_NaN();
	Check(detection.Observe(0.40, devices, poses).event == ManualDetection::Event::Collecting && detection.Speeds(0).back() == 0,
		"non-finite candidate pose must contribute zero rather than pause detection");
	poses[5] = TrackedPose(recovered);
	detection.Observe(0.45, devices, poses);
	Check(detection.Speeds(0).back() == 0, "candidate baseline must reset after a non-finite pose");
	poses[0].bPoseIsValid = false;
	progress = detection.Progress();
	Check(detection.Observe(0.50, devices, poses).event == ManualDetection::Event::HeadsetPaused && detection.Progress() == progress,
		"invalid HMD must still pause manual detection");
	poses[0] = TrackedPose(recovered);
	detection.Observe(0.55, devices, poses);
	Check(detection.Progress() == progress, "first HMD recovery tick must only refresh baselines");
	for (bool blinkHead : { false, true })
	{
		ManualDetection active;
		for (const auto &device : devices) poses[device.id] = TrackedPose(Pose{});
		active.Observe(0, devices, poses);
		uint32_t selected = vr::k_unTrackedDeviceIndexInvalid;
		for (int i = 1; i <= 90 && selected == vr::k_unTrackedDeviceIndexInvalid; ++i)
		{
			Pose moving;
			moving.rot = Rotation(0.25 * std::sin(i * 0.2), 0.45 * std::sin(i * 0.17));
			poses[0] = poses[5] = TrackedPose(moving);
			poses[6].bPoseIsValid = blinkHead;
			if (blinkHead && i == 12) poses[5].bPoseIsValid = false;
			auto result = active.Observe(i * 0.05, devices, poses);
			if (blinkHead && (i == 12 || i == 13))
				Check(result.event == ManualDetection::Event::Collecting && active.Serials().size() == 2 && active.Speeds(0).back() == 0,
					"head blink and first return must add zero without elimination selection");
			if (result.event == ManualDetection::Event::Selected) selected = result.id;
		}
		Check(selected == 5, blinkHead ? "a one-tick head blink must not prevent correlated selection" : "a permanently invalid candidate must not stall correlated head selection");
	}
	ManualDetection unresolved;
	for (const auto &device : devices) poses[device.id] = TrackedPose(Pose{});
	unresolved.Observe(0, devices, poses);
	for (int i = 1; i <= 40; ++i)
	{
		Pose moving;
		moving.rot = Rotation(0.25 * std::sin(i * 0.2), 0.45 * std::sin(i * 0.17));
		poses[0] = poses[5] = TrackedPose(moving);
		Check(unresolved.Observe(i * 0.05, devices, poses).event == ManualDetection::Event::Collecting,
			"unresolved selection fixture must retain its first 39 speed samples");
	}
	auto withoutHead = devices;
	withoutHead.erase(std::remove_if(withoutHead.begin(), withoutHead.end(), [](const DeviceSnapshot &device) { return device.id == 5; }), withoutHead.end());
	unresolved.Resolve(withoutHead);
	auto pending = unresolved.Observe(2.05, withoutHead, poses);
	Check(pending.event == ManualDetection::Event::Selected && pending.id == vr::k_unTrackedDeviceIndexInvalid && pending.serial == "committed-A",
		"selection must preserve the fixed candidate serial when its index is temporarily unresolved");
	Check(ResolveSerial(withoutHead, pending.serial) == vr::k_unTrackedDeviceIndexInvalid && ResolveSerial(devices, pending.serial) == 5,
		"the selected serial must resolve again when its device returns");
	for (const auto &device : devices) poses[device.id] = TrackedPose(Pose{});
	devices[1].deviceClass = vr::TrackedDeviceClass_GenericTracker;
	acquisition.Refresh(devices);
	Check(acquisition.Snapshot().front().pairs.size() == 2, "non-controller Left/Right role hints must be usable as hands");
	devices[2].role = vr::TrackedControllerRole_Invalid;
	acquisition.Refresh(devices);
	Check(acquisition.Snapshot().front().pairs.size() == 3, "unknown hand roles must be compatible with either controller");
	acquisition.Observe(0, poses);
	acquisition.Observe(0.05, poses);
	Check(acquisition.Snapshot().front().candidates.front().samples.size() == 1, "auto observer must retain paired HMD/candidate keyframes after two valid ticks");
	AutoAcquisition linkContinuity;
	linkContinuity.Refresh(devices);
	linkContinuity.Observe(0, poses);
	linkContinuity.Observe(0.05, poses);
	ObservationSpace missingConfiguration, configured;
	for (const auto &command : DesiredState(devices, CalibrationContext{}, false)) configured.Applied(command);
	Check(!missingConfiguration.HasConfiguration() && configured.HasConfiguration(), "observation space must expose configuration availability");
	auto linkEpoch = linkContinuity.Epoch();
	auto linkEvidence = evidence;
	Check(!linkContinuity.Confirm(linkEvidence, linkEpoch), "link continuity fixture starts with one confirmation pass");
	linkEvidence.passing.front().count += 20;
	Check(!linkContinuity.ObserveConfigured(0.10, poses, missingConfiguration), "unconfigured observation space must skip observation");
	Check(linkContinuity.Epoch() == linkEpoch && linkContinuity.Snapshot().front().candidates.front().samples.size() == 1,
		"driver link loss must preserve the observation epoch and reservoirs");
	Check(linkContinuity.ObserveConfigured(0.15, poses, configured)
		&& linkContinuity.Snapshot().front().pairs.front().samples.size() == 1
		&& linkContinuity.Snapshot().front().candidates.front().samples.size() == 1,
		"configured observations must resume without a false headset recovery reset");
	Check(linkContinuity.Confirm(linkEvidence, linkEpoch).has_value(), "driver link loss must preserve confirmation evidence");
	auto epoch = acquisition.Epoch();
	acquisition.ResetConfirmations();
	Check(!acquisition.Confirm(evidence, epoch), "initial current-epoch evidence must remain unconfirmed");
	evidence.passing.front().count += 20;
	poses[0].bPoseIsValid = false;
	acquisition.Observe(0.10, poses);
	Check(acquisition.Epoch() != epoch && !acquisition.Confirm(evidence, epoch), "HMD loss must invalidate ready worker results before recovery");
	poses[0].bPoseIsValid = true;
	acquisition.Observe(0.15, poses);
	Check(acquisition.Snapshot().front().pairs.front().samples.empty() && acquisition.Snapshot().front().candidates.front().samples.empty(), "HMD recovery must clear all acquisition stores");
	acquisition.Observe(0.20, poses);
	epoch = acquisition.Epoch();
	Pose turning;
	turning.rot = Rotation(0, 20 * Degrees);
	poses[0] = poses[5] = TrackedPose(turning);
	acquisition.Observe(0.25, poses);
	Check(acquisition.Epoch() == epoch, "fast natural head yaw with a still foot must not restart acquisition");
	auto afterTurn = acquisition.Snapshot();
	for (const auto &candidate : afterTurn.front().candidates)
		Check(candidate.samples.size() == 1, "stationary foot candidate must not wipe any candidate reservoir");
	Check(acquisition.Snapshot().front().pairs.front().samples.size() == 1, "fast head yaw must preserve hand-pair reservoirs");
	poses[0].mDeviceToAbsoluteTracking.m[0][0] = std::numeric_limits<float>::quiet_NaN();
	acquisition.Observe(0.30, poses);
	Check(acquisition.Epoch() != epoch, "non-finite HMD pose must enter acquisition loss handling");
	poses[0] = TrackedPose(turning);
	acquisition.Observe(0.35, poses);
	Check(acquisition.Snapshot().front().pairs.front().samples.empty(), "non-finite HMD recovery must reset acquisition");
	acquisition.Observe(0.40, poses);
	acquisition.Observe(181, poses);
	Check(acquisition.Snapshot().front().pairs.front().samples.empty() && acquisition.Snapshot().front().candidates.front().samples.empty(), "stationary acquisition keyframes older than 180 seconds must expire");
	acquisition.Observe(181.05, poses);
	poses[0].bPoseIsValid = false;
	acquisition.Observe(362, poses);
	Check(acquisition.Snapshot().front().pairs.front().samples.empty(), "acquisition expiry must also run during invalid observations");
	poses[0].bPoseIsValid = true;
	acquisition.Observe(362.05, poses);
	for (int i = 0; i < 450; ++i)
	{
		Pose translated;
		translated.trans.x() = i * 0.011;
		for (const auto &device : devices) poses[device.id] = TrackedPose(translated);
		acquisition.Observe(362.10 + i * 0.05, poses);
	}
	auto filled = acquisition.Snapshot();
	for (const auto &pair : filled.front().pairs)
		Check(pair.samples.size() == 400, "acquisition hand capacity must be fixed at 400");
	for (const auto &candidate : filled.front().candidates)
		Check(candidate.samples.size() == 400, "acquisition candidate capacity must be fixed at 400");
	auto beforeExpiry = filled.front().candidates.front().candidate;
	Evaluation earlierPass, pendingEvaluation;
	pendingEvaluation.passing.push_back(beforeExpiry);
	pendingEvaluation.passing.front().heightCertain = true;
	earlierPass = pendingEvaluation;
	earlierPass.passing.front().count -= 20;
	acquisition.ResetConfirmations();
	auto evaluationEpoch = acquisition.Epoch();
	Check(!acquisition.Confirm(earlierPass, evaluationEpoch), "expiry fixture starts with an earlier confirmation pass");
	acquisition.Observe(600, poses);
	auto expired = acquisition.Snapshot();
	Check(expired.front().candidates.front().samples.empty() && expired.front().candidates.front().candidate.count == beforeExpiry.count,
		"expiry must remove old samples while retaining monotonic evidence counts");
	Check(acquisition.Epoch() == evaluationEpoch && acquisition.Confirm(pendingEvaluation, evaluationEpoch).has_value(),
		"an evaluation launched before expiry must remain confirmable with its captured epoch");
	PersistenceState storage;
	auto readRevision = storage.revision;
	storage.Changed();
	storage.Saved(false);
	Check(storage.dirty && storage.resolved && !storage.AcceptRead(readRevision), "late startup read must not replace user changes after save failure");
	storage.Saved(true);
	Check(!storage.dirty, "successful retry must clear the pending save");
}

static void Scenario13()
{
	const std::string legacy = R"([{"target_tracking_system":"system-Y","tracker_serial":"committed-A","hmd_serial":"hmd-A","roll":1.25,"yaw":59.5,"pitch":-0.75,"x":120,"y":30,"z":-70,"scale":1,"targetModelScale":1.0034,"hmdScale":1.012,"native":false,"fallbackSlam":true,"eAngVel":true,"continuousSync":false,"predictionTime":2.5,"headFilterEnabled":false,"headFilter":{"minCutoff":5,"beta":0.8,"dCutoff":1},"driftFilter":{"minCutoff":3,"beta":1.3,"dCutoff":0.6},"rel_qw":1,"rel_qx":0,"rel_qy":0,"rel_qz":0,"rel_tx":0.01,"rel_ty":-0.08,"rel_tz":0.02,"calibration_speed":2,"chaperone":{"auto_apply":false,"play_space_size":[2,3],"standing_center":[1,0,0,0,0,1,0,0,0,0,1,0],"geometry":[0,0,0,1,0,0,1,2,0,0,2,0]},"future_field":{"ignored":true}}])";
	CalibrationContext profile;
	std::string error;
	Check(DecodeProfile(legacy, profile, error), "upstream v7 profile shape must load");
	Check(profile.trackerSerial == "committed-A" && profile.validRelativeOffset && profile.chaperone.valid && profile.chaperone.geometry.size() == 1, "binding, offsets and chaperone must load");
	Check(profile.headFilterParams.beta == 0.8 && profile.driftFilterParams.dCutoff == 0.6 && !profile.continuousSync && profile.predictionTime == 2.5f, "upstream smoothing/settings values must survive");
	Check(profile.calibrationSpeed == CalibrationContext::VERY_SLOW && profile.hmdScale == 1.012, "speed and scale must survive");
	CalibrationContext again;
	std::string encoded = EncodeProfile(profile);
	Check(DecodeProfile(encoded, again, error) && EncodeProfile(again) == encoded, "upstream profile must round-trip");
	Check(!DecodeProfile("broken JSON", again, error) && EncodeProfile(again) == encoded, "parse error must leave memory unchanged");
	Check(!DecodeProfile("{}", again, error) && EncodeProfile(again) == encoded, "wrong shape must be a parse error rather than an empty profile");
	Check(DecodeProfile(R"([{"target_tracking_system":"Y","roll":0,"yaw":0,"pitch":0,"x":0,"y":0,"z":0}])", again, error), "missing optional fields must default");
	Check(again.hmdScale == 1 && again.calibratedScale == 1 && again.fallbackToSlam && !again.validRelativeOffset, "optional profile defaults must match upstream");
	Check(DecodeProfile("", again, error) && !again.validProfile && again.trackerSerial.empty(), "legacy empty Config is unbound rather than unreadable");
	bool enabled = false;
	Check(DecodeSettings("", enabled, error) && enabled, "absent settings must default auto on");
	Check(DecodeSettings(R"({"future":1})", enabled, error) && enabled, "unknown settings fields must be ignored");
	Check(DecodeSettings(EncodeSettings(false), enabled, error) && !enabled, "unbound auto toggle must round-trip");
	Check(!DecodeSettings("[", enabled, error) && !enabled, "settings error must preserve memory");
	CalibrationContext beforeRead, ui = beforeRead, loaded;
	ui.headFilterParams.beta = 2.0;
	ui.driftFilterParams.dCutoff = 1.25;
	ui.predictionTime = 3.5f;
	ui.fallbackToSlam = false;
	ui.calibrationSpeed = CalibrationContext::SLOW;
	Check(DecodeProfile(legacy, loaded, error), "retry must decode the stored profile");
	PreserveChangedSettings(loaded, ui, beforeRead);
	Check(loaded.trackerSerial == "committed-A" && loaded.hmdScale == 1.012, "settings merge must retain the recovered binding and calibration");
	Check(loaded.headFilterParams.beta == 2.0 && loaded.driftFilterParams.dCutoff == 1.25 && loaded.predictionTime == 3.5f
		&& !loaded.fallbackToSlam && loaded.calibrationSpeed == CalibrationContext::SLOW, "late profile read must preserve direct UI settings edits");
	Check(!loaded.continuousSync && loaded.enableAngularVelocity, "untouched settings must still come from the stored profile");
	PersistenceState profileStorage, settingsStorage;
	auto memory = Profile();
	memory.continuousSync = false;
	memory.headFilterParams.beta = 2.5;
	auto untouched = EncodeProfile(memory);
	Check(!ApplyProfileRead(ReadResult::Error, "", memory, beforeRead, profileStorage, error) && !profileStorage.resolved
		&& EncodeProfile(memory) == untouched, "registry read errors must stay unresolved and preserve memory");
	Check(ApplyProfileRead(ReadResult::Present, "broken", memory, beforeRead, profileStorage, error) && !error.empty(), "malformed profile read must report its parse error once");
	Check(profileStorage.resolved && !profileStorage.dirty && memory.trackerSerial.empty() && !memory.validProfile
		&& !memory.continuousSync && memory.headFilterParams.beta == 2.5, "malformed profile must resolve unbound without rewriting or resetting user settings");
	Check(!ApplyProfileRead(ReadResult::Present, "broken", memory, beforeRead, profileStorage, error), "resolved malformed profile must not retry or log again");
	memory.autoAcquire = false;
	Check(!ApplySettingsRead(ReadResult::Error, "", memory, settingsStorage, error) && !settingsStorage.resolved && !memory.autoAcquire,
		"Settings registry error must preserve its unresolved state and in-memory value");
	Check(ApplySettingsRead(ReadResult::Present, "[", memory, settingsStorage, error) && !error.empty()
		&& settingsStorage.resolved && !settingsStorage.dirty && memory.autoAcquire, "malformed Settings must resolve default on without writing");
	Check(!ApplySettingsRead(ReadResult::Present, "[", memory, settingsStorage, error), "malformed Settings must be logged only once");
	auto disabled = legacy;
	disabled.insert(disabled.find('{') + 1, "\"valid_profile\":false,");
	Check(DecodeProfile(disabled, memory, error) && !memory.validProfile && memory.trackerSerial.empty()
		&& !memory.validRelativeOffset && memory.relativeRotation.w == 1 && memory.relativeTranslation.v[1] == 0,
		"invalid stored profile must not retain a binding or relative offset");
}

static HandPair RoughPair(size_t count = 12, double spread = 0.101)
{
	HandPair pair;
	for (size_t i = 0; i < count; ++i)
	{
		Sample sample;
		double angle = 2 * Pi * i / count;
		sample.target.trans = Eigen::Vector3d(spread * std::cos(angle), 1, spread * std::sin(angle));
		sample.ref.trans = SyncRotation * sample.target.trans + SyncTranslation;
		sample.sequence = i + 1;
		pair.samples.push_back(sample);
	}
	return pair;
}

static std::vector<Sample> YawHead(const std::vector<double> &angles)
{
	std::vector<Sample> samples;
	for (double angle : angles)
	{
		Sample sample;
		sample.ref.rot = Rotation(0, angle);
		sample.ref.trans = Eigen::Vector3d(0, 1.65, 0);
		sample.target.rot = SyncRotation.transpose() * sample.ref.rot;
		sample.target.trans = SyncRotation.transpose() * (sample.ref.trans + Eigen::Vector3d(0, 0.08, 0) - SyncTranslation);
		sample.sequence = samples.size() + 1;
		samples.push_back(sample);
	}
	return samples;
}

static void Scenario14()
{
	Check(FitHandPairs({ RoughPair() }).held, "12 keyframes and 10.1 cm spread must hold");
	Check(!FitHandPairs({ RoughPair(11) }).held, "11 hand keyframes must not hold");
	Check(!FitHandPairs({ RoughPair(12, 0.099) }).held, "9.9 cm spread must not hold");
	auto pair = RoughPair();
	for (size_t k : { 0, 5, 10 }) pair.samples[k].ref.trans.x() += k == 10 ? 0.149 : 0.079;
	Check(FitHandPairs({ pair }).held, "7.9 cm median and 14.9 cm p90 must hold");
	pair.samples[0].ref.trans.x() += 0.002;
	pair.samples[5].ref.trans.x() += 0.002;
	Check(!FitHandPairs({ pair }).held, "8.1 cm hand median must reject");
	pair.samples[0].ref.trans.x() -= 0.002;
	pair.samples[5].ref.trans.x() -= 0.002;
	pair.samples[10].ref.trans.x() += 0.002;
	Check(!FitHandPairs({ pair }).held, "15.1 cm hand p90 must reject");

	auto yaw = YawHead({ 0, 0, 0, 0, 0, 0.1501, 0.1501, 0.075 });
	auto sync = KnownSync(); sync.heightCertain = false;
	auto check = CheckHeadTracker(yaw, sync);
	Check(check.rigidity.deltas == 10 && IsHeadTracker(yaw, sync), "8 frames and 10 deltas must detect yaw-only head at 60 degree sync");
	yaw.pop_back();
	Check(!FitRigidity(yaw).observable, "7 candidate keyframes must reject even with 10 deltas");
	Check(!FitRigidity(YawHead({ 0, 0, 0, 0.16, 0.16, 0.16, 0.08, 0.08 })).observable, "9 valid deltas must reject");
	Check(!FitRigidity(YawHead({ 0, 0, 0, 0, 0, 0.1499, 0.1499, 0.075 })).observable, "rotations below 0.15 rad must not form deltas");
	auto mismatched = YawHead({ 0, 0, 0, 0, 0, 0.16, 0.16, 0.08 });
	for (size_t i = 5; i < 7; ++i) mismatched[i].target.rot = SyncRotation.transpose() * Rotation(0, 0.14);
	Check(!FitRigidity(mismatched).observable, "both rotations must reach the delta threshold");
	for (double angle : { 4.5, 5.5, 11.0, 13.0 })
	{
		auto noisy = HeadSamples(0.08);
		for (size_t i = 0; i < noisy.size(); ++i)
			if (angle < 6 || i % 5 == 0)
				noisy[i].target.rot = noisy[i].target.rot * Rotation((i % 2 ? -1 : 1) * angle * Degrees, 0);
		Check(IsHeadTracker(noisy, sync) == (angle == 4.5 || angle == 11.0), "rigidity median 5 deg and p90 12 deg must bound rough detection");
	}
	for (double angle : { 14.0, 16.0 })
	{
		auto otherSync = sync; otherSync.rotation = Rotation(0, (60 + angle) * Degrees);
		auto head = HeadSamples(0.08);
		for (auto &sample : head)
			sample.target.trans = otherSync.rotation.transpose() * (sample.ref.trans + sample.ref.rot * Eigen::Vector3d(0, 0.08, 0) - otherSync.translation);
		Check(IsHeadTracker(head, otherSync) == (angle == 14), "sync rotation agreement must allow 14 deg and reject 16 deg");
	}
	Check(IsHeadTracker(HeadSamples(0.349), sync) && !IsHeadTracker(HeadSamples(0.351), sync), "proximity median must use 35 cm");
	Check(IsHeadTracker(HeadSamples(0.179, true, false), sync) && !IsHeadTracker(HeadSamples(0.181, true, false), sync), "vertical prior must use minus 18 cm even with uncertain height");
	for (double spread : { 0.149, 0.151 })
	{
		auto head = HeadSamples(0.08);
		for (size_t i = 0; i < head.size(); i += 4)
			head[i].target.trans += SyncRotation.transpose() * head[i].ref.rot * Eigen::Vector3d(spread, 0, 0);
		Check(IsHeadTracker(head, sync) == (spread < 0.15), "local offset spread p90 must use 15 cm");
	}
	Hypothesis hypothesis;
	hypothesis.pairs = { RoughPair() };
	hypothesis.candidates.push_back({ { 5, "head", "system-Y", 8 }, YawHead({ 0, 0, 0, 0, 0, 0.16, 0.16, 0.08 }) });
	auto diagnostic = Evaluate({ hypothesis }, true);
	Check(diagnostic.pairs.size() == 1 && diagnostic.candidates.size() == 1 && diagnostic.passing.size() == 1,
		"diagnostics must report each pair and candidate while preserving decisions");
	hypothesis.pairs.front().samples.clear();
	diagnostic = Evaluate({ hypothesis }, true);
	Check(diagnostic.candidates.size() == 1 && std::string(diagnostic.candidates.front().check.failure) == "hand-sync",
		"candidate diagnostic must include the first failure before hand sync holds");
}

static size_t SpeedFrames(uint32_t id, double position, double rotation, bool pair)
{
	AutoAcquisition acquisition;
	acquisition.Refresh(Devices());
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
	for (const auto &device : Devices()) poses[device.id] = TrackedPose(Pose{});
	acquisition.Observe(0, poses);
	Pose moved;
	moved.trans.x() = position * 0.05;
	moved.rot = Rotation(rotation * 0.05, 0);
	poses[id] = TrackedPose(moved);
	acquisition.Observe(0.05, poses);
	auto snapshot = acquisition.Snapshot();
	return pair ? snapshot.front().pairs.front().samples.size() : snapshot.front().candidates.front().samples.size();
}

static void Scenario15()
{
	Check(SpeedFrames(1, 0.99, 8, true) == 1 && SpeedFrames(1, 1.01, 0, true) == 0, "hand position gate must be 1 m/s and ignore hand rotation");
	Check(SpeedFrames(3, 0.99, 2.99, true) == 1 && SpeedFrames(3, 1.01, 0, true) == 0 && SpeedFrames(3, 0, 3.01, true) == 0,
		"controller gates must be 1 m/s and 3 rad/s");
	for (uint32_t id : { 0, 5 })
		Check(SpeedFrames(id, 0.59, 1.99, false) == 1 && SpeedFrames(id, 0.61, 0, false) == 0 && SpeedFrames(id, 0, 2.01, false) == 0,
			"HMD and tracker acquisition gates must be 0.6 m/s and 2 rad/s");
	AutoAcquisition acquisition;
	auto devices = Devices();
	devices[5].serial = "head";
	devices.push_back({ 8, vr::TrackedDeviceClass_GenericTracker, vr::TrackedControllerRole_Invalid, "foot", "system-Y", true, true });
	acquisition.Refresh(devices);
	std::mt19937 random(20261008);
	std::normal_distribution<double> noise(0, 1);
	double confirmedAt = -1;
	bool twoHeld = false;
	for (int tick = 0; tick <= 300; ++tick)
	{
		double time = tick * 0.05;
		vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
		for (int hand = 0; hand < 2; ++hand)
		{
			Pose controller = Controller(time, hand, true), delayed = Controller(time - 0.060, hand, true), reference;
			Eigen::Vector3d offset(0.03 + hand * 0.02, 0.025, -0.04);
			reference.trans = SyncRotation * (delayed.trans + delayed.rot * offset) + SyncTranslation;
			for (int axis = 0; axis < 3; ++axis)
			{
				reference.trans(axis) += 0.003 * noise(random);
				controller.trans(axis) += 0.001 * noise(random);
			}
			poses[1 + hand] = TrackedPose(reference);
			poses[3 + hand] = TrackedPose(controller);
		}
		Pose head, tracker, chest, foot;
		head.rot = Rotation(0.38 * std::sin(time * 0.8), 0.5 * std::sin(time * 0.6), 0.15 * std::sin(time * 0.4));
		head.trans = Eigen::Vector3d(0.1 * std::sin(time * 0.4), 1.65 + 0.02 * std::sin(time), 0.1 * std::cos(time * 0.3));
		tracker.rot = SyncRotation.transpose() * head.rot * Rotation(0.12, -0.24, 0.15);
		tracker.trans = SyncRotation.transpose() * (head.trans + head.rot * Eigen::Vector3d(0, 0.08, 0) - SyncTranslation);
		chest.rot = SyncRotation.transpose() * Rotation(0.1 * std::sin(time * 0.3), 0.2 * std::sin(time * 0.4));
		chest.trans = SyncRotation.transpose() * (head.trans + Eigen::Vector3d(0, -0.27, 0) - SyncTranslation);
		foot.trans = SyncRotation.transpose() * (Eigen::Vector3d(0.1, 0.05, 0.1) - SyncTranslation);
		for (int axis = 0; axis < 3; ++axis)
		{
			head.trans(axis) += 0.001 * noise(random);
			tracker.trans(axis) += 0.001 * noise(random);
		}
		poses[0] = TrackedPose(head); poses[5] = TrackedPose(tracker);
		poses[6] = TrackedPose(chest); poses[8] = TrackedPose(foot);
		acquisition.Observe(time, poses);
		if (tick % 20 == 0)
		{
			auto evaluation = Evaluate(acquisition.Snapshot());
			twoHeld = twoHeld || evaluation.handPairs == 2;
			for (const auto &passing : evaluation.passing)
				Check(passing.serial == "head", "natural motion must never pass the chest or foot");
			auto confirmed = acquisition.Confirm(evaluation, acquisition.Epoch());
			if (confirmed)
			{
				Check(confirmed->serial == "head", "natural motion must only confirm the head tracker");
				if (confirmedAt < 0) confirmedAt = time;
			}
		}
	}
	Check(twoHeld, "natural motion fixture must hold both latency/noise hand pairs");
	Check(confirmedAt >= 0 && confirmedAt <= 15, "20 Hz natural motion must confirm within 15 seconds");
	std::printf("SPEED: head confirmed at %.2f s (20 Hz, 60 ms hand latency, noise, chest and foot present)\n", confirmedAt);
}

static void Scenario16()
{
	Hypothesis hypothesis;
	hypothesis.pairs = { MakePair(0), MakePair(1) };
	auto frames = HeadSamples(0.08);
	hypothesis.candidates.push_back({ { 5, "head", "system-Y", frames.size() - 5 }, std::vector<Sample>(frames.begin(), frames.end() - 5) });
	AutoAcquisition acquisition;
	Check(!acquisition.Confirm(Evaluate({ hypothesis }), acquisition.Epoch()), "seed fixture needs first confirmation pass");
	hypothesis.candidates.front().samples = frames;
	hypothesis.candidates.front().candidate.count = frames.size();
	auto confirmation = acquisition.Confirm(Evaluate({ hypothesis }), acquisition.Epoch());
	Check(confirmation.has_value(), "five new evaluated candidate frames must provide a seed");
	CalibrationContext context;
	auto attempt = BeginAutomaticAttempt(*confirmation, CalibrationCapacity(context));
	Check(attempt.automatic && attempt.serial == "head" && attempt.trackingSystem == "system-Y" && attempt.id == 5,
		"automatic attempt must retain the confirmed identity");
	const auto &seed = attempt.sampler.Store().Samples();
	Check(seed.size() == frames.size(), "confirmation must seed every candidate frame within capacity");
	for (size_t i = 0; i < seed.size(); ++i)
		Check(seed[i].ref.trans == frames[i].ref.trans && seed[i].target.trans == frames[i].target.trans
			&& seed[i].ref.rot == frames[i].ref.rot && seed[i].target.rot == frames[i].target.rot
			&& seed[i].sequence == frames[i].sequence, "seed poses and sequence must remain raw and unchanged");
	Check(CalibrationReadyToSolve(attempt, context.SampleCount()), "sufficient seed must be immediately solve-ready without new observations");
	Check(AutomaticCalibrationSucceeded(SolveCalibration(attempt.sampler.Snapshot(), 1.0)), "seed alone must solve successfully without new samples");
	Check(attempt.rigiditySampler.Store().Samples().empty(), "abandon evidence must start empty after seeding");
	attempt.lastSolveCount = attempt.sampler.Store().Count();
	Check(!CalibrationReadyToSolve(attempt, context.SampleCount()), "an already solved seed needs genuinely new samples before retry");
	auto newer = frames.back();
	attempt.sampler.Observe(30, newer.ref, true, newer.target, true, CalibrationCapacity(context));
	newer.ref.trans.x() += 0.015; newer.target.trans += SyncRotation.transpose() * Eigen::Vector3d(0.015, 0, 0);
	Check(attempt.sampler.Observe(30.05, newer.ref, true, newer.target, true, CalibrationCapacity(context)) == SampleEvent::Accepted,
		"normal consecutive sampling must continue after the seed");
	Check(CalibrationReadyToSolve(attempt, context.SampleCount()), "new samples after seed must enable another solve");
	attempt.sampler.Observe(30.10, newer.ref, false, newer.target, true, CalibrationCapacity(context));
	Check(attempt.sampler.Observe(30.15, newer.ref, true, newer.target, true, CalibrationCapacity(context)) == SampleEvent::SegmentRestarted
		&& attempt.sampler.Store().Samples().empty(), "tracking recovery must invalidate a seeded segment just like normal samples");
}

static void Scenario17()
{
	Check(overlaylog::WindowsError(0xE1234567UL) == std::to_string(0xE1234567UL), "unmapped Windows error must preserve its numeric code");
	Check(!overlaylog::ShouldRotate(0) && !overlaylog::ShouldRotate(2 * 1024 * 1024)
		&& overlaylog::ShouldRotate(2 * 1024 * 1024 + 1), "log rotation must occur only above 2 MiB");
	auto directory = std::filesystem::path("out/fast-acquisition-verification/log-test");
	std::filesystem::create_directories(directory);
	Check(!overlaylog::Append(directory, "unwritable destination"), "writing to a directory must fail without throwing");
	auto path = directory / "overlay.log";
	{ std::ofstream file(path, std::ios::binary | std::ios::trunc); file << "small"; }
	overlaylog::Prepare(path);
	Check(std::filesystem::file_size(path) == 5, "a small log must survive startup");
	{ std::ofstream file(path, std::ios::binary | std::ios::trunc); file.seekp(2 * 1024 * 1024); file.put('x'); }
	{ std::ofstream old(directory / "overlay.old.log", std::ios::trunc); old << "previous"; }
	overlaylog::Prepare(path);
	Check(!std::filesystem::exists(path) && std::filesystem::file_size(directory / "overlay.old.log") == 2 * 1024 * 1024 + 1,
		"startup rotation must replace the previous old log");
	Check(overlaylog::Append(path, "event serial=\"\xED\x97\xA4\xEB\x93\x9C\"\nnext\rline"), "file logging must recover after a failed write");
	std::ifstream file(path, std::ios::binary);
	std::string line((std::istreambuf_iterator<char>(file)), {});
	Check(line.size() > 13 && line[2] == ':' && line[5] == ':' && line[8] == '.' && line[12] == ' '
		&& std::count(line.begin(), line.end(), '\n') == 1 && line.find("\\nnext\\rline") != std::string::npos
		&& line.find("\xED\x97\xA4\xEB\x93\x9C") != std::string::npos, "log must contain local milliseconds and one UTF-8 line per event");
}

static void Scenario18()
{
	const double nan = std::numeric_limits<double>::quiet_NaN();
	auto head = HeadSamples(0.08);
	Check(IsHeadTracker(head, KnownSync()), "NaN fixture must start with a passing head tracker");
	for (bool reference : { false, true })
		for (bool all : { false, true })
		{
			auto invalid = head;
			for (size_t i = 0; i < (all ? invalid.size() : 1); ++i)
				(reference ? invalid[i].ref : invalid[i].target).trans.x() = nan;
			auto check = CheckHeadTracker(invalid, KnownSync());
			Check(std::isnan(check.distance.median) && std::isnan(check.spread.p90),
				"even one NaN sample must propagate into candidate statistics");
			Check(!IsHeadTracker(invalid, KnownSync()), "NaN sample statistics must never pass");
		}
	for (bool rotation : { false, true })
	{
		auto sync = KnownSync();
		if (rotation) sync.rotation(0, 0) = nan;
		else sync.translation.y() = nan;
		auto check = CheckHeadTracker(head, sync);
		Check(std::isnan(check.distance.median), "NaN sync fixture must produce NaN distance statistics");
		Check(!IsHeadTracker(head, sync), "NaN sync statistics must never pass");
	}
	for (bool reference : { false, true })
	{
		auto invalid = head;
		(reference ? invalid.front().ref : invalid.front().target).rot(0, 0) = nan;
		auto check = CheckHeadTracker(invalid, KnownSync());
		Check(check.rigidity.observable && std::isnan(check.rigidity.error.median),
			"mixed NaN rotations must exercise observable NaN rigidity statistics");
		Check(std::string(check.failure) == "rotation-median" && !IsHeadTracker(invalid, KnownSync()),
			"NaN rigidity must fail at the first rotation criterion");
	}
}

static std::vector<Sample> PitchedTracker(double pitch, bool chest, size_t count = 60)
{
	std::vector<Sample> samples;
	for (size_t i = 0; i < count; ++i)
	{
		Sample sample;
		sample.sequence = i + 1;
		sample.time = i * 0.15;
		Eigen::Matrix3d body = Rotation(0, 0.55 * std::sin(sample.time));
		sample.ref.rot = body * Rotation(-pitch * Degrees, 0);
		sample.ref.trans = Eigen::Vector3d(0, 1.65, 0);
		Eigen::Matrix3d trackerRotation = chest ? body : sample.ref.rot;
		// OpenVR forward is -Z; the back-of-head mount is +Z in the head frame.
		Eigen::Vector3d offset = chest ? Eigen::Vector3d(0, -0.25, -0.10) : Eigen::Vector3d(0, 0.03, 0.08);
		sample.target.rot = SyncRotation.transpose() * trackerRotation * Rotation(0.12, -0.24, 0.15);
		sample.target.trans = SyncRotation.transpose() * (sample.ref.trans + trackerRotation * offset - SyncTranslation);
		samples.push_back(sample);
	}
	return samples;
}

static void Scenario19()
{
	Check(std::string(CheckHeadTracker(HeadSamples(0.27, true, false), KnownSync()).failure) == "vertical",
		"head-local vertical failure must precede world vertical failure");
	for (double pitch : { 40.0, 50.0, 60.0 })
		for (double error : { -0.05, 0.0, 0.05 })
			for (bool heightCertain : { false, true })
			{
				auto sync = KnownSync();
				sync.translation.y() += error;
				sync.heightCertain = heightCertain;
				auto chest = PitchedTracker(pitch, true);
				auto check = CheckHeadTracker(chest, sync);
				Check(check.rigidity.observable && check.rigidity.error.median <= 5 * Degrees
					&& check.rigidity.error.p90 <= 12 * Degrees && check.agreement <= 15 * Degrees,
					"looking-down chest fixture must pass every rigidity criterion");
				Check(check.distance.median <= 0.35 && check.spread.p90 <= 0.15 && check.vertical.median >= -0.18,
					"looking-down chest fixture must pass distance, spread and head-local vertical");
				Check(std::abs(check.verticalWorld.median - (-0.25 + error)) < 1e-12,
					"world vertical statistics must retain the synced gravity-aligned difference");
				Check(std::string(check.failure) == "vertical-world" && !IsHeadTracker(chest, sync),
					"looking-down chest must fail vertical-world for every pitch and sync height error");
			}
}

static void Scenario20()
{
	for (double pitch : { 40.0, 50.0, 60.0 })
		for (double error : { -0.05, 0.0, 0.05 })
		{
			Hypothesis hypothesis;
			hypothesis.system = "system-Y";
			hypothesis.pairs = { MakePair(0, 240, false, false, false), MakePair(1, 240, false, false, false) };
			for (auto &pair : hypothesis.pairs)
				for (auto &sample : pair.samples)
					sample.ref.trans.y() += error;
			auto sync = CombineHandPairs(hypothesis.pairs);
			Check(sync.held && std::abs(sync.translation.y() - SyncTranslation.y() - error) < 0.005,
				"chest confirmation fixture must hold hand sync with the requested vertical error");
			AutoAcquisition acquisition;
			for (size_t count : { 60, 65, 70, 75 })
			{
				hypothesis.candidates = { { { 6, "chest", "system-Y", count }, PitchedTracker(pitch, true, count) } };
				auto evaluation = Evaluate({ hypothesis }, true);
				Check(!acquisition.Confirm(evaluation, acquisition.Epoch()),
					"a lone looking-down chest must never confirm across growing evidence");
			}
		}
}

static void Scenario21()
{
	for (double pitch : { 40.0, 50.0, 60.0 })
		for (double error : { -0.05, 0.0, 0.05 })
			for (bool heightCertain : { false, true })
			{
				auto sync = KnownSync();
				sync.translation.y() += error;
				sync.heightCertain = heightCertain;
				Check(IsHeadTracker(PitchedTracker(pitch, false), sync),
					"back-of-head tracker must pass while looking down despite sync height error");
			}
}

static protocol::DriftState DriftState(const CalibrationContext &profile)
{
	const auto hmd = DesiredState({}, profile, false).front().setHmdTracker;
	Eigen::Quaterniond drift(Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitY()));
	return { 0xfedcba9876543210ULL, true, 120, { drift.w(), drift.x(), drift.y(), drift.z() }, { 0.24, -0.13, 0.36 },
		profile.calibratedScale / profile.hmdScale, true, false, hmd.offsetRotation, hmd.offsetTranslation,
		hmd.calibrationRotation, hmd.calibrationTranslation, hmd.calibrationScale, hmd.hmdScale };
}

static Eigen::Quaterniond Q(const vr::HmdQuaternion_t &q) { return { q.w, q.x, q.y, q.z }; }
static Eigen::Vector3d V(const vr::HmdVector3d_t &v) { return { v.v[0], v.v[1], v.v[2] }; }

static std::vector<double *> DriftNumbers(protocol::DriftState &state)
{
	return { &state.rotation.w, &state.rotation.x, &state.rotation.y, &state.rotation.z,
		&state.translation.v[0], &state.translation.v[1], &state.translation.v[2], &state.slamScale,
		&state.offsetRotation.w, &state.offsetRotation.x, &state.offsetRotation.y, &state.offsetRotation.z,
		&state.offsetTranslation.v[0], &state.offsetTranslation.v[1], &state.offsetTranslation.v[2],
		&state.calibrationRotation.w, &state.calibrationRotation.x, &state.calibrationRotation.y, &state.calibrationRotation.z,
		&state.calibrationTranslation.v[0], &state.calibrationTranslation.v[1], &state.calibrationTranslation.v[2],
		&state.calibrationScale, &state.hmdScale };
}

static std::vector<double *> ProfileBasisNumbers(CalibrationContext &profile)
{
	return { &profile.calibratedRotation.x(), &profile.calibratedRotation.y(), &profile.calibratedRotation.z(),
		&profile.calibratedTranslation.x(), &profile.calibratedTranslation.y(), &profile.calibratedTranslation.z(),
		&profile.relativeRotation.w, &profile.relativeRotation.x, &profile.relativeRotation.y, &profile.relativeRotation.z,
		&profile.relativeTranslation.v[0], &profile.relativeTranslation.v[1], &profile.relativeTranslation.v[2],
		&profile.calibratedScale, &profile.hmdScale };
}

static void Scenario22()
{
	std::mt19937 random(0x5ace);
	std::uniform_real_distribution<double> number(-1.0, 1.0);
	for (double scale : { 1.0, 1.03 })
		for (int trial = 0; trial < 100; ++trial)
		{
			auto profile = Profile();
			profile.calibratedRotation = Eigen::Vector3d(number(random), number(random), number(random)) * 179.0;
			profile.calibratedTranslation = Eigen::Vector3d(number(random), number(random), number(random)) * 250.0;
			profile.calibratedScale = 1.017;
			profile.hmdScale = profile.calibratedScale / scale;
			auto state = DriftState(profile);
			Eigen::Quaterniond drift(Eigen::AngleAxisd(number(random) * EIGEN_PI, Eigen::Vector3d::UnitY()));
			state.rotation = { drift.w(), drift.x(), drift.y(), drift.z() };
			state.translation = { number(random), number(random), number(random) };
			const auto original = profile;
			const auto folded = spacememory::Fold(state);
			profile.calibratedRotation = folded.rotation;
			profile.calibratedTranslation = folded.translation;
			const auto packet = DesiredState(Devices(), profile, false).front().setHmdTracker;
			const Eigen::Quaterniond expected = drift.inverse() * Q(state.calibrationRotation);
			Check((Q(packet.calibrationRotation).toRotationMatrix() - expected.toRotationMatrix()).norm() < 1e-12,
				"folded Euler degrees must round-trip through DesiredState with nonzero roll and pitch");
			Check((V(packet.calibrationTranslation) - drift.inverse() * (V(state.calibrationTranslation) - V(state.translation))).norm() < 1e-12,
				"folded translation must rotate the subtracted drift and preserve centimeter conversion");
			for (int point = 0; point < 8; ++point)
			{
				Eigen::Vector3d raw(number(random), number(random), number(random));
				Eigen::Vector3d lighthouse = Q(state.calibrationRotation).inverse()
					* (drift * (scale * raw) + V(state.translation) - V(state.calibrationTranslation)) / original.calibratedScale;
				Eigen::Vector3d corrected = Q(packet.calibrationRotation) * (packet.calibrationScale * lighthouse) + V(packet.calibrationTranslation);
				Check((corrected - scale * raw).norm() < 1e-11, "fold must align lighthouse and scaled raw points for both SLAM scales");
			}
			Check(profile.calibratedScale == original.calibratedScale && profile.hmdScale == original.hmdScale
				&& profile.targetModelScale == original.targetModelScale, "fold must leave calibration, HMD and model scales untouched");
		}
	Check(protocol::Version == 8 && sizeof(protocol::DriftState) == 224 && sizeof(protocol::Response) == 232,
		"v8 drift snapshot and response layout must agree on the wire");
	std::printf("SPACE-PROTOCOL: Version=%u DriftState=%zu Response=%zu\n", protocol::Version, sizeof(protocol::DriftState), sizeof(protocol::Response));
}

static void Scenario23()
{
	const auto profile = Profile();
	auto state = DriftState(profile);
	const auto capture = spacememory::Capture(state, profile);
	Check(capture && capture->session == state.session && capture->basis.trackerSerial == profile.trackerSerial,
		"settled bound capture must retain session and tracker identity");
	const auto folded = spacememory::Fold(state);
	Check(capture->alignment.rotation == folded.rotation && capture->alignment.translation == folded.translation,
		"capture must use the folded alignment");
	for (const auto &change : std::vector<std::function<void(protocol::DriftState &)>>{
		[](auto &s) { s.valid = false; }, [](auto &s) { s.enabled = false; }, [](auto &s) { s.native = true; },
		[](auto &s) { s.updatesSinceChange = 119; }, [](auto &s) { s.session = 0; } })
	{
		auto invalid = state;
		change(invalid);
		Check(!spacememory::Capture(invalid, profile), "invalid, disabled, native, unsettled or sessionless drift must not capture");
	}
	for (const auto &change : std::vector<std::function<void(CalibrationContext &)>>{
		[](auto &p) { p.validProfile = false; }, [](auto &p) { p.trackerSerial.clear(); },
		[](auto &p) { p.validRelativeOffset = false; }, [](auto &p) { p.enableNative = true; } })
	{
		auto invalid = profile;
		change(invalid);
		Check(!spacememory::Capture(state, invalid), "unbound or native profile must not capture");
	}
	for (double invalid : { std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() })
	{
		for (size_t index = 0; index < DriftNumbers(state).size(); ++index)
		{
			auto changed = state;
			*DriftNumbers(changed)[index] = invalid;
			Check(!spacememory::Capture(changed, profile), "every drift and echo number must be finite before capture");
		}
		auto copy = profile;
		for (size_t index = 0; index < ProfileBasisNumbers(copy).size(); ++index)
		{
			auto changed = profile;
			*ProfileBasisNumbers(changed)[index] = invalid;
			Check(!spacememory::Capture(state, changed), "every profile basis number must be finite before capture");
		}
	}
	auto echoed = state;
	echoed.calibrationTranslation.v[0] += 0.25;
	echoed.calibrationScale = 1.03;
	echoed.hmdScale = 1.01;
	echoed.offsetTranslation.v[1] -= 0.02;
	const auto echoCapture = spacememory::Capture(echoed, profile);
	Check(echoCapture && echoCapture->basis.calibrationTranslation.v[0] == echoed.calibrationTranslation.v[0]
		&& echoCapture->basis.calibrationScale == echoed.calibrationScale && echoCapture->basis.hmdScale == echoed.hmdScale
		&& echoCapture->basis.offsetTranslation.v[1] == echoed.offsetTranslation.v[1]
		&& echoCapture->alignment.translation == spacememory::Fold(echoed).translation,
		"capture must use the driver echo rather than substituting the current profile");
}

static void Scenario24()
{
	using Result = spacememory::RestoreResult;
	const auto profile = Profile();
	const auto before = EncodeProfile(profile);
	const auto state = DriftState(profile);
	const auto stored = spacememory::Capture(state, profile);
	const auto restored = spacememory::Restore(stored, profile, state.session + 1);
	Check(restored.result == Result::Applied && restored.alignment.rotation == stored->alignment.rotation
		&& restored.alignment.translation == stored->alignment.translation, "new session with matching basis must restore the captured alignment");
	Check(spacememory::Restore(stored, profile, state.session).result == Result::SameSession && EncodeProfile(profile) == before,
		"same driver session must keep the profile unchanged");
	auto loaded = Profile();
	std::string error;
	Check(DecodeProfile(before, loaded, error) && spacememory::Restore(stored, loaded, state.session + 1).result == Result::Applied,
		"loaded profile basis must use the exact DesiredState conversion");
	auto copy = profile;
	for (size_t index = 0; index < ProfileBasisNumbers(copy).size(); ++index)
	{
		auto changed = profile;
		*ProfileBasisNumbers(changed)[index] += 0.1;
		Check(spacememory::Restore(stored, changed, state.session + 1).result == Result::BasisMismatch,
			"each calibration, offset and scale component must participate in the restore basis");
		Check(spacememory::Restore(stored, changed, state.session).result == Result::BasisMismatch,
			"stale basis must be discarded even in the captured session");
	}
	copy.trackerSerial += "-different";
	Check(spacememory::Restore(stored, copy, state.session + 1).result == Result::BasisMismatch, "tracker serial must participate in the restore basis");
	for (const auto &change : std::vector<std::function<void(CalibrationContext &)>>{
		[](auto &p) { p.validProfile = false; }, [](auto &p) { p.trackerSerial.clear(); },
		[](auto &p) { p.validRelativeOffset = false; }, [](auto &p) { p.enableNative = true; } })
	{
		copy = profile;
		change(copy);
		Check(spacememory::Restore(stored, copy, state.session + 1).result == Result::NotBound, "unbound or native profile cannot restore");
	}
	Check(spacememory::Restore({}, profile, state.session + 1).result == Result::Absent
		&& spacememory::Restore(spacememory::Decode("broken"), profile, state.session + 1).result == Result::Absent,
		"absent and malformed memory must restore as absent");
	copy = profile;
	copy.calibratedTranslation.x() += 0.00005;
	copy.relativeTranslation.v[1] += 0.0000005;
	copy.relativeRotation.x += 0.0000005;
	copy.calibratedRotation.x() += 0.00001;
	copy.calibratedScale += 0.0000005;
	copy.hmdScale += 0.0000005;
	Check(spacememory::Restore(stored, copy, state.session + 1).result == Result::Applied, "basis tolerance must allow sub-micrometer and rounding changes");
	copy = profile;
	copy.calibratedTranslation.x() += 0.0002;
	Check(spacememory::Restore(stored, copy, state.session + 1).result == Result::BasisMismatch, "basis tolerance must compare meters after centimeter conversion");
	copy = profile;
	copy.relativeRotation.w = -copy.relativeRotation.w;
	Check(spacememory::Restore(stored, copy, state.session + 1).result == Result::Applied, "equivalent quaternion signs must describe the same basis");
	copy = profile;
	copy.calibratedRotation.z() += 360;
	Check(spacememory::Restore(stored, copy, state.session + 1).result == Result::Applied, "equivalent Euler angles must compare as their DesiredState quaternion");
	copy = profile;
	copy.targetModelScale *= 1.01;
	copy.fallbackToSlam = !copy.fallbackToSlam;
	copy.headFilterEnabled = !copy.headFilterEnabled;
	Check(spacememory::Restore(stored, copy, state.session + 1).result == Result::Applied, "unrelated profile settings must not change the specified basis");
	for (const auto &[result, name] : std::vector<std::pair<Result, const char *>>{
		{ Result::Applied, "applied" }, { Result::SameSession, "same-session" }, { Result::BasisMismatch, "basis-mismatch" },
		{ Result::NotBound, "not-bound" }, { Result::Absent, "absent" } })
		Check(std::string(spacememory::ResultName(result)) == name, "restore result names must match the overlay log contract");
}

static void Scenario25()
{
	const auto profile = Profile();
	const auto stored = spacememory::Capture(DriftState(profile), profile);
	const auto encoded = spacememory::Encode(*stored);
	const auto decoded = spacememory::Decode(encoded);
	Check(decoded && decoded->session == stored->session && spacememory::Encode(*decoded) == encoded,
		"stored alignment, basis and all 64 session bits must round-trip losslessly");
	picojson::value value;
	Check(picojson::parse(value, encoded).empty(), "encoded memory must be JSON");
	auto object = value.get<picojson::object>();
	object["future"] = picojson::value(true);
	object["basis"].get<picojson::object>()["future"] = picojson::value(42.0);
	Check(spacememory::Decode(picojson::value(object).serialize())
		&& spacememory::Encode(*spacememory::Decode(picojson::value(object).serialize())) == encoded, "unknown fields at either level must be ignored");
	for (const auto &[key, unused] : value.get<picojson::object>())
	{
		auto missing = value.get<picojson::object>();
		missing.erase(key);
		Check(!spacememory::Decode(picojson::value(missing).serialize()), "every top-level memory field must be present");
		missing[key] = picojson::value(false);
		Check(!spacememory::Decode(picojson::value(missing).serialize()), "wrong top-level types must be absent");
	}
	for (const auto &[key, unused] : value.get("basis").get<picojson::object>())
	{
		auto missing = value.get<picojson::object>();
		missing["basis"].get<picojson::object>().erase(key);
		Check(!spacememory::Decode(picojson::value(missing).serialize()), "every basis field must be present");
		missing["basis"].get<picojson::object>()[key] = picojson::value(false);
		Check(!spacememory::Decode(picojson::value(missing).serialize()), "wrong basis field types must be absent");
	}
	for (const auto &invalid : std::vector<std::string>{ "", "[", "{}", "[]", "null", encoded + " trailing" })
		Check(!spacememory::Decode(invalid), "missing or malformed JSON must be absent");
	Check(spacememory::Decode(encoded + " \r\n\t").has_value(), "trailing JSON whitespace must remain valid");
	for (const char *session : { "", "0", "-1", "xyz", "10000000000000000", "123x", "0x123" })
	{
		auto malformed = value.get<picojson::object>();
		malformed["session"] = picojson::value(std::string(session));
		Check(!spacememory::Decode(picojson::value(malformed).serialize()), "malformed, overflowing or zero session identity must be absent");
	}
	auto emptySerial = value.get<picojson::object>();
	emptySerial["basis"].get<picojson::object>()["trackerSerial"] = picojson::value(std::string());
	Check(!spacememory::Decode(picojson::value(emptySerial).serialize()), "empty persisted tracker identity must be absent");
	for (const char *key : { "rotation", "translation", "calibrationRotation", "calibrationTranslation", "offsetRotation", "offsetTranslation" })
	{
		for (int change = 0; change < 3; ++change)
		{
			auto malformed = value.get<picojson::object>();
			auto &field = malformed.contains(key) ? malformed[key] : malformed["basis"].get<picojson::object>()[key];
			auto &numbers = field.get<picojson::array>();
			if (change == 0) numbers.pop_back();
			if (change == 1) numbers.emplace_back(0.0);
			if (change == 2) numbers[0] = picojson::value("NaN");
			Check(!spacememory::Decode(picojson::value(malformed).serialize()), "every stored vector must have exact finite numeric components");
		}
	}
	std::string overflow = encoded;
	const auto scale = overflow.find("\"calibrationScale\":");
	Check(scale != std::string::npos, "codec fixture must contain calibration scale");
	const auto start = scale + std::string("\"calibrationScale\":").size();
	overflow.replace(start, overflow.find(',', start) - start, "1e999");
	Check(!spacememory::Decode(overflow), "non-finite JSON numbers must be absent");
}

static void Scenario26()
{
	const auto stored = spacememory::Capture(DriftState(Profile()), Profile());
	Check(!spacememory::ShouldWrite({}, {}, 20, 0) && !spacememory::ShouldWrite({}, {}, 20, 0, true), "absent captures must never write");
	Check(!spacememory::ShouldWrite(stored, {}, 9.999, 0) && spacememory::ShouldWrite(stored, {}, 10, 0),
		"first capture must respect the ten-second cadence");
	Check(!spacememory::ShouldWrite(stored, stored, 100, 0) && !spacememory::ShouldWrite(stored, stored, 100, 0, true),
		"unchanged captures must not write periodically or at exit");
	auto changed = stored;
	changed->alignment.translation.x() += 0.001;
	Check(!spacememory::ShouldWrite(changed, stored, 19.999, 10) && spacememory::ShouldWrite(changed, stored, 20, 10),
		"changed capture must wait until exactly ten seconds after the last successful write");
	Check(spacememory::ShouldWrite(changed, stored, 10.001, 10, true) && spacememory::ShouldWrite(stored, {}, 0, 0, true),
		"exit must write the latest pending capture regardless of cadence");
	for (int field = 0; field < 3; ++field)
	{
		changed = stored;
		if (field == 0) changed->alignment.rotation.y() += 0.01;
		if (field == 1) changed->session += 1;
		if (field == 2) changed->basis.trackerSerial += "-new";
		Check(spacememory::ShouldWrite(changed, stored, 20, 10), "rotation, session and basis changes must count as pending writes");
	}
	std::optional<spacememory::StoredAlignment> written;
	Check(spacememory::ShouldWrite(stored, written, 20, 0) && spacememory::ShouldWrite(stored, written, 21, 0),
		"failed writes must stay pending and retry");
	written = stored;
	Check(!spacememory::ShouldWrite(stored, written, 40, 20), "successful writes must suppress unchanged retries");
}

static void Benchmark()
{
	Hypothesis hypothesis;
	hypothesis.system = "system-Y";
	hypothesis.pairs = { MakePair(0, 400), MakePair(1, 400) };
	for (size_t i = 0; i < 6; ++i)
		hypothesis.candidates.push_back({ { static_cast<uint32_t>(5 + i), "tracker" + std::to_string(i), "system-Y", 400 }, HeadSamples(i ? 0.27 : 0.08, i != 0, true, 400) });
	auto start = std::chrono::steady_clock::now();
	auto evaluation = Evaluate({ hypothesis });
	Check(evaluation.passing.size() == 1, "benchmark must exercise real acquisition evaluation");
	double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	std::printf("BENCH: 2 hand pairs, 6 candidates, 400 keyframes each: %.3f ms/evaluation, %.3f ms evaluation time per second at 1 Hz, worker thread in overlay\n", milliseconds, milliseconds);
}

int main(int argc, char **argv)
{
#ifdef _WIN32
	// A crash must end the run with its exit code instead of waiting behind a modal error box on the desktop.
	SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
	_set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
	const std::function<void()> scenarios[] = { Scenario1, Scenario2, Scenario3, Scenario4, Scenario5, Scenario6, Scenario7,
		Scenario8, Scenario9, Scenario10, Scenario11, Scenario12, Scenario13, Scenario14, Scenario15, Scenario16, Scenario17,
		Scenario18, Scenario19, Scenario20, Scenario21,
		Scenario22, Scenario23, Scenario24, Scenario25, Scenario26 };
	try
	{
		for (int i = 0; i < 26; ++i)
		{
			if (argc > 1 && std::atoi(argv[1]) != i + 1)
				continue;
			CurrentScenario = i + 1;
			scenarios[i]();
			std::printf("Scenario%d PASS\n", CurrentScenario);
		}
		if (argc == 1)
			Benchmark();
		std::puts("ALL REQUESTED SCENARIOS PASSED");
		return 0;
	}
	catch (const std::exception &error)
	{
		std::fprintf(stderr, "%s\n", error.what());
		return 1;
	}
}
