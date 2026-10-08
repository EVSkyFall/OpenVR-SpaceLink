// SPDX-License-Identifier: AGPL-3.0-only

#include "AttemptLifecycle.h"
#include "AutoAcquisition.h"

CalibrationAttempt BeginAutomaticAttempt(const acquisition::CandidateEvidence &candidate, size_t capacity)
{
	CalibrationAttempt attempt;
	attempt.automatic = true;
	attempt.serial = candidate.serial;
	attempt.trackingSystem = candidate.system;
	attempt.id = candidate.id;
	attempt.confirmationRotation = candidate.rotation;
	attempt.sampler.Seed(candidate.samples, capacity);
	return attempt;
}

bool CalibrationReadyToSolve(const CalibrationAttempt &attempt, size_t required)
{
	const auto &store = attempt.sampler.Store();
	if (store.Samples().size() < required || store.Count() == attempt.lastSolveCount)
		return false;
	std::vector<Eigen::Matrix3d> rotations;
	for (const auto &sample : store.Samples())
		rotations.push_back(sample.target.rot);
	return acquisition::RotationInformation(rotations) >= 0.10 * 0.10;
}

void ClearCommittedProfile(CalibrationContext &ctx)
{
	ctx.chaperone.geometry.clear();
	ctx.chaperone.standingCenter = {};
	ctx.chaperone.playSpaceSize = {};
	ctx.chaperone.valid = false;
	ctx.calibratedRotation.setZero();
	ctx.calibratedTranslation.setZero();
	ctx.calibratedScale = ctx.targetModelScale = ctx.hmdScale = 1.0;
	ctx.relativeRotation = { 1, 0, 0, 0 };
	ctx.relativeTranslation = {};
	ctx.validRelativeOffset = false;
	ctx.targetTrackingSystem.clear();
	ctx.hmdSerial.clear();
	ctx.trackerSerial.clear();
	ctx.targetID = vr::k_unTrackedDeviceIndexInvalid;
	ctx.enabled = ctx.validProfile = false;
}

void CommitAttempt(CalibrationContext &ctx, const CalibrationAttempt &attempt)
{
	ctx.trackerSerial = attempt.serial;
	ctx.targetID = attempt.id;
	ctx.targetTrackingSystem = attempt.trackingSystem;
	ctx.hmdSerial = attempt.hmdSerial;
	ctx.calibratedRotation = attempt.result.rotation;
	ctx.calibratedTranslation = attempt.result.translation;
	ctx.calibratedScale = attempt.result.scale;
	ctx.targetModelScale = attempt.result.targetModelScale;
	ctx.hmdScale = attempt.result.hmdScale;
	ctx.relativeRotation = attempt.result.relativeRotation;
	ctx.relativeTranslation = attempt.result.relativeTranslation;
	ctx.validRelativeOffset = ctx.validProfile = true;
}

void DiscardAttempt(CalibrationContext &ctx, std::optional<CalibrationAttempt> &attempt)
{
	attempt.reset();
	ctx.state = CalibrationState::None;
	ctx.statusLine.clear();
}

void RemoveCalibrationState(CalibrationContext &ctx, std::optional<CalibrationAttempt> &attempt, acquisition::AutoAcquisition &acquisition)
{
	ClearCommittedProfile(ctx);
	DiscardAttempt(ctx, attempt);
	acquisition.Clear();
}

bool ApplyAutoAcquireState(CalibrationContext &ctx, std::optional<CalibrationAttempt> &attempt, acquisition::AutoAcquisition &acquisition, bool enabled)
{
	ctx.autoAcquire = enabled;
	acquisition.Clear();
	if (!enabled && attempt && attempt->automatic)
	{
		DiscardAttempt(ctx, attempt);
		return true;
	}
	return false;
}

bool AutoAcquisitionEligible(const CalibrationContext &ctx, bool profileReadSucceeded, bool settingsReadSucceeded)
{
	return ctx.trackerSerial.empty() && ctx.state == CalibrationState::None && ctx.autoAcquire && profileReadSucceeded && settingsReadSucceeded;
}

size_t CalibrationCapacity(const CalibrationContext &committed)
{
	return 2 * committed.SampleCount();
}

AcquireState DescribeAcquisition(const CalibrationContext &ctx, const CalibrationAttempt *attempt,
	bool profileReadSucceeded, bool settingsReadSucceeded, bool handsAvailable, int handPairs)
{
	if (!ctx.trackerSerial.empty()) return AcquireState::Bound;
	if ((ctx.state != CalibrationState::None || attempt) && !(attempt && attempt->automatic)) return AcquireState::Paused;
	if (!ctx.autoAcquire) return AcquireState::Off;
	if (!profileReadSucceeded || !settingsReadSucceeded) return AcquireState::ProfileUnreadable;
	if (attempt) return AcquireState::Calibrating;
	if (!handsAvailable) return AcquireState::NeedHands;
	return handPairs ? AcquireState::Searching : AcquireState::Syncing;
}

std::string WaitingForTracker(const std::string &serial)
{
	return "Waiting for head tracker " + serial + ".";
}

CalibrationNotice FoundNotice(const std::string &serial)
{
	return { "Head tracker found: " + serial + ". Look around naturally for a few seconds to finish.", vr::EVRNotificationType_Persistent };
}

CalibrationNotice RecalibrationNotice()
{
	return { "Head tracker moved on the headset. Look around naturally for a few seconds to recalibrate.", vr::EVRNotificationType_Persistent };
}

const char *CalibrationAttemptMode(const CalibrationAttempt &attempt)
{
	return attempt.tiltRecalibration ? "recalibrate" : attempt.automatic ? "automatic" : "manual";
}

CalibrationNotice ReadyNotice(const CalibrationContext &committed)
{
	return { "Head tracker ready: " + committed.trackerSerial + ".", vr::EVRNotificationType_Transient };
}
