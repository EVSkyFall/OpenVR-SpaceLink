// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "CalibrationMath.h"
#include "SamplerPolicy.h"

#include <optional>

namespace acquisition { class AutoAcquisition; struct CandidateEvidence; }

struct CalibrationAttempt
{
	std::string serial, trackingSystem, hmdSerial, statusLine;
	uint32_t id = vr::k_unTrackedDeviceIndexInvalid;
	bool automatic = false;
	bool tiltRecalibration = false;
	bool paused = false;
	acquisition::Sampler sampler, rigiditySampler;
	Eigen::Matrix3d confirmationRotation = Eigen::Matrix3d::Identity();
	acquisition::CalibrationSolution result;
	double modelScale = 1.0;
	uint64_t lastSolveCount = 0;
};

CalibrationAttempt BeginAutomaticAttempt(const acquisition::CandidateEvidence &candidate, size_t capacity);
bool CalibrationReadyToSolve(const CalibrationAttempt &attempt, size_t required);

void ClearCommittedProfile(CalibrationContext &committed);
void CommitAttempt(CalibrationContext &committed, const CalibrationAttempt &attempt);
void DiscardAttempt(CalibrationContext &committed, std::optional<CalibrationAttempt> &attempt);
void RemoveCalibrationState(CalibrationContext &committed, std::optional<CalibrationAttempt> &attempt, acquisition::AutoAcquisition &acquisition);
bool ApplyAutoAcquireState(CalibrationContext &committed, std::optional<CalibrationAttempt> &attempt, acquisition::AutoAcquisition &acquisition, bool enabled);
bool AutoAcquisitionEligible(const CalibrationContext &committed, bool profileReadSucceeded, bool settingsReadSucceeded);
size_t CalibrationCapacity(const CalibrationContext &committed);
AcquireState DescribeAcquisition(const CalibrationContext &committed, const CalibrationAttempt *attempt,
	bool profileReadSucceeded, bool settingsReadSucceeded, bool handsAvailable, int handPairs);
std::string WaitingForTracker(const std::string &serial);

struct CalibrationNotice
{
	std::string text;
	vr::EVRNotificationType type;
};
CalibrationNotice FoundNotice(const std::string &serial);
CalibrationNotice RecalibrationNotice();
const char *CalibrationAttemptMode(const CalibrationAttempt &attempt);
CalibrationNotice ReadyNotice(const CalibrationContext &committed);

struct PersistenceState
{
	bool resolved = false, dirty = false;
	uint64_t revision = 0;
	void Changed() { ++revision; resolved = true; dirty = true; }
	bool AcceptRead(uint64_t startedAt) const { return !resolved && revision == startedAt; }
	void Saved(bool success) { if (success) dirty = false; }
};
