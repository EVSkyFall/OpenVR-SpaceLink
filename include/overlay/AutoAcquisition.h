// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "DesiredState.h"
#include "SamplerPolicy.h"

#include <array>
#include <optional>

namespace acquisition
{

struct CandidateEvidence
{
	uint32_t id = 0;
	std::string serial, system;
	uint64_t count = 0;
	Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	bool heightCertain = false;
	std::vector<Sample> samples;
};

struct CandidateFrames
{
	CandidateEvidence candidate;
	std::vector<Sample> samples;
};

struct Hypothesis
{
	std::string system;
	std::vector<HandPair> pairs;
	std::vector<CandidateFrames> candidates;
};

struct PairDiagnostic
{
	std::string hand, controller;
	size_t keyframes = 0;
	SyncResult fit;
};

struct CandidateDiagnostic
{
	std::string serial;
	size_t keyframes = 0;
	CandidateCheck check;
};

struct Evaluation
{
	int handPairs = 0;
	std::vector<CandidateEvidence> passing;
	bool diagnostics = false;
	std::vector<PairDiagnostic> pairs;
	std::vector<CandidateDiagnostic> candidates;
};

Evaluation Evaluate(const std::vector<Hypothesis> &hypotheses, bool diagnostics = false);

class Confirmation
{
public:
	std::optional<CandidateEvidence> Update(const Evaluation &evaluation);
	void Reset() { previous.reset(); }

private:
	std::optional<CandidateEvidence> previous;
};

class AutoAcquisition
{
public:
	void Refresh(const std::vector<DeviceSnapshot> &devices);
	bool ObserveConfigured(double time, const vr::TrackedDevicePose_t *observed, const ObservationSpace &space);
	void Observe(double time, const vr::TrackedDevicePose_t *poses);
	std::vector<Hypothesis> Snapshot() const;
	void Clear();
	void ResetConfirmations() { confirmation.Reset(); }
	std::optional<CandidateEvidence> Confirm(const Evaluation &evaluation, uint64_t evaluatedEpoch) {
		return evaluatedEpoch == epoch ? confirmation.Update(evaluation) : std::nullopt;
	}
	bool HasHands() const { return !pairs.empty(); }
	uint64_t Epoch() const { return epoch; }

private:
	struct PairStore
	{
		std::string system, handSerial, controllerSerial;
		uint32_t hand = 0, controller = 0;
		Sampler sampler;
	};
	struct CandidateStore
	{
		CandidateEvidence candidate;
		Sampler sampler;
	};
	std::vector<PairStore> pairs;
	std::vector<CandidateStore> candidates;
	Confirmation confirmation;
	uint64_t epoch = 0;
	bool headsetLost = false;
};

}
