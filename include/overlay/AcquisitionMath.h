// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Dense>
#include <openvr.h>
#include <cstdint>
#include <vector>

namespace acquisition
{

constexpr double Pi = 3.14159265358979323846;
constexpr double Degrees = Pi / 180.0;

struct Pose
{
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d trans = Eigen::Vector3d::Zero();

	Pose() = default;
	explicit Pose(const vr::HmdMatrix34_t &matrix);
	bool Finite() const { return rot.allFinite() && trans.allFinite(); }
};

struct Sample
{
	Pose ref, target;
	uint64_t sequence = 0;
	double time = 0;
};

struct ErrorStats
{
	double median = 0, p90 = 0, rms = 0;
};

double RotationAngle(const Eigen::Matrix3d &rotation);
Eigen::Vector3d RotationVector(const Eigen::Matrix3d &rotation);
Eigen::Matrix3d MeanRotation(const std::vector<Eigen::Matrix3d> &rotations);
double RotationInformation(const std::vector<Eigen::Matrix3d> &rotations);
ErrorStats Statistics(std::vector<double> values);
bool IsHoldout(size_t index);

struct HandPair
{
	uint32_t hand = 0, controller = 0;
	std::vector<Sample> samples;
};

struct SyncResult
{
	bool held = false, heightCertain = false;
	Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	Eigen::Vector3d translation = Eigen::Vector3d::Zero();
	std::vector<Eigen::Vector3d> offsets;
	ErrorStats error;
	int handPairs = 0;
};

SyncResult FitHandPairs(const std::vector<HandPair> &pairs);
SyncResult CombineHandPairs(const std::vector<HandPair> &pairs);

struct RigidityResult
{
	bool observable = false;
	Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	ErrorStats error;
};

ErrorStats RotationConsistency(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation);
RigidityResult FitRigidity(const std::vector<Sample> &samples);
bool IsHeadTracker(const std::vector<Sample> &samples, const SyncResult &sync, RigidityResult *rigidity = nullptr);

}
