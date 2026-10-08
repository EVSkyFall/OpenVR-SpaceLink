// SPDX-License-Identifier: AGPL-3.0-only

#include "AcquisitionMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string_view>

namespace acquisition
{

Pose::Pose(const vr::HmdMatrix34_t &matrix)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			rot(i, j) = matrix.m[i][j];
		trans(i) = matrix.m[i][3];
	}
}

double RotationAngle(const Eigen::Matrix3d &rotation)
{
	return std::acos(std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0));
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d &rotation)
{
	Eigen::AngleAxisd angle(rotation);
	return angle.angle() * angle.axis();
}

Eigen::Matrix3d MeanRotation(const std::vector<Eigen::Matrix3d> &rotations)
{
	if (rotations.empty())
		return Eigen::Matrix3d::Identity();
	Eigen::Matrix4d accum = Eigen::Matrix4d::Zero();
	for (const auto &rotation : rotations)
	{
		Eigen::Vector4d q = Eigen::Quaterniond(rotation).coeffs();
		accum += q * q.transpose();
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(accum);
	Eigen::Quaterniond mean;
	mean.coeffs() = solver.eigenvectors().col(3);
	return mean.normalized().toRotationMatrix();
}

double RotationInformation(const std::vector<Eigen::Matrix3d> &rotations)
{
	if (rotations.empty())
		return 0;
	Eigen::Matrix3d mean = MeanRotation(rotations);
	std::vector<Eigen::Vector3d> vectors;
	Eigen::Vector3d center = Eigen::Vector3d::Zero();
	for (const auto &rotation : rotations)
	{
		vectors.push_back(RotationVector(rotation * mean.transpose()));
		center += vectors.back();
	}
	center /= static_cast<double>(vectors.size());
	Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
	for (const auto &vector : vectors)
	{
		Eigen::Vector3d delta = vector - center;
		covariance += delta * delta.transpose();
	}
	covariance /= static_cast<double>(vectors.size());
	return Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(covariance).eigenvalues()(1);
}

ErrorStats Statistics(std::vector<double> values)
{
	if (values.empty())
		return { INFINITY, INFINITY, INFINITY };
	std::sort(values.begin(), values.end());
	double squares = 0;
	for (double value : values)
		squares += value * value;
	return { values[values.size() / 2], values[static_cast<size_t>(std::ceil(0.9 * values.size())) - 1],
		std::sqrt(squares / static_cast<double>(values.size())) };
}

bool IsHoldout(size_t index)
{
	return index % 5 == 0;
}

static Eigen::Matrix3d Yaw(double yaw)
{
	return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

double DampedYawStep(double numerator, double information)
{
	return std::clamp(numerator / (information + 1e-6), -0.2, 0.2);
}

struct FitRow
{
	const Sample *sample;
	size_t pair;
};

static double InitialYaw(const std::vector<HandPair> &pairs)
{
	double dot = 0, cross = 0;
	for (const auto &pair : pairs)
	{
		Eigen::Vector3d s = Eigen::Vector3d::Zero(), l = s;
		size_t count = 0;
		for (size_t k = 0; k < pair.samples.size(); ++k)
			if (!IsHoldout(k))
			{
				s += pair.samples[k].ref.trans;
				l += pair.samples[k].target.trans;
				++count;
			}
		if (!count)
			continue;
		s /= static_cast<double>(count);
		l /= static_cast<double>(count);
		for (size_t k = 0; k < pair.samples.size(); ++k)
			if (!IsHoldout(k))
			{
				Eigen::Vector3d a = pair.samples[k].target.trans - l;
				Eigen::Vector3d b = pair.samples[k].ref.trans - s;
				dot += a.x() * b.x() + a.z() * b.z();
				cross += a.z() * b.x() - a.x() * b.z();
			}
	}
	return std::atan2(cross, dot);
}

static Eigen::VectorXd FitLinear(const std::vector<FitRow> &rows, size_t pairs, double yaw,
	const std::vector<double> &weights, Eigen::MatrixXd *design = nullptr)
{
	const Eigen::Index count = static_cast<Eigen::Index>(rows.size());
	Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(count * 3 + 3 * pairs, 3 + 3 * pairs);
	Eigen::VectorXd constants = Eigen::VectorXd::Zero(count * 3 + 3 * pairs);
	Eigen::Matrix3d rotation = Yaw(yaw);
	for (Eigen::Index i = 0; i < count; ++i)
	{
		const auto &row = rows[static_cast<size_t>(i)];
		double weight = std::sqrt(weights[static_cast<size_t>(i)]);
		coefficients.block<3, 3>(i * 3, 0) = weight * Eigen::Matrix3d::Identity();
		coefficients.block<3, 3>(i * 3, 3 + 3 * row.pair) = weight * rotation * row.sample->target.rot;
		constants.segment<3>(i * 3) = weight * (row.sample->ref.trans - rotation * row.sample->target.trans);
	}
	for (size_t p = 0; p < pairs; ++p)
		coefficients.block<3, 3>(count * 3 + 3 * p, 3 + 3 * p) = (0.03 / 0.10) * Eigen::Matrix3d::Identity();
	Eigen::VectorXd result = coefficients.completeOrthogonalDecomposition().solve(constants);
	if (design)
		*design = std::move(coefficients);
	return result;
}

SyncResult FitHandPairs(const std::vector<HandPair> &pairs)
{
	SyncResult result;
	if (pairs.empty())
		return result;
	std::vector<FitRow> rows;
	bool enough = true;
	for (size_t p = 0; p < pairs.size(); ++p)
	{
		const auto &samples = pairs[p].samples;
		if (samples.empty())
			return result;
		Eigen::Vector3d center = Eigen::Vector3d::Zero();
		for (const auto &sample : samples)
		{
			center += sample.target.trans;
		}
		center /= static_cast<double>(samples.size());
		double spread = 0;
		for (size_t k = 0; k < samples.size(); ++k)
		{
			Eigen::Vector3d delta = samples[k].target.trans - center;
			spread += delta.x() * delta.x() + delta.z() * delta.z();
			if (!IsHoldout(k))
				rows.push_back({ &samples[k], p });
		}
		enough = enough && samples.size() >= 12 && std::sqrt(spread / samples.size()) >= 0.10;
	}
	if (rows.empty())
		return result;
	double yaw = InitialYaw(pairs);
	std::vector<double> weights(rows.size(), 1.0);
	Eigen::VectorXd fit;
	for (int iteration = 0; iteration < 12; ++iteration)
	{
		Eigen::MatrixXd design;
		fit = FitLinear(rows, pairs.size(), yaw, weights, &design);
		Eigen::Matrix3d rotation = Yaw(yaw);
		Eigen::VectorXd derivative = Eigen::VectorXd::Zero(design.rows());
		Eigen::VectorXd residual = Eigen::VectorXd::Zero(design.rows());
		for (size_t i = 0; i < rows.size(); ++i)
		{
			const auto &row = rows[i];
			Eigen::Vector3d point = rotation * (row.sample->target.trans + row.sample->target.rot * fit.segment<3>(3 + 3 * row.pair));
			Eigen::Vector3d error = row.sample->ref.trans - point - fit.head<3>();
			double weight = std::sqrt(weights[i]);
			derivative.segment<3>(3 * i) = weight * Eigen::Vector3d::UnitY().cross(point);
			residual.segment<3>(3 * i) = weight * error;
			weights[i] = 0.03 / std::max(0.03, error.norm());
		}
		for (size_t p = 0; p < pairs.size(); ++p)
			residual.segment<3>(rows.size() * 3 + 3 * p) = -(0.03 / 0.10) * fit.segment<3>(3 + 3 * p);
		// Variable projection removes the part of a yaw change that the linear offsets can explain.
		Eigen::VectorXd projected = derivative - design * design.completeOrthogonalDecomposition().solve(derivative);
		yaw += DampedYawStep(projected.dot(residual), projected.squaredNorm());
	}
	Eigen::MatrixXd design;
	fit = FitLinear(rows, pairs.size(), yaw, weights, &design);
	design = design.topRows(static_cast<Eigen::Index>(rows.size() * 3)).eval();
	Eigen::MatrixXd nuisance(design.rows(), design.cols() - 1);
	nuisance.col(0) = design.col(0);
	nuisance.rightCols(design.cols() - 2) = design.rightCols(design.cols() - 2);
	// Project out every other unknown, including horizontal translation. A tilted single axis can also hide height.
	Eigen::VectorXd vertical = design.col(1);
	Eigen::VectorXd heightResidual = vertical - nuisance * nuisance.completeOrthogonalDecomposition().solve(vertical);
	result.heightCertain = heightResidual.squaredNorm() / vertical.squaredNorm() >= 0.01;
	result.rotation = Yaw(yaw);
	result.translation = fit.head<3>();
	std::vector<double> errors;
	for (size_t p = 0; p < pairs.size(); ++p)
	{
		result.offsets.push_back(fit.segment<3>(3 + 3 * p));
		for (size_t k = 0; k < pairs[p].samples.size(); ++k)
			if (IsHoldout(k))
			{
				const auto &sample = pairs[p].samples[k];
				errors.push_back((sample.ref.trans - result.rotation * (sample.target.trans + sample.target.rot * result.offsets.back()) - result.translation).norm());
			}
	}
	result.error = Statistics(std::move(errors));
	result.held = enough && result.error.median <= 0.08 && result.error.p90 <= 0.15;
	result.handPairs = result.held ? static_cast<int>(pairs.size()) : 0;
	return result;
}

SyncResult CombineHandPairs(const std::vector<HandPair> &pairs, std::vector<SyncResult> *individual)
{
	std::vector<SyncResult> fits;
	for (const auto &pair : pairs)
		fits.push_back(FitHandPairs({ pair }));
	if (individual)
		*individual = fits;
	SyncResult best;
	for (size_t seed = 0; seed < pairs.size(); ++seed)
	{
		if (!fits[seed].held)
			continue;
		std::vector<size_t> members{ seed };
		std::set<uint32_t> used{ pairs[seed].hand, pairs[seed].controller };
		for (size_t p = 0; p < pairs.size(); ++p)
		{
			if (!fits[p].held || used.count(pairs[p].hand) || used.count(pairs[p].controller))
				continue;
			bool agrees = true;
			for (size_t other : members)
				agrees = agrees && RotationAngle(fits[p].rotation * fits[other].rotation.transpose()) <= 3 * Degrees
					&& (fits[p].translation - fits[other].translation).norm() <= 0.05;
			if (agrees)
			{
				members.push_back(p);
				used.insert(pairs[p].hand);
				used.insert(pairs[p].controller);
			}
		}
		std::vector<HandPair> joint;
		for (size_t member : members)
			joint.push_back(pairs[member]);
		SyncResult fit = members.size() == 1 ? fits[seed] : FitHandPairs(joint);
		if (fit.held && (!best.held || fit.handPairs > best.handPairs || (fit.handPairs == best.handPairs && fit.error.median < best.error.median)))
			best = std::move(fit);
	}
	return best;
}

ErrorStats RotationConsistency(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation)
{
	std::vector<Eigen::Matrix3d> offsets;
	for (const auto &sample : samples)
		offsets.push_back((rotation * sample.target.rot).transpose() * sample.ref.rot);
	Eigen::Matrix3d mean = MeanRotation(offsets);
	std::vector<double> errors;
	for (const auto &offset : offsets)
		errors.push_back(RotationAngle(offset * mean.transpose()));
	return Statistics(std::move(errors));
}

RigidityResult FitRigidity(const std::vector<Sample> &samples, const Eigen::Matrix3d &preferredRotation)
{
	RigidityResult result;
	result.error = Statistics({});
	if (samples.size() < 8)
		return result;
	Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
	size_t deltas = 0;
	for (size_t i = 0; i < samples.size(); ++i)
		for (size_t j = 0; j < i; ++j)
		{
			Eigen::Vector3d h = RotationVector(samples[i].ref.rot * samples[j].ref.rot.transpose());
			Eigen::Vector3d c = RotationVector(samples[i].target.rot * samples[j].target.rot.transpose());
			if (h.norm() >= 0.15 && c.norm() >= 0.15)
			{
				covariance += h * c.transpose();
				++deltas;
			}
		}
	result.deltas = deltas;
	if (deltas < 10)
		return result;
	// A single rotation axis leaves a free twist; use the hand sync to resolve only that ambiguity.
	covariance += 1e-9 * static_cast<double>(deltas) * preferredRotation;
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
	sign(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0 ? -1 : 1;
	result.rotation = svd.matrixU() * sign * svd.matrixV().transpose();
	result.error = RotationConsistency(samples, result.rotation);
	result.observable = true;
	return result;
}

CandidateCheck CheckHeadTracker(const std::vector<Sample> &samples, const SyncResult &sync)
{
	CandidateCheck check;
	check.rigidity = FitRigidity(samples, sync.rotation);
	check.agreement = RotationAngle(check.rigidity.rotation * sync.rotation.transpose());
	std::vector<Eigen::Vector3d> offsets;
	std::vector<double> distances, heights;
	for (const auto &sample : samples)
	{
		Eigen::Vector3d delta = sync.rotation * sample.target.trans + sync.translation - sample.ref.trans;
		distances.push_back(delta.norm());
		offsets.push_back(sample.ref.rot.transpose() * delta);
		heights.push_back(offsets.back().y());
	}
	Eigen::Vector3d center;
	for (int axis = 0; axis < 3; ++axis)
	{
		std::vector<double> components;
		for (const auto &offset : offsets)
			components.push_back(offset(axis));
		center(axis) = Statistics(std::move(components)).median;
	}
	std::vector<double> deviations;
	for (const auto &offset : offsets)
		deviations.push_back((offset - center).norm());
	check.distance = Statistics(std::move(distances));
	check.vertical = Statistics(std::move(heights));
	check.spread = Statistics(std::move(deviations));
	if (!sync.held) check.failure = "hand-sync";
	else if (samples.size() < 8) check.failure = "keyframes";
	else if (!check.rigidity.observable) check.failure = "rotation-deltas";
	else if (check.rigidity.error.median > 5 * Degrees) check.failure = "rotation-median";
	else if (check.rigidity.error.p90 > 12 * Degrees) check.failure = "rotation-p90";
	else if (check.agreement > 15 * Degrees) check.failure = "sync-rotation";
	else if (check.distance.median > 0.35) check.failure = "distance";
	else if (check.spread.p90 > 0.15) check.failure = "offset-spread";
	else if (check.vertical.median < -0.18) check.failure = "vertical";
	return check;
}

bool IsHeadTracker(const std::vector<Sample> &samples, const SyncResult &sync, RigidityResult *rigidity)
{
	auto check = CheckHeadTracker(samples, sync);
	if (rigidity)
		*rigidity = check.rigidity;
	return std::string_view(check.failure) == "pass";
}

}
