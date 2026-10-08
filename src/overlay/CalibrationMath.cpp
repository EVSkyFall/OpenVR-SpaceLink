// SPDX-License-Identifier: AGPL-3.0-only

#include "CalibrationMath.h"

#include <cstdio>

namespace acquisition
{

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

DSample DeltaRotationSamples(Sample s1, Sample s2)
{
	// Difference in rotation between samples.
	auto dref = s1.ref.rot * s2.ref.rot.transpose();
	auto dtarget = s1.target.rot * s2.target.rot.transpose();

	// When stuck together, the two tracked objects rotate as a pair,
	// therefore their axes of rotation must be equal between any given pair of samples.
	DSample ds;
	ds.ref = RotationVector(dref);
	ds.target = RotationVector(dtarget);

	// Reject samples that were too close to each other.
	auto refA = RotationAngle(dref);
	auto targetA = RotationAngle(dtarget);
	ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

	return ds;
}

Eigen::Vector3d CalibrateRotation(const std::vector<Sample>& samples, const std::function<void(const std::string &)> &log)
{
	std::vector<DSample> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	char buf[256];
	snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
	if (log) log(buf);
	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) = deltas[i].ref;
		targetPoints.row(i) = deltas[i].target;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
	{
		i(2, 2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

	snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	if (log) log(buf);
	return euler;
}

static const double ScaleSpreadThreshold = 0.1;
static const double MinCalibratedScale = 0.9;
static const double MaxCalibratedScale = 1.1;

Eigen::Vector3d CalibrateTranslation(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation, double scale, const std::function<void(const std::string &)> &log)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		Sample s_i = samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = scale * (rotation * s_i.target.trans);

		for (size_t j = 0; j < i; j++)
		{
			Sample s_j = samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = scale * (rotation * s_j.target.trans);

			auto QAi = s_i.ref.rot.transpose();
			auto QAj = s_j.ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CA, dQA));

			auto QBi = s_i.target.rot.transpose();
			auto QBj = s_j.target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CB, dQB));
		}
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		for (int axis = 0; axis < 3; axis++)
		{
			constants(i * 3 + axis) = deltas[i].first(axis);
			coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
		}
	}

	Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	auto transcm = trans * 100.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	if (log) log(buf);
	return transcm;
}

static double EstimateHmdSpaceScale(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation, double targetModelScale, const std::function<void(const std::string &)> &log)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for (auto &sample : samples)
		centroid += rotation * sample.target.trans;
	centroid /= (double)samples.size();

	double spread = 0;
	for (auto &sample : samples)
		spread += (rotation * sample.target.trans - centroid).squaredNorm();
	spread = std::sqrt(spread / (double)samples.size());

	char buf[256];
	if (spread < ScaleSpreadThreshold)
	{
		snprintf(buf, sizeof buf, "Not enough positional movement to estimate headset scale (spread %.2f m), assuming 1\n", spread);
		if (log) log(buf);
		return 1.0;
	}

	Eigen::MatrixXd coefficients(samples.size() * 3, 7);
	Eigen::VectorXd constants(samples.size() * 3);

	for (size_t i = 0; i < samples.size(); i++)
	{
		Eigen::Vector3d rotatedPos = rotation * samples[i].target.trans;
		Eigen::Matrix3d rotatedRot = rotation * samples[i].target.rot;

		coefficients.block<3, 1>(i * 3, 0) = rotatedPos;
		coefficients.block<3, 3>(i * 3, 1) = Eigen::Matrix3d::Identity();
		coefficients.block<3, 3>(i * 3, 4) = rotatedRot;
		constants.segment<3>(i * 3) = samples[i].ref.trans;
	}

	Eigen::VectorXd result = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	double fittedScale = result(0);

	if (fittedScale < MinCalibratedScale || fittedScale > MaxCalibratedScale)
	{
		snprintf(buf, sizeof buf, "Fitted space scale %.5f is not plausible, assuming headset scale 1\n", fittedScale);
		if (log) log(buf);
		return 1.0;
	}

	snprintf(buf, sizeof buf, "Fitted headset space scale relative to lighthouse: %.5f (%+.2f%%), implied absolute headset scale: %.5f\n",
		fittedScale, (fittedScale - 1.0) * 100.0, fittedScale / targetModelScale);
	if (log) log(buf);
	return fittedScale;
}

static Eigen::Vector3d ComputeRefToTargetOffset(const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
		accum += sample.ref.rot.transpose() * (calScale * (calRot * sample.target.trans) + calTrans - sample.ref.trans);

	return accum / (double)samples.size();
}

static double RetargetingErrorRMS(const std::vector<Sample> &samples, const Eigen::Vector3d &hmdToTargetPos, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	double accum = 0;

	for (auto &sample : samples)
		accum += (calScale * (calRot * sample.target.trans) + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

	return std::sqrt(accum / (double)samples.size());
}

// https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions/27410865
void ComputeRelativeOffset(CalibrationSolution &ctx, const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	if (samples.empty())
		return;

	Eigen::Matrix4d quatAccum = Eigen::Matrix4d::Zero();
	Eigen::Vector3d transAccum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
	{
		Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
		Eigen::Vector3d trackerTrans = calScale * (calRot * sample.target.trans) + calTrans;

		Eigen::Matrix3d offsetRot = trackerRot.transpose() * sample.ref.rot;
		Eigen::Vector3d offsetTrans = trackerRot.transpose() * (sample.ref.trans - trackerTrans);

		Eigen::Quaterniond q(offsetRot);
		Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
		quatAccum += v * v.transpose();
		transAccum += offsetTrans;
	}

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quatAccum);
	Eigen::Vector4d avg = solver.eigenvectors().col(3).normalized();

	Eigen::Quaterniond q(avg(0), avg(1), avg(2), avg(3));
	q.normalize();
	if (q.w() < 0)
		q.coeffs() = -q.coeffs();

	transAccum /= (double)samples.size();

	ctx.relativeRotation.w = q.w();
	ctx.relativeRotation.x = q.x();
	ctx.relativeRotation.y = q.y();
	ctx.relativeRotation.z = q.z();
	ctx.relativeTranslation.v[0] = transAccum.x();
	ctx.relativeTranslation.v[1] = transAccum.y();
	ctx.relativeTranslation.v[2] = transAccum.z();
}

bool AutomaticCalibrationSucceeded(const CalibrationSolution &solution)
{
	return solution.holdout.rms <= 0.10 && solution.holdout.p90 <= 0.15 && solution.rotationError.median <= 3 * Degrees;
}

bool AutomaticCalibrationAbandoned(size_t newerKeyframes, double rotationMedian)
{
	return newerKeyframes >= 20 && rotationMedian > 8 * Degrees;
}

CalibrationSolution SolveCalibration(const std::vector<Sample> &raw, double targetModelScale,
	const std::function<void(const std::string &)> &log)
{
	std::vector<Sample> training, holdout;
	for (size_t i = 0; i < raw.size(); ++i)
		(IsHoldout(i) ? holdout : training).push_back(raw[i]);
	CalibrationSolution solution;
	solution.targetModelScale = targetModelScale;
	solution.rotation = CalibrateRotation(training, log);
	Eigen::Vector3d radians = solution.rotation * EIGEN_PI / 180.0;
	Eigen::Matrix3d rotation = (Eigen::AngleAxisd(radians(0), Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(radians(1), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(radians(2), Eigen::Vector3d::UnitX())).toRotationMatrix();
	solution.hmdScale = EstimateHmdSpaceScale(training, rotation, targetModelScale, log);
	for (auto &sample : training)
		sample.ref.trans /= solution.hmdScale;
	for (auto &sample : holdout)
		sample.ref.trans /= solution.hmdScale;
	solution.translation = CalibrateTranslation(training, rotation, solution.scale, log);
	Eigen::Vector3d translation = solution.translation * 0.01;
	Eigen::Vector3d offset = ComputeRefToTargetOffset(training, rotation, translation, solution.scale);
	std::vector<double> errors;
	for (const auto &sample : holdout)
		errors.push_back((solution.scale * (rotation * sample.target.trans) + translation
			- (sample.ref.rot * offset + sample.ref.trans)).norm());
	solution.holdout = Statistics(std::move(errors));
	solution.holdout.rms = RetargetingErrorRMS(holdout, offset, rotation, translation, solution.scale);
	solution.rotationError = RotationConsistency(holdout, rotation);
	ComputeRelativeOffset(solution, training, rotation, translation, solution.scale);
	return solution;
}

}
