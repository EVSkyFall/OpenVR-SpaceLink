// SPDX-License-Identifier: AGPL-3.0-only

#include "SpaceMemory.h"
#include "DesiredState.h"

#include <Geometry>
#include <picojson.h>
#include <charconv>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace spacememory
{

static Eigen::Quaterniond Quaternion(const vr::HmdQuaternion_t &q) { return { q.w, q.x, q.y, q.z }; }
static Eigen::Vector3d Vector(const vr::HmdVector3d_t &v) { return { v.v[0], v.v[1], v.v[2] }; }

static bool Bound(const CalibrationContext &profile)
{
	return profile.validProfile && !profile.trackerSerial.empty() && profile.validRelativeOffset && !profile.enableNative;
}

static Basis ProfileBasis(const CalibrationContext &profile)
{
	const auto hmd = DesiredState({}, profile, false).front().setHmdTracker;
	return { hmd.calibrationRotation, hmd.offsetRotation, hmd.calibrationTranslation, hmd.offsetTranslation,
		hmd.calibrationScale, hmd.hmdScale, profile.trackerSerial };
}

static bool Finite(const Basis &basis)
{
	return Quaternion(basis.calibrationRotation).coeffs().allFinite() && Vector(basis.calibrationTranslation).allFinite()
		&& Quaternion(basis.offsetRotation).coeffs().allFinite() && Vector(basis.offsetTranslation).allFinite()
		&& std::isfinite(basis.calibrationScale) && std::isfinite(basis.hmdScale);
}

static bool Finite(const StoredAlignment &stored)
{
	return stored.alignment.rotation.allFinite() && stored.alignment.translation.allFinite() && Finite(stored.basis);
}

static bool Close(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b)
{
	const auto qa = Quaternion(a).coeffs().eval(), qb = Quaternion(b).coeffs().eval();
	return (qa - qb).cwiseAbs().maxCoeff() <= 1e-6 || (qa + qb).cwiseAbs().maxCoeff() <= 1e-6;
}

static bool Close(const vr::HmdVector3d_t &a, const vr::HmdVector3d_t &b)
{
	return (Vector(a) - Vector(b)).cwiseAbs().maxCoeff() <= 1e-6;
}

static bool Close(const Basis &a, const Basis &b)
{
	return a.trackerSerial == b.trackerSerial
		&& Close(a.calibrationRotation, b.calibrationRotation) && Close(a.calibrationTranslation, b.calibrationTranslation)
		&& Close(a.offsetRotation, b.offsetRotation) && Close(a.offsetTranslation, b.offsetTranslation)
		&& std::abs(a.calibrationScale - b.calibrationScale) <= 1e-6 && std::abs(a.hmdScale - b.hmdScale) <= 1e-6;
}

static bool Same(const Basis &a, const Basis &b)
{
	return a.trackerSerial == b.trackerSerial
		&& Quaternion(a.calibrationRotation).coeffs() == Quaternion(b.calibrationRotation).coeffs()
		&& Vector(a.calibrationTranslation) == Vector(b.calibrationTranslation)
		&& Quaternion(a.offsetRotation).coeffs() == Quaternion(b.offsetRotation).coeffs()
		&& Vector(a.offsetTranslation) == Vector(b.offsetTranslation)
		&& a.calibrationScale == b.calibrationScale && a.hmdScale == b.hmdScale;
}

bool TiltRecalibration::Update(const protocol::DriftState &state, const CalibrationContext &profile, bool attempting)
{
	const auto basis = ProfileBasis(profile);
	if (cancelledBasis && (cancelledSession != state.session || !Same(*cancelledBasis, basis)))
		cancelledBasis.reset();
	const bool mismatch = !cancelledBasis && !attempting && state.enabled && profile.validProfile
		&& profile.validRelativeOffset && !profile.trackerSerial.empty() && state.tiltSamples >= 30 && state.tiltMismatchDeg >= 10.0;
	const bool trigger = mismatch && previousMismatch && previousSession == state.session && previousBasis && Same(*previousBasis, basis);
	previousMismatch = mismatch;
	previousSession = state.session;
	previousBasis = basis;
	return trigger;
}

void TiltRecalibration::Cancel(const CalibrationContext &profile, uint64_t session)
{
	cancelledBasis = ProfileBasis(profile);
	cancelledSession = session;
	ResetPolls();
}

Alignment Fold(const protocol::DriftState &state)
{
	const Eigen::Quaterniond inverse = Quaternion(state.rotation).inverse();
	const Eigen::Quaterniond rotation = inverse * Quaternion(state.calibrationRotation);
	Eigen::Vector3d angles = rotation.toRotationMatrix().eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	Eigen::Vector3d alternate(angles.x() + 180.0, 180.0 - angles.y(), angles.z() + 180.0);
	auto wrap = [](double angle) {
		double wrapped = std::remainder(angle, 360.0);
		return wrapped == -180.0 ? 180.0 : wrapped;
	};
	angles = angles.unaryExpr(wrap).eval();
	alternate = alternate.unaryExpr(wrap).eval();
	if (std::abs(alternate.x()) + std::abs(alternate.z()) < std::abs(angles.x()) + std::abs(angles.z()))
		angles = alternate;
	return { angles,
		(inverse * (Vector(state.calibrationTranslation) - Vector(state.translation))) * 100.0 };
}

std::optional<StoredAlignment> Capture(const protocol::DriftState &state, const CalibrationContext &profile)
{
	if (!state.valid || !state.enabled || state.native || state.updatesSinceChange < 120 || !Bound(profile) || !state.session)
		return std::nullopt;
	if (state.tiltSamples < 30 || !(state.tiltMismatchDeg < 5.0))
		return std::nullopt;
	Basis basis{ state.calibrationRotation, state.offsetRotation, state.calibrationTranslation, state.offsetTranslation,
		state.calibrationScale, state.hmdScale, profile.trackerSerial };
	const auto expected = ProfileBasis(profile);
	if (!Finite(basis) || !Quaternion(state.rotation).coeffs().allFinite() || !Vector(state.translation).allFinite()
		|| !std::isfinite(state.slamScale) || !Finite(expected) || !Close(basis, expected))
		return std::nullopt;
	StoredAlignment stored{ Fold(state), basis, state.session };
	return Finite(stored) ? std::optional<StoredAlignment>(stored) : std::nullopt;
}

Restoration Restore(const std::optional<StoredAlignment> &stored, const CalibrationContext &profile, uint64_t currentSession)
{
	if (!stored || !stored->session || !Finite(*stored))
		return {};
	if (!Bound(profile))
		return { RestoreResult::NotBound, stored->alignment };
	const auto basis = ProfileBasis(profile);
	if (!Finite(basis) || !Close(stored->basis, basis))
		return { RestoreResult::BasisMismatch, stored->alignment };
	return { stored->session == currentSession ? RestoreResult::SameSession : RestoreResult::Applied, stored->alignment };
}

RestorePreparation PrepareRestore(bool profileReadSucceeded, uint64_t session, uint64_t lastSession, uint64_t profilePendingSession)
{
	if (!profileReadSucceeded)
		return { false, false, session && session != profilePendingSession };
	return { true, session && session != lastSession, false };
}

bool ShouldPoll(bool linkPending, bool connected, double now, double lastPoll)
{
	return !linkPending && connected && !(now - lastPoll < 1.0);
}

const char *ResultName(RestoreResult result)
{
	switch (result)
	{
	case RestoreResult::Applied: return "applied";
	case RestoreResult::SameSession: return "same-session";
	case RestoreResult::BasisMismatch: return "basis-mismatch";
	case RestoreResult::NotBound: return "not-bound";
	default: return "absent";
	}
}

static picojson::value Numbers(std::initializer_list<double> numbers)
{
	picojson::array array;
	for (double number : numbers)
		array.emplace_back(number);
	return picojson::value(array);
}

std::string Encode(const StoredAlignment &stored)
{
	const auto &b = stored.basis;
	picojson::object basis;
	basis["calibrationRotation"] = Numbers({ b.calibrationRotation.w, b.calibrationRotation.x, b.calibrationRotation.y, b.calibrationRotation.z });
	basis["calibrationTranslation"] = Numbers({ b.calibrationTranslation.v[0], b.calibrationTranslation.v[1], b.calibrationTranslation.v[2] });
	basis["offsetRotation"] = Numbers({ b.offsetRotation.w, b.offsetRotation.x, b.offsetRotation.y, b.offsetRotation.z });
	basis["offsetTranslation"] = Numbers({ b.offsetTranslation.v[0], b.offsetTranslation.v[1], b.offsetTranslation.v[2] });
	basis["calibrationScale"] = picojson::value(b.calibrationScale);
	basis["hmdScale"] = picojson::value(b.hmdScale);
	basis["trackerSerial"] = picojson::value(b.trackerSerial);
	char session[16];
	auto end = std::to_chars(std::begin(session), std::end(session), stored.session, 16).ptr;
	picojson::object value;
	value["session"] = picojson::value(std::string(session, end));
	value["rotation"] = Numbers({ stored.alignment.rotation.x(), stored.alignment.rotation.y(), stored.alignment.rotation.z() });
	value["translation"] = Numbers({ stored.alignment.translation.x(), stored.alignment.translation.y(), stored.alignment.translation.z() });
	value["basis"] = picojson::value(basis);
	return picojson::value(value).serialize();
}

static bool ReadNumber(const picojson::value &value, double &number)
{
	if (!value.is<double>() || !std::isfinite(value.get<double>()))
		return false;
	number = value.get<double>();
	return true;
}

static bool ReadNumbers(const picojson::value &value, std::initializer_list<double *> numbers)
{
	if (!value.is<picojson::array>() || value.get<picojson::array>().size() != numbers.size())
		return false;
	size_t index = 0;
	for (double *number : numbers)
		if (!ReadNumber(value.get<picojson::array>()[index++], *number))
			return false;
	return true;
}

std::optional<StoredAlignment> Decode(const std::string &json)
try
{
	picojson::value value;
	std::string error;
	const auto end = picojson::parse(value, json.begin(), json.end(), &error);
	if (!error.empty() || !value.is<picojson::object>()
		|| std::any_of(end, json.end(), [](unsigned char c) { return !std::isspace(c); }))
		return std::nullopt;
	const auto &basis = value.get("basis");
	if (!basis.is<picojson::object>() || !value.get("session").is<std::string>() || !basis.get("trackerSerial").is<std::string>())
		return std::nullopt;
	StoredAlignment stored;
	const auto &session = value.get("session").get<std::string>();
	auto parsed = std::from_chars(session.data(), session.data() + session.size(), stored.session, 16);
	if (parsed.ec != std::errc{} || parsed.ptr != session.data() + session.size() || !stored.session)
		return std::nullopt;
	auto &b = stored.basis;
	b.trackerSerial = basis.get("trackerSerial").get<std::string>();
	if (b.trackerSerial.empty()
		|| !ReadNumbers(value.get("rotation"), { &stored.alignment.rotation.x(), &stored.alignment.rotation.y(), &stored.alignment.rotation.z() })
		|| !ReadNumbers(value.get("translation"), { &stored.alignment.translation.x(), &stored.alignment.translation.y(), &stored.alignment.translation.z() })
		|| !ReadNumbers(basis.get("calibrationRotation"), { &b.calibrationRotation.w, &b.calibrationRotation.x, &b.calibrationRotation.y, &b.calibrationRotation.z })
		|| !ReadNumbers(basis.get("calibrationTranslation"), { &b.calibrationTranslation.v[0], &b.calibrationTranslation.v[1], &b.calibrationTranslation.v[2] })
		|| !ReadNumbers(basis.get("offsetRotation"), { &b.offsetRotation.w, &b.offsetRotation.x, &b.offsetRotation.y, &b.offsetRotation.z })
		|| !ReadNumbers(basis.get("offsetTranslation"), { &b.offsetTranslation.v[0], &b.offsetTranslation.v[1], &b.offsetTranslation.v[2] })
		|| !ReadNumber(basis.get("calibrationScale"), b.calibrationScale) || !ReadNumber(basis.get("hmdScale"), b.hmdScale))
		return std::nullopt;
	return stored;
}
catch (const std::exception &)
{
	return std::nullopt;
}

bool ShouldWrite(const std::optional<StoredAlignment> &latest, const std::optional<StoredAlignment> &written,
	double now, double lastWrite, bool exiting)
{
	return latest && (!written || Encode(*latest) != Encode(*written)) && (exiting || now - lastWrite >= 10.0);
}

}
