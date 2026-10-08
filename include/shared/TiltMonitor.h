// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tilt
{

inline double MismatchDegrees(const vr::HmdQuaternion_t &derived, const vr::HmdQuaternion_t &raw)
{
	auto localUp = [](const vr::HmdQuaternion_t &q) {
		return vr::HmdQuaternion_t{ 0, 2 * (q.x * q.y + q.w * q.z),
			q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z, 2 * (q.y * q.z - q.w * q.x) };
	};
	const auto a = localUp(derived), b = localUp(raw);
	const double dot = a.x * b.x + a.y * b.y + a.z * b.z;
	const double norm = std::sqrt((a.x * a.x + a.y * a.y + a.z * a.z) * (b.x * b.x + b.y * b.y + b.z * b.z));
	return std::acos(std::clamp(dot / norm, -1.0, 1.0)) * (180.0 / 3.14159265358979323846);
}

struct Monitor
{
	double mismatchDeg = 0;
	uint64_t samples = 0;

	void Reset() { mismatchDeg = 0; samples = 0; }

	void Update(const vr::HmdQuaternion_t &derived, const vr::HmdQuaternion_t &raw, double angularSpeed, double dt)
	{
		if (angularSpeed > 1.0)
			return;
		const double value = MismatchDegrees(derived, raw);
		mismatchDeg = samples ? mismatchDeg + (1 - std::exp(-dt / 0.3)) * (value - mismatchDeg) : value;
		++samples;
	}
};

}
