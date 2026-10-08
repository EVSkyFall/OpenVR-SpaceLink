// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"
#include "UserInterface.h"

#include <functional>
#include <string>
#include <vector>

// Stands in for the calibrator and SteamVR, so UserInterface can be drawn without them.
namespace preview
{
	struct State
	{
		DriverLinkStatus driver{ true, "" };
		HeadTrackerState headTracker = HeadTrackerState::Unbound;
		AcquireStatus acquire;
		SpaceRestoreStatus restore;
		VRState vr;
	};

	// What the driver reports on a Korean Windows when the pipe has no listener.
	extern const char *const KoreanDriverError;

	struct Calls
	{
		int startCalibration = 0, cancelCalibration = 0, removeCalibration = 0;
		int setAutoAcquire = 0, sendOneEuroParams = 0, saveProfile = 0;
	};

	extern State Current;
	extern Calls Count;

	struct Scene
	{
		std::string name;
		std::function<void()> setup;
		UserInterface::Page page = UserInterface::Page::Calibration;
		bool progress = false;
		bool overlay = false;
	};

	void Reset();
	std::vector<Scene> Scenes();
}
