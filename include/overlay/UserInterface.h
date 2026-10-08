// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <string>
#include <vector>

#include <openvr.h>

struct VRDevice
{
	int id = -1;
	vr::TrackedDeviceClass deviceClass;
	std::string model = "";
	std::string serial = "";
	std::string trackingSystem = "";
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
};

struct VRState
{
	std::vector<std::string> trackingSystems;
	std::vector<VRDevice> devices;
};

// Reads the devices from SteamVR; the UI preview links its own version.
VRState ReadVRState();

class UserInterface
{
public:
	enum class Page { Calibration, Smoothing, Settings };

	void Setup();
	void Render(bool runningInOverlay);
	void ShowPage(Page page) { this->page = page; }
	void ShowCalibrationProgress() { openProgress = true; }

private:
	Page page = Page::Calibration;
	bool openProgress = false;
};
