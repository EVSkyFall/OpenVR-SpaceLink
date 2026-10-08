// SPDX-License-Identifier: AGPL-3.0-only

#include "PreviewState.h"
#include "Configuration.h"

CalibrationContext CalCtx;

namespace preview
{
	State Current;
	Calls Count;
}

void StartCalibration() { ++preview::Count.startCalibration; }
void CancelCalibration() { ++preview::Count.cancelCalibration; }
void RemoveCalibration() { ++preview::Count.removeCalibration; }
void SendOneEuroParams() { ++preview::Count.sendOneEuroParams; }
void SaveProfile(CalibrationContext &, bool) { ++preview::Count.saveProfile; }

void SetAutoAcquire(bool enabled)
{
	CalCtx.autoAcquire = enabled;
	++preview::Count.setAutoAcquire;
}

HeadTrackerState GetHeadTrackerState() { return preview::Current.headTracker; }
AcquireStatus GetAcquireStatus() { return preview::Current.acquire; }
DriverLinkStatus GetDriverLinkStatus() { return preview::Current.driver; }
SpaceRestoreStatus GetSpaceRestoreStatus() { return preview::Current.restore; }
VRState ReadVRState() { return preview::Current.vr; }

namespace preview
{
	const char *const KoreanDriverError = "Driver is unavailable. \xED\x8C\x8C\xEC\x9D\xB4\xED\x94\x84\xEC\x9D\x98 \xEB\x8B\xA4\xEB\xA5\xB8 \xEB\x81\x9D\xEC\x97\x90 \xED\x94\x84\xEB\xA1\x9C\xEC\x84\xB8\xEC\x8A\xA4\xEA\xB0\x80 \xEC\x97\x86\xEC\x8A\xB5\xEB\x8B\x88\xEB\x8B\xA4.\r\n";

	namespace
	{
		const char *HmdSerial = "2G0YC5ZG0K0BXQ";
		const char *TrackerSerial = "LHR-2E9F41C7";
		const uint32_t TrackerId = 5;

		VRDevice Device(int id, vr::TrackedDeviceClass deviceClass, const char *system, const char *model, const char *serial,
			vr::ETrackedControllerRole role = vr::TrackedControllerRole_Invalid)
		{
			VRDevice device;
			device.id = id;
			device.deviceClass = deviceClass;
			device.trackingSystem = system;
			device.model = model;
			device.serial = serial;
			device.controllerRole = role;
			return device;
		}

		void Status(const std::string &text, int progress = -1, int target = 0)
		{
			CalCtx.statusLine = text;
			CalCtx.messages.clear();
			CalibrationContext::Message message(CalibrationContext::Message::String);
			message.str = text;
			CalCtx.messages.push_back(message);
			if (progress >= 0)
				CalCtx.Progress(progress, target);
		}

		void Unbound(AcquireState state, int handPairs)
		{
			Current.acquire.state = state;
			Current.acquire.handPairs = handPairs;
		}

		void Bound(HeadTrackerState headTracker)
		{
			CalCtx.trackerSerial = TrackerSerial;
			CalCtx.targetTrackingSystem = "lighthouse";
			CalCtx.hmdSerial = HmdSerial;
			CalCtx.targetID = TrackerId;
			CalCtx.validProfile = CalCtx.validRelativeOffset = CalCtx.enabled = true;
			CalCtx.calibratedRotation = Eigen::Vector3d(0.84, -37.18, 1.07);
			CalCtx.calibratedTranslation = Eigen::Vector3d(12.40, -3.85, 27.92);
			CalCtx.calibratedScale = 1.0;
			CalCtx.hmdScale = 1.0;
			CalCtx.devicePoses[TrackerId].bDeviceIsConnected = true;
			CalCtx.devicePoses[TrackerId].bPoseIsValid = headTracker == HeadTrackerState::Active;
			Current.headTracker = headTracker;
			Current.acquire.state = AcquireState::Bound;
			Current.acquire.trackerSerial = TrackerSerial;
			Current.acquire.handPairs = 2;
			Current.restore = { "applied", 12.6, 0.42 };
		}

		void Offline(const char *error)
		{
			Current.driver = { false, error };
			Current.restore = {};
		}
	}

	void Reset()
	{
		CalCtx = CalibrationContext();
		Current = State();
		Count = Calls();
		Current.vr.trackingSystems = { "oculus", "lighthouse" };
		Current.vr.devices = {
			Device(0, vr::TrackedDeviceClass_HMD, "oculus", "Meta Quest 3", HmdSerial),
			Device(3, vr::TrackedDeviceClass_Controller, "lighthouse", "Knuckles Left", "LHR-4F2A10B1", vr::TrackedControllerRole_LeftHand),
			Device(4, vr::TrackedDeviceClass_Controller, "lighthouse", "Knuckles Right", "LHR-4F2A10B2", vr::TrackedControllerRole_RightHand),
			Device(TrackerId, vr::TrackedDeviceClass_GenericTracker, "lighthouse", "VIVE Tracker 3.0 MV", TrackerSerial),
		};
		CalCtx.devicePoses[0].bDeviceIsConnected = CalCtx.devicePoses[0].bPoseIsValid = true;
	}

	std::vector<Scene> Scenes()
	{
		using Page = UserInterface::Page;
		return {
			{ "need-hands", [] { Unbound(AcquireState::NeedHands, 0); } },
			{ "searching", [] { Unbound(AcquireState::Searching, 2); } },
			{ "auto-calibrating", [] {
				Unbound(AcquireState::Calibrating, 2);
				Current.acquire.trackerSerial = TrackerSerial;
				Current.acquire.progress = 62;
				Current.acquire.target = 100;
				CalCtx.state = CalibrationState::Sampling;
				CalCtx.statusLine = "Collecting head movement.";
			} },
			{ "active", [] { Bound(HeadTrackerState::Active); } },
			{ "first-session", [] {
				Bound(HeadTrackerState::Active);
				Current.restore = { "absent" };
			} },
			{ "still-aligned", [] {
				Bound(HeadTrackerState::Active);
				Current.restore = { "same-session", 12.6, 0.42 };
			} },
			{ "fresh-alignment", [] {
				Bound(HeadTrackerState::Waiting);
				Current.restore = { "basis-mismatch", 9.3, 0.38 };
			} },
			{ "tilt-recalibrating", [] {
				Bound(HeadTrackerState::Waiting);
				CalCtx.state = CalibrationState::Sampling;
				Current.acquire.tiltRecalibration = true;
				Status("Collecting head movement.", 46, 100);
			} },
			{ "waiting", [] { Bound(HeadTrackerState::Waiting); } },
			{ "driver-offline", [] {
				Bound(HeadTrackerState::Waiting);
				Offline("Driver is unavailable. The system cannot find the file specified.\r\n");
			} },
			{ "korean-error", [] {
				Bound(HeadTrackerState::Waiting);
				Offline(KoreanDriverError);
			} },
			{ "manual", [] {
				Unbound(AcquireState::Off, 0);
				CalCtx.autoAcquire = false;
			} },
			{ "offline-searching", [] {
				Unbound(AcquireState::Searching, 2);
				Offline("Driver is unavailable. The system cannot find the file specified.\r\n");
			} },
			{ "needs-calibration", [] {
				Bound(HeadTrackerState::Waiting);
				CalCtx.validRelativeOffset = false;
				Current.restore = {};
			} },
			{ "edit", [] {
				Bound(HeadTrackerState::Active);
				CalCtx.state = CalibrationState::Editing;
			} },
			{ "progress", [] {
				Bound(HeadTrackerState::Waiting);
				CalCtx.state = CalibrationState::Sampling;
				Status("Collecting head movement.", 37, 100);
			}, Page::Calibration, true },
			{ "progress-detect", [] {
				Unbound(AcquireState::Paused, 2);
				CalCtx.state = CalibrationState::Detect;
				Status("Move your head to identify the head tracker.", 14, 40);
			}, Page::Calibration, true },
			{ "progress-done", [] {
				Bound(HeadTrackerState::Active);
				Status("Calibration finished.\n");
				CalCtx.statusLine.clear();
			}, Page::Calibration, true },
			{ "smoothing", [] { Bound(HeadTrackerState::Active); }, Page::Smoothing },
			{ "settings", [] { Bound(HeadTrackerState::Active); }, Page::Settings },
			{ "overlay", [] { Bound(HeadTrackerState::Active); }, Page::Calibration, false, true },
		};
	}
}
