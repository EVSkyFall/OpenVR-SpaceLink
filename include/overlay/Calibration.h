// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Core>
#include <openvr.h>
#include <vector>
#include <string>
#include <iostream>

#include "Protocol.h"

enum class CalibrationState
{
	None,
	Begin,
	Detect,
	WaitForTracker,
	Sampling,
	Editing,
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	uint32_t targetID = vr::k_unTrackedDeviceIndexInvalid;

	Eigen::Vector3d calibratedRotation = Eigen::Vector3d::Zero();
	Eigen::Vector3d calibratedTranslation = Eigen::Vector3d::Zero();
	double calibratedScale = 1.0;
	double targetModelScale = 1.0;
	double hmdScale = 1.0;

	vr::HmdQuaternion_t relativeRotation = { 1, 0, 0, 0 };
	vr::HmdVector3d_t relativeTranslation = { 0, 0, 0 };
	bool validRelativeOffset = false;

	std::string targetTrackingSystem;

	std::string hmdSerial;
	std::string trackerSerial;

	bool enabled = false;
	bool validProfile = false;
	bool autoAcquire = true;
	std::string statusLine;
	double timeLastTick = 0, timeLastScan = 0;
	double wantedUpdateInterval = 1.0;

	bool enableNative = false;
	bool fallbackToSlam = true;
	bool enableAngularVelocity = false;
	bool continuousSync = true;
	float predictionTime = 1.0f;

	bool headFilterEnabled = false;
	protocol::OneEuroParams headFilterParams = { 5.0, 0.8, 1.0 };
	protocol::OneEuroParams driftFilterParams = { 3.0, 1.3, 0.6 };

	vr::VRNotificationId notificationId = 0;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

	vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount]{};

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter{};
		vr::HmdVector2_t playSpaceSize{};
	} chaperone;

	void Clear();

	size_t SampleCount() const
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 100;
		case SLOW:
			return 250;
		case VERY_SLOW:
			return 500;
		}
		return 100;
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type) { }

		std::string str;
		int progress, target;
	};

	std::vector<Message> messages;

	void Log(const std::string &msg)
	{
		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		messages.back().str += msg;
		std::cerr << msg;
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
void CalibrationTick(double time);
void StartCalibration();
void LoadChaperoneBounds();
void ApplyChaperoneBounds();
void SendOneEuroParams();
enum class HeadTrackerState { Unbound, Waiting, Active };
enum class AcquireState { Off, Bound, ProfileUnreadable, NeedHands, Syncing, Searching, Calibrating, Paused };
struct AcquireStatus { AcquireState state = AcquireState::Off; int handPairs = 0; std::string trackerSerial; int progress = 0, target = 0; };
struct DriverLinkStatus { bool connected = false; std::string lastError; };
HeadTrackerState GetHeadTrackerState();
AcquireStatus GetAcquireStatus();
DriverLinkStatus GetDriverLinkStatus();
void CancelCalibration();
void SetAutoAcquire(bool enabled);
void RemoveCalibration();
void FlushSpaceMemory();
void SetCalibrationNotificationHandler(vr::VRNotificationId (*show)(const char *, vr::EVRNotificationType), void (*remove)(vr::VRNotificationId));
