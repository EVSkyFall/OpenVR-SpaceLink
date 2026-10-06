// SPDX-License-Identifier: AGPL-3.0-only

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "AttemptLifecycle.h"
#include "AutoAcquisition.h"
#include "DesiredState.h"
#include "ManualDetection.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <future>
#include <optional>

using acquisition::Pose;

static IPCClient Driver;
CalibrationContext CalCtx;
static std::optional<CalibrationAttempt> Attempt;
static acquisition::AutoAcquisition Acquisition;
static std::vector<DeviceSnapshot> Devices;
static ObservationSpace AppliedSpace;
static acquisition::Evaluation AcquisitionResult;
static std::future<acquisition::Evaluation> AcquisitionWork;
static std::future<acquisition::CalibrationSolution> CalibrationWork;
static uint64_t Generation = 0, AcquisitionGeneration = 0, AcquisitionEpoch = 0;
static uint64_t SolveGeneration = 0, SolveSegment = 0;
static double NextEvaluation = 0, NextSolve = 0, LastApply = -1;
static bool RawPoseReady = false;
static vr::VRNotificationId (*ShowCalibrationNotification)(const char *, vr::EVRNotificationType) = nullptr;
static void (*RemoveCalibrationNotification)(vr::VRNotificationId) = nullptr;

static ManualDetection Detection;

static std::string GetDeviceSerial(uint32_t id)
{
	char serial[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serial, vr::k_unMaxPropertyStringSize);
	return std::string(serial);
}

static std::string GetDeviceTrackingSystem(uint32_t id)
{
	char system[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, system, vr::k_unMaxPropertyStringSize);
	return std::string(system);
}

static std::string GetDeviceModelNumber(uint32_t id)
{
	char model[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, model, vr::k_unMaxPropertyStringSize);
	return std::string(model);
}

struct ModelScaleEntry
{
	const char *pattern;
	double scale;
};

static const ModelScaleEntry ModelScales[] = {
	{ "tundra tracker",           0.9969 	},  	// Tundra Tracker
	{ "vive tracker 3.0 mv",      1.0034 	},  	// HTC Vive Tracker 3.0
	{ "vive tracker mv",          1.00585 	}, 		// HTC Vive Tracker 1.0 / 2018
};

static double GetLighthouseModelScale(uint32_t id)
{
	if (id == vr::k_unTrackedDeviceIndexInvalid) 
		return 1.0;

	std::string model = GetDeviceModelNumber(id);
	std::transform(model.begin(), model.end(), model.begin(),
		[](unsigned char c) { return (char)std::tolower(c); });

	for (auto &entry : ModelScales)
	{
		if (model.find(entry.pattern) != std::string::npos)
			return entry.scale;
	}

	return 1.0;
}

static bool SendCommands(const std::vector<protocol::Request> &commands)
{
	if (!Driver.Connected())
		return false;
	try
	{
		for (const auto &command : commands)
		{
			auto response = Driver.SendBlocking(command);
			if (response.type != protocol::ResponseSuccess)
				throw std::runtime_error("Driver did not acknowledge the request.");
			AppliedSpace.Applied(command);
		}
		return true;
	}
	catch (const std::exception &error)
	{
		Driver.Disconnect(error.what(), CalCtx.timeLastTick);
		AppliedSpace = {};
		RawPoseReady = false;
		return false;
	}
}

static void ApplyCommittedState()
{
	CalCtx.targetID = ResolveSerial(Devices, CalCtx.trackerSerial);
	CalCtx.enabled = CalCtx.validProfile;
	bool sampling = Attempt && CalCtx.state == CalibrationState::Sampling;
	bool sent = SendCommands(DesiredState(Devices, CalCtx, sampling, sampling ? Attempt->id : vr::k_unTrackedDeviceIndexInvalid));
	if (sampling)
		RawPoseReady = sent;
	LastApply = CalCtx.timeLastTick;
}

void SendOneEuroParams()
{
	auto commands = DesiredState({}, CalCtx, false);
	SendCommands({ commands.back() });
}

static void RefreshDevices()
{
	Devices.clear();
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;
		auto role = static_cast<vr::ETrackedControllerRole>(vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32));
		Devices.push_back({ id, deviceClass, role,
			GetDeviceSerial(id), GetDeviceTrackingSystem(id), CalCtx.devicePoses[id].bDeviceIsConnected,
			CalCtx.devicePoses[id].bPoseIsValid, GetLighthouseModelScale(id) });
	}
	CalCtx.targetID = ResolveSerial(Devices, CalCtx.trackerSerial);
	if (Attempt)
		Attempt->id = ResolveSerial(Devices, Attempt->serial);
	Detection.Resolve(Devices);
	Acquisition.Refresh(Devices);
}

static void RemoveFoundNotification()
{
	if (CalCtx.notificationId && RemoveCalibrationNotification)
		RemoveCalibrationNotification(CalCtx.notificationId);
	CalCtx.notificationId = 0;
}

static void EndAttempt()
{
	RemoveFoundNotification();
	DiscardAttempt(CalCtx, Attempt);
	Detection = {};
	RawPoseReady = false;
	++Generation;
	ApplyCommittedState();
}

void CalibrationContext::Clear()
{
	if (this == &CalCtx)
	{
		RemoveCalibrationState(*this, Attempt, Acquisition);
		Acquisition.Refresh(Devices);
		AcquisitionResult = {};
		EndAttempt();
	}
	else
		ClearCommittedProfile(*this);
}

void RemoveCalibration()
{
	CalCtx.Clear();
	SaveProfile(CalCtx);
}

void CancelCalibration()
{
	if (Attempt && !Attempt->automatic)
		EndAttempt();
}

static void ApplyAutoAcquire(bool enabled)
{
	bool ended = ApplyAutoAcquireState(CalCtx, Attempt, Acquisition, enabled);
	AcquisitionResult = {};
	if (ended)
		EndAttempt();
	if (enabled)
		Acquisition.Refresh(Devices);
}

void SetAutoAcquire(bool enabled)
{
	ApplyAutoAcquire(enabled);
	SaveAppSettings(CalCtx);
}

void SetCalibrationNotificationHandler(vr::VRNotificationId (*show)(const char *, vr::EVRNotificationType), void (*remove)(vr::VRNotificationId))
{
	ShowCalibrationNotification = show;
	RemoveCalibrationNotification = remove;
}

void InitCalibrator()
{
	Driver.Connect();
}

HeadTrackerState GetHeadTrackerState()
{
	if (CalCtx.trackerSerial.empty())
		return HeadTrackerState::Unbound;
	uint32_t id = ResolveSerial(Devices, CalCtx.trackerSerial);
	return Driver.Connected() && CalCtx.validProfile && CalCtx.validRelativeOffset
		&& CalCtx.state != CalibrationState::Sampling && id < vr::k_unMaxTrackedDeviceCount && CalCtx.devicePoses[id].bPoseIsValid
		? HeadTrackerState::Active : HeadTrackerState::Waiting;
}

AcquireStatus GetAcquireStatus()
{
	AcquireStatus status;
	status.handPairs = AcquisitionResult.handPairs;
	if (Attempt)
	{
		status.trackerSerial = Attempt->serial;
		status.progress = static_cast<int>(Attempt->sampler.Store().Samples().size());
		status.target = static_cast<int>(CalCtx.SampleCount());
	}
	if (!CalCtx.trackerSerial.empty())
		status.trackerSerial = CalCtx.trackerSerial;
	status.state = DescribeAcquisition(CalCtx, Attempt ? &*Attempt : nullptr, ProfileReadSucceeded(), SettingsReadSucceeded(),
		Acquisition.HasHands(), status.handPairs);
	return status;
}

DriverLinkStatus GetDriverLinkStatus()
{
	return { Driver.Connected(), Driver.LastError() };
}

static void Status(const std::string &text)
{
	CalCtx.statusLine = text;
	if (!Attempt)
		return;
	Attempt->statusLine = text;
	if (!Attempt->automatic)
	{
		CalCtx.messages.clear();
		CalCtx.messages.push_back(CalibrationContext::Message(CalibrationContext::Message::String));
		CalCtx.messages.back().str = text;
		CalCtx.Progress(static_cast<int>(Attempt->sampler.Store().Samples().size()), static_cast<int>(CalCtx.SampleCount()));
	}
}

static void ChooseTracker(uint32_t id)
{
	Attempt->id = id;
	for (const auto &device : Devices)
		if (device.id == id)
		{
			Attempt->serial = device.serial;
			Attempt->trackingSystem = device.trackingSystem;
			Attempt->modelScale = device.modelScale;
		}
	Attempt->hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);
	CalCtx.state = CalibrationState::WaitForTracker;
	Status(WaitingForTracker(Attempt->serial));
}

void StartCalibration()
{
	if (Attempt)
		EndAttempt();
	++Generation;
	Attempt.emplace();
	CalCtx.messages.clear();
	CalCtx.wantedUpdateInterval = 0;
	Detection = {};
	Acquisition.ResetConfirmations();
	if (!CalCtx.trackerSerial.empty())
	{
		Attempt->serial = CalCtx.trackerSerial;
		Attempt->trackingSystem = CalCtx.targetTrackingSystem;
		Attempt->id = ResolveSerial(Devices, Attempt->serial);
		CalCtx.state = CalibrationState::WaitForTracker;
		Status(WaitingForTracker(Attempt->serial));
	}
	else
	{
		CalCtx.state = CalibrationState::Begin;
		Status("Waiting for a head tracker.");
	}
}

static bool AutoEligible()
{
	return AutoAcquisitionEligible(CalCtx, ProfileReadSucceeded(), SettingsReadSucceeded());
}

static void TickAcquisition(double time, const ObservationSpace &observationSpace)
{
	if (AutoEligible())
		Acquisition.ObserveConfigured(time, CalCtx.devicePoses, observationSpace);
	if (AcquisitionWork.valid() && AcquisitionWork.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
	{
		try
		{
			auto result = AcquisitionWork.get();
			if (AutoEligible() && AcquisitionGeneration == Generation && AcquisitionEpoch == Acquisition.Epoch())
			{
				AcquisitionResult = std::move(result);
				auto confirmed = Acquisition.Confirm(AcquisitionResult, AcquisitionEpoch);
				if (confirmed)
				{
					Attempt.emplace();
					Attempt->automatic = true;
					Attempt->confirmationRotation = confirmed->rotation;
					Attempt->serial = confirmed->serial;
					Attempt->trackingSystem = confirmed->system;
					ChooseTracker(ResolveSerial(Devices, confirmed->serial));
					++Generation;
					if (ShowCalibrationNotification)
					{
						auto notice = FoundNotice(confirmed->serial);
						CalCtx.notificationId = ShowCalibrationNotification(notice.text.c_str(), notice.type);
					}
				}
			}
		}
		catch (const std::exception &error)
		{
			std::cerr << "Evaluating acquisition: " << error.what() << '\n';
		}
	}
	if (!AutoEligible())
		return;
	if (time >= NextEvaluation && !AcquisitionWork.valid())
	{
		AcquisitionGeneration = Generation;
		AcquisitionEpoch = Acquisition.Epoch();
		NextEvaluation = time + 1.0;
		auto input = Acquisition.Snapshot();
		AcquisitionWork = std::async(std::launch::async, [input = std::move(input)] { return acquisition::Evaluate(input); });
	}
}

static void TickManualDetection(double time)
{
	auto result = Detection.Observe(time, Devices, CalCtx.devicePoses);
	if (Detection.Started())
		CalCtx.state = CalibrationState::Detect;
	switch (result.event)
	{
	case ManualDetection::Event::Waiting:
		Status("Waiting for a head tracker.");
		break;
	case ManualDetection::Event::HeadsetPaused:
		Status("Headset is not tracking. Paused until it is back.");
		break;
	case ManualDetection::Event::TrackerPaused:
		Status("Head tracker is not tracking. Paused until it is back.");
		break;
	case ManualDetection::Event::Selected:
		Attempt->serial = result.serial;
		ChooseTracker(result.id);
		break;
	case ManualDetection::Event::Collecting:
		Status("Move your head to identify the head tracker.");
		CalCtx.Progress(static_cast<int>(Detection.Progress()), 40);
		break;
	}
}

static void TickSampling(double time)
{
	if (!RawPoseReady)
		return;
	const auto &hmd = CalCtx.devicePoses[0];
	uint32_t id = Attempt->id;
	vr::TrackedDevicePose_t tracker{};
	if (id < vr::k_unMaxTrackedDeviceCount)
		tracker = CalCtx.devicePoses[id];
	Pose reference(hmd.mDeviceToAbsoluteTracking), target(tracker.mDeviceToAbsoluteTracking);
	auto event = Attempt->sampler.Observe(time, reference, hmd.bPoseIsValid, target, tracker.bPoseIsValid, CalibrationCapacity(CalCtx));
	if (event == acquisition::SampleEvent::HeadsetPaused)
	{
		Attempt->paused = true;
		Status("Headset is not tracking. Paused until it is back.");
	}
	else if (event == acquisition::SampleEvent::TrackerPaused)
	{
		Attempt->paused = true;
		Status("Head tracker is not tracking. Paused until it is back.");
	}
	else if (event == acquisition::SampleEvent::SegmentRestarted)
	{
		Attempt->lastSolveCount = 0;
		Attempt->rigiditySampler.Clear();
		Attempt->paused = true;
		Status("Headset tracking jumped. Starting the sample set again.");
	}
	else
	{
		Status(Attempt->paused ? "Collecting head movement." : Attempt->statusLine);
		Attempt->paused = false;
	}
	if (event == acquisition::SampleEvent::HeadsetPaused || event == acquisition::SampleEvent::TrackerPaused)
	{
		Attempt->rigiditySampler.Observe(time, reference, hmd.bPoseIsValid, target, tracker.bPoseIsValid,
			CalibrationCapacity(CalCtx), true, true, true);
		return;
	}
	if (Attempt->automatic)
	{
		Attempt->rigiditySampler.Observe(time, reference, hmd.bPoseIsValid, target, tracker.bPoseIsValid,
			CalibrationCapacity(CalCtx), true, true, true);
		const auto &newer = Attempt->rigiditySampler.Store().Samples();
		if (acquisition::AutomaticCalibrationAbandoned(newer.size(), acquisition::RotationConsistency(newer, Attempt->confirmationRotation).median))
		{
			Acquisition.ResetConfirmations();
			EndAttempt();
			return;
		}
	}
	if (CalibrationWork.valid() && CalibrationWork.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
	{
		try
		{
			auto result = CalibrationWork.get();
			if (SolveGeneration == Generation && SolveSegment == Attempt->sampler.Segment())
			{
				Attempt->result = result;
				bool success = Attempt->automatic
					? acquisition::AutomaticCalibrationSucceeded(result)
					: result.holdout.rms <= 0.10;
				if (success)
				{
					bool automatic = Attempt->automatic;
					CommitAttempt(CalCtx, *Attempt);
					SaveProfile(CalCtx);
					EndAttempt();
					if (automatic && ShowCalibrationNotification)
					{
						auto notice = ReadyNotice(CalCtx);
						ShowCalibrationNotification(notice.text.c_str(), notice.type);
					}
					else if (!automatic)
					{
						CalCtx.messages.clear();
						CalCtx.Log("Calibration finished.\n");
					}
					return;
				}
				if (!Attempt->automatic)
				{
					char message[192];
					snprintf(message, sizeof message, "Calibration error too high (%.1f mm). Collecting more samples, move smoothly.", result.holdout.rms * 1000.0);
					Status(message);
				}
			}
		}
		catch (const std::exception &error)
		{
			std::cerr << "Solving calibration: " << error.what() << '\n';
			Attempt->lastSolveCount = 0;
			NextSolve = time + 1.0;
		}
	}
	const auto &store = Attempt->sampler.Store();
	if (store.Samples().size() >= CalCtx.SampleCount() && store.Count() != Attempt->lastSolveCount && !CalibrationWork.valid() && time >= NextSolve)
	{
		std::vector<Eigen::Matrix3d> rotations;
		for (const auto &sample : store.Samples())
			rotations.push_back(sample.target.rot);
		if (acquisition::RotationInformation(rotations) < 0.10 * 0.10)
		{
			Status("Need more varied head movement. Tilt and turn your head in different directions.");
			return;
		}
		Attempt->lastSolveCount = store.Count();
		SolveGeneration = Generation;
		SolveSegment = Attempt->sampler.Segment();
		auto samples = Attempt->sampler.Snapshot();
		double scale = Attempt->modelScale;
		try
		{
			CalibrationWork = std::async(std::launch::async, [samples = std::move(samples), scale] { return acquisition::SolveCalibration(samples, scale); });
		}
		catch (...)
		{
			Attempt->lastSolveCount = 0;
			NextSolve = time + 1.0;
			throw;
		}
	}
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem() || time - CalCtx.timeLastTick < 0.05)
		return;
	CalCtx.timeLastTick = time;
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, CalCtx.devicePoses, vr::k_unMaxTrackedDeviceCount);
	const auto observationSpace = AppliedSpace;
	bool wasRaw = RawPoseReady;
	bool connected = Driver.Connect(time);
	bool scan = Devices.empty() || time - CalCtx.timeLastScan >= 1.0;
	if (scan)
	{
		bool autoBeforeRead = CalCtx.autoAcquire;
		RetryConfiguration(CalCtx);
		if (autoBeforeRead != CalCtx.autoAcquire)
			ApplyAutoAcquire(CalCtx.autoAcquire);
		if (Attempt && !CalCtx.trackerSerial.empty() && Attempt->serial != CalCtx.trackerSerial)
		{
			EndAttempt();
			StartCalibration();
		}
		RefreshDevices();
		CalCtx.timeLastScan = time;
	}
	if (scan || connected || (CalCtx.state == CalibrationState::Editing && time - LastApply >= 0.1))
		ApplyCommittedState();
	if (scan && CalCtx.validProfile && CalCtx.chaperone.valid && CalCtx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);
		// Preserve manually moved bounds; restore when SteamVR resets their geometry.
		if (quadCount != CalCtx.chaperone.geometry.size())
			ApplyChaperoneBounds();
	}
	if (Attempt && CalCtx.state != CalibrationState::Begin && CalCtx.state != CalibrationState::Detect
		&& CalCtx.state != CalibrationState::WaitForTracker && CalCtx.state != CalibrationState::Sampling)
		EndAttempt();
	try
	{
		TickAcquisition(time, observationSpace);
		if (!Attempt)
		{
			if (CalibrationWork.valid() && CalibrationWork.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
				CalibrationWork.get();
			return;
		}
		if (CalCtx.state == CalibrationState::Begin || CalCtx.state == CalibrationState::Detect)
		{
			TickManualDetection(time);
			return;
		}
		if (CalCtx.state == CalibrationState::WaitForTracker)
		{
			uint32_t id = Attempt->id;
			if (id >= vr::k_unMaxTrackedDeviceCount || !CalCtx.devicePoses[id].bPoseIsValid || !Pose(CalCtx.devicePoses[id].mDeviceToAbsoluteTracking).Finite())
			{
				Status(WaitingForTracker(Attempt->serial));
				return;
			}
			if (!CalCtx.devicePoses[0].bPoseIsValid || !Pose(CalCtx.devicePoses[0].mDeviceToAbsoluteTracking).Finite())
			{
				Status("Headset is not tracking. Paused until it is back.");
				return;
			}
			ChooseTracker(id);
			CalCtx.state = CalibrationState::Sampling;
			ApplyCommittedState();
			Status("Collecting head movement.");
			return;
		}
		// The poses fetched before a successful disable still include the previous override.
		if (CalCtx.state == CalibrationState::Sampling && wasRaw)
			TickSampling(time);
	}
	catch (const std::exception &error)
	{
		std::cerr << "Calibration tick: " << error.what() << '\n';
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
