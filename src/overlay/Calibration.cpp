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
#include "OverlayLog.h"
#include "SpaceMemory.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <future>
#include <optional>
#include <tuple>

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
static double NextEvaluation = 0, NextSearchLog = 0, NextSolve = 0, LastApply = -1;
static std::string SolveSerial;
static size_t SolveKeyframes = 0;
static bool SolveAutomatic = false;
static const char *SolveMode = "manual";
static bool RawPoseReady = false;
static bool SpaceLinkPending = false;
static uint64_t LastSpaceSession = 0, ProfilePendingSession = 0;
static double LastSpacePoll = -1, LastSpaceWrite = 0;
static std::optional<spacememory::StoredAlignment> LatestSpace, WrittenSpace;
static spacememory::TiltRecalibration TiltRecalibration;
static spacememory::SpaceRealign SpaceRealign;
static double LastBoundLog = 0;
static void StartCalibration(bool tiltRecalibration);
static void ApplyCommittedState();
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

static std::optional<protocol::DriftState> ReadDriftState()
{
	try
	{
		return Driver.GetDriftState();
	}
	catch (const std::exception &error)
	{
		overlaylog::Write("space-read-error error=", std::quoted(error.what()));
		Driver.Disconnect(error.what(), CalCtx.timeLastTick);
		AppliedSpace = {};
		RawPoseReady = false;
		return std::nullopt;
	}
}

static SpaceRestoreStatus SpaceRestore;

static void LogSpaceRestore(const char *result, const spacememory::Alignment &alignment = {})
{
	overlaylog::Write("space-restore result=", result, " yaw_deg=", alignment.rotation.y(),
		" translation_m=", alignment.translation.norm() * 0.01);
	SpaceRestore = { result, alignment.rotation.y(), alignment.translation.norm() * 0.01 };
}

static void PrepareSpaceRestore()
{
	if (!SpaceLinkPending || !Driver.Connected())
		return;
	const auto state = ReadDriftState();
	const auto prepare = spacememory::PrepareRestore(ProfileReadSucceeded(), state ? state->session : 0,
		LastSpaceSession, ProfilePendingSession);
	if (prepare.logProfilePending)
	{
		ProfilePendingSession = state->session;
		LogSpaceRestore("profile-pending");
	}
	if (!prepare.consumeLink)
		return;
	// Consume before sending any desired state; a late polling reply must never fold mid-session.
	SpaceLinkPending = false;
	LastSpacePoll = CalCtx.timeLastTick;
	if (!prepare.claimSession)
		return;
	LastSpaceSession = state->session;
	LatestSpace.reset();
	WrittenSpace.reset();
	try
	{
		if (LoadLastSpace(WrittenSpace) == ReadResult::Error)
		{
			LogSpaceRestore("read-error");
			return;
		}
		const auto restore = spacememory::Restore(WrittenSpace, CalCtx, state->session);
		LogSpaceRestore(spacememory::ResultName(restore.result), restore.alignment);
		if (restore.result == spacememory::RestoreResult::Applied)
		{
			CalCtx.calibratedRotation = restore.alignment.rotation;
			CalCtx.calibratedTranslation = restore.alignment.translation;
			SaveProfile(CalCtx);
		}
		if (restore.result == spacememory::RestoreResult::Applied || restore.result == spacememory::RestoreResult::BasisMismatch
			|| restore.result == spacememory::RestoreResult::NotBound)
		{
			DeleteLastSpace();
			WrittenSpace.reset();
		}
	}
	catch (const std::exception &error)
	{
		overlaylog::Write("space-restore-error error=", std::quoted(error.what()));
	}
}

static void WriteSpaceMemory(bool exiting)
{
	try
	{
		if (!spacememory::ShouldWrite(LatestSpace, WrittenSpace, CalCtx.timeLastTick, LastSpaceWrite, exiting)
			|| !SaveLastSpace(*LatestSpace))
			return;
		WrittenSpace = LatestSpace;
		LastSpaceWrite = CalCtx.timeLastTick;
		if (exiting)
			overlaylog::Write("space-capture session=", std::hex, LatestSpace->session, std::dec,
				" yaw_deg=", LatestSpace->alignment.rotation.y(), " translation_m=", LatestSpace->alignment.translation.norm() * 0.01);
	}
	catch (const std::exception &error)
	{
		overlaylog::Write("space-write-error error=", std::quoted(error.what()));
	}
}

void FlushSpaceMemory()
{
	WriteSpaceMemory(true);
}

static void PollSpaceMemory(double time)
{
	if (!spacememory::ShouldPoll(SpaceLinkPending, Driver.Connected(), time, LastSpacePoll))
		return;
	LastSpacePoll = time;
	const auto state = ReadDriftState();
	if (state && state->session)
	{
		if (state->session != LastSpaceSession)
		{
			LastSpaceSession = state->session;
			LatestSpace.reset();
			WrittenSpace.reset();
			LogSpaceRestore("read-error");
		}
		if (state->enabled && time - LastBoundLog >= 60.0)
		{
			overlaylog::Write("bound-status tilt_deg=", state->tiltMismatchDeg, " tilt_samples=", state->tiltSamples,
				" updates=", state->updatesSinceChange, " drift_valid=", state->valid,
				" drift_yaw_deg=", spacememory::DriftYawDegrees(state->rotation), " drift_m=", Eigen::Map<const Eigen::Vector3d>(state->translation.v).norm());
			LastBoundLog = time;
		}
		if (TiltRecalibration.Update(*state, CalCtx, Attempt.has_value()))
		{
			overlaylog::Write("tilt-mismatch deg=", state->tiltMismatchDeg, " samples=", state->tiltSamples, " action=recalibrate");
			StartCalibration(true);
		}
		if (SpaceRealign.Update(*state, CalCtx, Attempt.has_value()))
		{
			const auto alignment = spacememory::Fold(*state);
			CalCtx.calibratedRotation = alignment.rotation;
			CalCtx.calibratedTranslation = alignment.translation;
			SaveProfile(CalCtx);
			ApplyCommittedState();
			overlaylog::Write("space-realign yaw_deg=", spacememory::DriftYawDegrees(state->rotation),
				" translation_m=", Eigen::Map<const Eigen::Vector3d>(state->translation.v).norm());
		}
		if (auto capture = spacememory::Capture(*state, CalCtx))
			LatestSpace = std::move(capture);
	}
	else
	{
		TiltRecalibration.ResetPolls();
		SpaceRealign.ResetPolls();
	}
	WriteSpaceMemory(false);
}

static void ApplyCommittedState()
{
	PrepareSpaceRestore();
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
	PrepareSpaceRestore();
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
		char controllerType[vr::k_unMaxPropertyStringSize] = {};
		vr::ETrackedPropertyError error;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ControllerType_String, controllerType, sizeof(controllerType), &error);
		Devices.push_back({ id, deviceClass, role,
			GetDeviceSerial(id), GetDeviceTrackingSystem(id), CalCtx.devicePoses[id].bDeviceIsConnected,
			CalCtx.devicePoses[id].bPoseIsValid, GetLighthouseModelScale(id), error == vr::TrackedProp_Success ? controllerType : "" });
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

static const char *AttemptMode() { return Attempt ? CalibrationAttemptMode(*Attempt) : "manual"; }

static void LogAttemptEnd(const char *event, const char *reason)
{
	if (Attempt)
		overlaylog::Write(event, " attempt=", Generation, " mode=", AttemptMode(), " serial=", std::quoted(Attempt->serial), " reason=", reason);
}

static void EndAttempt(const char *event, const char *reason)
{
	LogAttemptEnd(event, reason);
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
		LatestSpace.reset();
		WrittenSpace.reset();
		DeleteLastSpace();
		TiltRecalibration = {};
		SpaceRealign = {};
		LogAttemptEnd("cancel", "removed");
		RemoveCalibrationState(*this, Attempt, Acquisition);
		Acquisition.Refresh(Devices);
		AcquisitionResult = {};
		EndAttempt("cancel", "removed");
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
	{
		// Any user cancel holds tilt recalibration, also when Calibrate replaced a running one.
		TiltRecalibration.Cancel(CalCtx, LastSpaceSession);
		EndAttempt("cancel", "user");
	}
}

static void ApplyAutoAcquire(bool enabled)
{
	if (!enabled && Attempt && Attempt->automatic)
		LogAttemptEnd("cancel", "auto-off");
	overlaylog::Write("auto-setting enabled=", enabled);
	bool ended = ApplyAutoAcquireState(CalCtx, Attempt, Acquisition, enabled);
	AcquisitionResult = {};
	if (ended)
		EndAttempt("cancel", "auto-off");
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
	SpaceLinkPending = Driver.Connect();
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
		status.tiltRecalibration = Attempt->tiltRecalibration;
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

SpaceRestoreStatus GetSpaceRestoreStatus()
{
	return SpaceRestore;
}

static void Status(const std::string &text)
{
	CalCtx.statusLine = text;
	if (!Attempt)
		return;
	if (Attempt->statusLine != text)
		overlaylog::Write("attempt-status attempt=", Generation, " mode=", AttemptMode(), " serial=", std::quoted(Attempt->serial), " text=", std::quoted(text));
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

static void StartCalibration(bool tiltRecalibration)
{
	if (Attempt)
		EndAttempt("cancel", "manual-request");
	++Generation;
	Attempt.emplace();
	Attempt->tiltRecalibration = tiltRecalibration;
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
	overlaylog::Write("attempt-start attempt=", Generation, " mode=", AttemptMode(), " serial=", std::quoted(Attempt->serial), " seed=0");
	if (tiltRecalibration && ShowCalibrationNotification)
	{
		auto notice = RecalibrationNotice();
		CalCtx.notificationId = ShowCalibrationNotification(notice.text.c_str(), notice.type);
	}
}

void StartCalibration()
{
	StartCalibration(false);
}

static bool AutoEligible()
{
	return AutoAcquisitionEligible(CalCtx, ProfileReadSucceeded(), SettingsReadSucceeded());
}

static void LogSearch(const acquisition::Evaluation &evaluation)
{
	if (!evaluation.diagnostics) return;
	overlaylog::Write("search held_pairs=", evaluation.handPairs, " passing=", evaluation.passing.size());
	for (const auto &pair : evaluation.pairs)
		overlaylog::Write("hand-pair ", std::quoted(pair.hand), " <-> ", std::quoted(pair.controller),
			" keyframes=", pair.keyframes, " held=", pair.fit.held, " median_m=", pair.fit.error.median, " p90_m=", pair.fit.error.p90);
	for (const auto &candidate : evaluation.candidates)
		overlaylog::Write("candidate serial=", std::quoted(candidate.serial), " keyframes=", candidate.keyframes,
			" rigidity_deg=", candidate.check.rigidity.error.median / acquisition::Degrees,
			" distance_m=", candidate.check.distance.median, " vertical_m=", candidate.check.vertical.median,
			" vertical_world_m=", candidate.check.verticalWorld.median, " result=", candidate.check.failure);
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
				LogSearch(AcquisitionResult);
				auto confirmed = Acquisition.Confirm(AcquisitionResult, AcquisitionEpoch);
				if (confirmed)
				{
					overlaylog::Write("confirmation serial=", std::quoted(confirmed->serial), " keyframes=", confirmed->samples.size(), " evidence=", confirmed->count);
					Attempt = BeginAutomaticAttempt(*confirmed, CalibrationCapacity(CalCtx));
					NextSolve = 0;
					++Generation;
					ChooseTracker(ResolveSerial(Devices, confirmed->serial));
					overlaylog::Write("attempt-start attempt=", Generation, " mode=automatic serial=", std::quoted(Attempt->serial), " seed=", Attempt->sampler.Store().Samples().size());
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
			overlaylog::Write("error context=acquisition error=", std::quoted(error.what()));
		}
	}
	if (!AutoEligible())
		return;
	if (time >= NextEvaluation && !AcquisitionWork.valid())
	{
		AcquisitionGeneration = Generation;
		AcquisitionEpoch = Acquisition.Epoch();
		NextEvaluation = time + 1.0;
		bool diagnostics = time >= NextSearchLog;
		if (diagnostics) NextSearchLog = time + 5.0;
		auto input = Acquisition.Snapshot();
		AcquisitionWork = std::async(std::launch::async, [input = std::move(input), diagnostics] { return acquisition::Evaluate(input, diagnostics); });
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
		overlaylog::Write("tracker-selected attempt=", Generation, " mode=", AttemptMode(), " serial=", std::quoted(result.serial));
		ChooseTracker(result.id);
		break;
	case ManualDetection::Event::Collecting:
		Status("Move your head to identify the head tracker.");
		CalCtx.Progress(static_cast<int>(Detection.Progress()), 40);
		break;
	}
}

static acquisition::CalibrationSolution ReceiveSolve(bool current)
{
	try
	{
		auto result = CalibrationWork.get();
		bool success = SolveAutomatic ? acquisition::AutomaticCalibrationSucceeded(result) : result.holdout.rms <= 0.10;
		overlaylog::Write("solve attempt=", SolveGeneration, " mode=", SolveMode,
			" serial=", std::quoted(SolveSerial), " keyframes=", SolveKeyframes, " rms_m=", result.holdout.rms,
			" p90_m=", result.holdout.p90, " rotation_deg=", result.rotationError.median / acquisition::Degrees,
			" success=", success, " result=", current ? (success ? "commit" : "collecting") : "stale");
		return result;
	}
	catch (const std::exception &error)
	{
		overlaylog::Write("solve attempt=", SolveGeneration, " mode=", SolveMode,
			" serial=", std::quoted(SolveSerial), " keyframes=", SolveKeyframes, " result=error error=", std::quoted(error.what()));
		throw;
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
		overlaylog::Write("segment-restart attempt=", Generation, " mode=", AttemptMode(), " serial=", std::quoted(Attempt->serial), " segment=", Attempt->sampler.Segment(), " reason=headset-tracking-jump-or-return");
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
			EndAttempt("abandon", "new-rotation-evidence");
			return;
		}
	}
	if (CalibrationWork.valid() && CalibrationWork.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
	{
		try
		{
			auto result = ReceiveSolve(SolveGeneration == Generation && SolveSegment == Attempt->sampler.Segment());
			if (SolveGeneration == Generation && SolveSegment == Attempt->sampler.Segment())
			{
				Attempt->result = result;
				bool success = Attempt->automatic
					? acquisition::AutomaticCalibrationSucceeded(result)
					: result.holdout.rms <= 0.10;
				if (success)
				{
					bool automatic = Attempt->automatic;
					bool notify = automatic || Attempt->tiltRecalibration;
					CommitAttempt(CalCtx, *Attempt);
					SaveProfile(CalCtx);
					EndAttempt("commit", "solve-succeeded");
					if (notify && ShowCalibrationNotification)
					{
						auto notice = ReadyNotice(CalCtx);
						ShowCalibrationNotification(notice.text.c_str(), notice.type);
					}
					if (!automatic)
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
		if (!CalibrationReadyToSolve(*Attempt, CalCtx.SampleCount()))
		{
			Status("Need more varied head movement. Tilt and turn your head in different directions.");
			return;
		}
		Attempt->lastSolveCount = store.Count();
		SolveGeneration = Generation;
		SolveSegment = Attempt->sampler.Segment();
		SolveSerial = Attempt->serial;
		SolveAutomatic = Attempt->automatic;
		SolveMode = AttemptMode();
		SolveKeyframes = store.Samples().size();
		overlaylog::Write("solve-start attempt=", SolveGeneration, " mode=", SolveMode, " serial=", std::quoted(SolveSerial), " keyframes=", SolveKeyframes);
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

static void LogDeviceAndAcquisitionChanges()
{
	static std::vector<DeviceSnapshot> previous;
	static std::optional<AcquireState> previousState;
	for (auto &device : Devices)
	{
		device.connected = CalCtx.devicePoses[device.id].bDeviceIsConnected;
		device.poseValid = CalCtx.devicePoses[device.id].bPoseIsValid;
	}
	// Hand tracking flips pose validity whenever a hand leaves view, so only identity changes reprint the list.
	auto fields = [](const DeviceSnapshot &d) {
		return std::tie(d.id, d.deviceClass, d.role, d.serial, d.trackingSystem);
	};
	if (!previousState || previous.size() != Devices.size()
		|| !std::equal(Devices.begin(), Devices.end(), previous.begin(), [&](const auto &a, const auto &b) { return fields(a) == fields(b); }))
	{
		overlaylog::Write("devices count=", Devices.size());
		for (const auto &d : Devices)
			overlaylog::Write("device index=", d.id, " class=", d.deviceClass, " role_hint=", d.role,
				" serial=", std::quoted(d.serial), " system=", std::quoted(d.trackingSystem), " controller_type=", std::quoted(d.controllerType),
				" connected=", d.connected, " pose_valid=", d.poseValid);
		previous = Devices;
	}
	auto state = DescribeAcquisition(CalCtx, Attempt ? &*Attempt : nullptr, ProfileReadSucceeded(), SettingsReadSucceeded(), Acquisition.HasHands(), AcquisitionResult.handPairs);
	if (!previousState || *previousState != state)
	{
		const char *name = "unknown";
		switch (state)
		{
		case AcquireState::Off: name = "off"; break;
		case AcquireState::Bound: name = "bound"; break;
		case AcquireState::ProfileUnreadable: name = "profile-pending"; break;
		case AcquireState::NeedHands: name = "need-hands"; break;
		case AcquireState::Syncing: name = "syncing"; break;
		case AcquireState::Searching: name = "searching"; break;
		case AcquireState::Calibrating: name = "calibrating"; break;
		case AcquireState::Paused: name = "paused"; break;
		}
		overlaylog::Write("acquisition state=", name);
		previousState = state;
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
	if (connected) SpaceLinkPending = true;
	bool scan = Devices.empty() || time - CalCtx.timeLastScan >= 1.0;
	if (scan)
	{
		bool autoBeforeRead = CalCtx.autoAcquire;
		RetryConfiguration(CalCtx);
		if (autoBeforeRead != CalCtx.autoAcquire)
			ApplyAutoAcquire(CalCtx.autoAcquire);
		if (Attempt && !CalCtx.trackerSerial.empty() && Attempt->serial != CalCtx.trackerSerial)
		{
			EndAttempt("cancel", "profile-binding-changed");
			StartCalibration();
		}
		RefreshDevices();
		CalCtx.timeLastScan = time;
	}
	LogDeviceAndAcquisitionChanges();
	if (scan || connected || (CalCtx.state == CalibrationState::Editing && time - LastApply >= 0.1))
		ApplyCommittedState();
	PollSpaceMemory(time);
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
		EndAttempt("cancel", "calibration-state-changed");
	try
	{
		TickAcquisition(time, observationSpace);
		if (!Attempt)
		{
			if (CalibrationWork.valid() && CalibrationWork.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
				ReceiveSolve(false);
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
		overlaylog::Write("error context=tick error=", std::quoted(error.what()));
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
