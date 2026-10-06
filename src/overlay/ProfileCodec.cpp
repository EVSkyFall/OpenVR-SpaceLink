// SPDX-License-Identifier: AGPL-3.0-only

#include "ProfileCodec.h"
#include "AttemptLifecycle.h"

#include <picojson.h>
#include <sstream>
#include <stdexcept>
#include <algorithm>

static picojson::array FloatArray(const float *buf, int numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

static void LoadFloatArray(const picojson::value &obj, float *buf, int numFloats)
{
	if (!obj.is<picojson::array>())
		throw std::runtime_error("expected array, got " + obj.to_str());

	auto &arr = obj.get<picojson::array>();
	if (arr.size() != numFloats)
		throw std::runtime_error("wrong buffer size");

	for (int i = 0; i < numFloats; i++)
		{
		if (!arr[i].is<double>())
			throw std::runtime_error("Expected a chaperone number.");
		buf[i] = static_cast<float>(arr[i].get<double>());
	}
}

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty())
		throw std::runtime_error(err);

	if (!v.is<picojson::array>())
		throw std::runtime_error("Expected a profile array.");
	auto arr = v.get<picojson::array>();
	if (arr.size() < 1)
		throw std::runtime_error("no profiles in file");

	if (!arr[0].is<picojson::object>())
		throw std::runtime_error("Expected a profile object.");
	auto obj = arr[0].get<picojson::object>();
	for (const char *key : { "target_tracking_system" })
		if (!obj[key].is<std::string>())
			throw std::runtime_error(std::string("Expected a string: ") + key);
	for (const char *key : { "roll", "yaw", "pitch", "x", "y", "z" })
		if (!obj[key].is<double>())
			throw std::runtime_error(std::string("Expected a number: ") + key);

	ctx.targetTrackingSystem = obj["target_tracking_system"].get<std::string>();

	if (obj["hmd_serial"].is<std::string>())
		ctx.hmdSerial = obj["hmd_serial"].get<std::string>();
	if (obj["tracker_serial"].is<std::string>())
		ctx.trackerSerial = obj["tracker_serial"].get<std::string>();
	ctx.calibratedRotation(0) = obj["roll"].get<double>();
	ctx.calibratedRotation(1) = obj["yaw"].get<double>();
	ctx.calibratedRotation(2) = obj["pitch"].get<double>();
	ctx.calibratedTranslation(0) = obj["x"].get<double>();
	ctx.calibratedTranslation(1) = obj["y"].get<double>();
	ctx.calibratedTranslation(2) = obj["z"].get<double>();

	if (obj["scale"].is<double>())
		ctx.calibratedScale = obj["scale"].get<double>();
	else
		ctx.calibratedScale = 1.0;

	if (obj["targetModelScale"].is<double>())
		ctx.targetModelScale = obj["targetModelScale"].get<double>();
	else
		ctx.targetModelScale = ctx.calibratedScale;

	if (obj["hmdScale"].is<double>())
		ctx.hmdScale = obj["hmdScale"].get<double>();
	else
		ctx.hmdScale = 1.0;

	if (ctx.targetModelScale <= 0.0)
		ctx.targetModelScale = 1.0;
	if (ctx.hmdScale <= 0.0)
		ctx.hmdScale = 1.0;

	ctx.enableNative = obj["native"].is<bool>() ? obj["native"].get<bool>() : false;
	ctx.fallbackToSlam = obj["fallbackSlam"].is<bool>() ? obj["fallbackSlam"].get<bool>() : true;
	ctx.enableAngularVelocity = obj["eAngVel"].is<bool>() ? obj["eAngVel"].get<bool>() : false;

	if (obj["continuousSync"].is<bool>())
		ctx.continuousSync = obj["continuousSync"].get<bool>();
	else
		ctx.continuousSync = true;

	if (obj["predictionTime"].is<double>())
		ctx.predictionTime = static_cast<float>(obj["predictionTime"].get<double>());
	else
		ctx.predictionTime = 1.0;

	auto loadOneEuro = [&](const char *key, protocol::OneEuroParams &out, protocol::OneEuroParams def) {
		out = def;
		if (!obj[key].is<picojson::object>())
			return;
		auto o = obj[key].get<picojson::object>();
		if (o["minCutoff"].is<double>()) out.minCutoff = o["minCutoff"].get<double>();
		if (o["beta"].is<double>())      out.beta = o["beta"].get<double>();
		if (o["dCutoff"].is<double>())   out.dCutoff = o["dCutoff"].get<double>();
	};

	ctx.headFilterEnabled = obj["headFilterEnabled"].is<bool>() ? obj["headFilterEnabled"].get<bool>() : true;
	loadOneEuro("headFilter", ctx.headFilterParams, { 2.0, 0.5, 1.0 });
	loadOneEuro("driftFilter", ctx.driftFilterParams, { 1.0, 0.4, 0.85 });

	if (obj["rel_qw"].is<double>())
	{
		for (const char *key : { "rel_qx", "rel_qy", "rel_qz", "rel_tx", "rel_ty", "rel_tz" })
			if (!obj[key].is<double>())
				throw std::runtime_error(std::string("Expected a relative offset number: ") + key);
		ctx.relativeRotation.w = obj["rel_qw"].get<double>();
		ctx.relativeRotation.x = obj["rel_qx"].get<double>();
		ctx.relativeRotation.y = obj["rel_qy"].get<double>();
		ctx.relativeRotation.z = obj["rel_qz"].get<double>();
		ctx.relativeTranslation.v[0] = obj["rel_tx"].get<double>();
		ctx.relativeTranslation.v[1] = obj["rel_ty"].get<double>();
		ctx.relativeTranslation.v[2] = obj["rel_tz"].get<double>();
		ctx.validRelativeOffset = true;
	}
	else
	{
		ctx.validRelativeOffset = false;
	}

	if (obj["calibration_speed"].is<double>())
		ctx.calibrationSpeed = (CalibrationContext::Speed)(int) obj["calibration_speed"].get<double>();

	if (obj["chaperone"].is<picojson::object>())
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
		ctx.chaperone.autoApply = chaperone["auto_apply"].is<bool>() ? chaperone["auto_apply"].get<bool>() : true;

		LoadFloatArray(chaperone["play_space_size"], ctx.chaperone.playSpaceSize.v, 2);

		LoadFloatArray(
			chaperone["standing_center"],
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		);

		if (!chaperone["geometry"].is<picojson::array>())
			throw std::runtime_error("chaperone geometry is not an array");

		auto &geometry = chaperone["geometry"].get<picojson::array>();

		if (geometry.size() > 0)
		{
			if (geometry.size() % 12 != 0)
				throw std::runtime_error("Incomplete chaperone quad.");
			ctx.chaperone.geometry.resize(geometry.size() / 12);
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), static_cast<int>(geometry.size()));

			ctx.chaperone.valid = true;
		}
	}

	ctx.validProfile = obj["valid_profile"].is<bool>() ? obj["valid_profile"].get<bool>() : true;
	if (!ctx.validProfile)
	{
		ctx.trackerSerial.clear();
		ctx.relativeRotation = { 1, 0, 0, 0 };
		ctx.relativeTranslation = {};
		ctx.validRelativeOffset = false;
	}
}

static void WriteProfile(const CalibrationContext &ctx, std::ostream &out)
{
	picojson::object profile;
	profile["valid_profile"].set<bool>(ctx.validProfile);
	profile["target_tracking_system"].set<std::string>(ctx.targetTrackingSystem);
	profile["hmd_serial"].set<std::string>(ctx.hmdSerial);
	profile["tracker_serial"].set<std::string>(ctx.trackerSerial);
	profile["roll"].set<double>(ctx.calibratedRotation(0));
	profile["yaw"].set<double>(ctx.calibratedRotation(1));
	profile["pitch"].set<double>(ctx.calibratedRotation(2));
	profile["x"].set<double>(ctx.calibratedTranslation(0));
	profile["y"].set<double>(ctx.calibratedTranslation(1));
	profile["z"].set<double>(ctx.calibratedTranslation(2));
	profile["scale"].set<double>(ctx.calibratedScale);
	profile["targetModelScale"].set<double>(ctx.targetModelScale);
	profile["hmdScale"].set<double>(ctx.hmdScale);

	profile["native"].set<bool>(ctx.enableNative);
	profile["fallbackSlam"].set<bool>(ctx.fallbackToSlam);
	profile["eAngVel"].set<bool>(ctx.enableAngularVelocity);
	profile["continuousSync"].set<bool>(ctx.continuousSync);

	double time = ctx.predictionTime;
	profile["predictionTime"].set<double>(time);

	profile["headFilterEnabled"].set<bool>(ctx.headFilterEnabled);

	auto saveOneEuro = [](const protocol::OneEuroParams &p) {
		picojson::object o;
		o["minCutoff"].set<double>(p.minCutoff);
		o["beta"].set<double>(p.beta);
		o["dCutoff"].set<double>(p.dCutoff);
		return o;
	};
	profile["headFilter"].set<picojson::object>(saveOneEuro(ctx.headFilterParams));
	profile["driftFilter"].set<picojson::object>(saveOneEuro(ctx.driftFilterParams));

	if (ctx.validRelativeOffset)
	{
		profile["rel_qw"].set<double>(ctx.relativeRotation.w);
		profile["rel_qx"].set<double>(ctx.relativeRotation.x);
		profile["rel_qy"].set<double>(ctx.relativeRotation.y);
		profile["rel_qz"].set<double>(ctx.relativeRotation.z);
		profile["rel_tx"].set<double>(ctx.relativeTranslation.v[0]);
		profile["rel_ty"].set<double>(ctx.relativeTranslation.v[1]);
		profile["rel_tz"].set<double>(ctx.relativeTranslation.v[2]);
	}

	double speed = (int) ctx.calibrationSpeed;
	profile["calibration_speed"].set<double>(speed);

	if (ctx.chaperone.valid)
	{
		picojson::object chaperone;
		chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
		chaperone["play_space_size"].set<picojson::array>(FloatArray(ctx.chaperone.playSpaceSize.v, 2));

		chaperone["standing_center"].set<picojson::array>(FloatArray(
			(const float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		));

		chaperone["geometry"].set<picojson::array>(FloatArray(
			(const float *) ctx.chaperone.geometry.data(),
			static_cast<int>(sizeof(ctx.chaperone.geometry[0]) / sizeof(float) * ctx.chaperone.geometry.size())
		));

		profile["chaperone"].set<picojson::object>(chaperone);
	}

	picojson::value profileV;
	profileV.set<picojson::object>(profile);

	picojson::array profiles;
	profiles.push_back(profileV);

	picojson::value profilesV;
	profilesV.set<picojson::array>(profiles);

	out << profilesV.serialize(true);
}

bool DecodeProfile(const std::string &json, CalibrationContext &profile, std::string &error)
{
	try
	{
		CalibrationContext parsed;
		if (!json.empty())
		{
			std::istringstream stream(json);
			ParseProfile(parsed, stream);
		}
		else
			ClearCommittedProfile(parsed);
		parsed.state = profile.state;
		parsed.autoAcquire = profile.autoAcquire;
		parsed.timeLastScan = profile.timeLastScan;
		parsed.timeLastTick = profile.timeLastTick;
		parsed.messages = profile.messages;
		parsed.statusLine = profile.statusLine;
		parsed.notificationId = profile.notificationId;
		std::copy(std::begin(profile.devicePoses), std::end(profile.devicePoses), std::begin(parsed.devicePoses));
		profile = std::move(parsed);
		error.clear();
		return true;
	}
	catch (const std::exception &ex)
	{
		error = ex.what();
		return false;
	}
}

void PreserveChangedSettings(CalibrationContext &loaded, const CalibrationContext &current, const CalibrationContext &beforeRead)
{
	auto preserve = [](auto &loadedValue, const auto &currentValue, const auto &beforeValue) {
		if (currentValue != beforeValue)
			loadedValue = currentValue;
	};
	preserve(loaded.enableNative, current.enableNative, beforeRead.enableNative);
	preserve(loaded.fallbackToSlam, current.fallbackToSlam, beforeRead.fallbackToSlam);
	preserve(loaded.enableAngularVelocity, current.enableAngularVelocity, beforeRead.enableAngularVelocity);
	preserve(loaded.continuousSync, current.continuousSync, beforeRead.continuousSync);
	preserve(loaded.predictionTime, current.predictionTime, beforeRead.predictionTime);
	preserve(loaded.calibrationSpeed, current.calibrationSpeed, beforeRead.calibrationSpeed);
	preserve(loaded.headFilterEnabled, current.headFilterEnabled, beforeRead.headFilterEnabled);
	preserve(loaded.chaperone.autoApply, current.chaperone.autoApply, beforeRead.chaperone.autoApply);
	auto filter = [&](protocol::OneEuroParams &loadedFilter, const protocol::OneEuroParams &currentFilter, const protocol::OneEuroParams &beforeFilter) {
		preserve(loadedFilter.minCutoff, currentFilter.minCutoff, beforeFilter.minCutoff);
		preserve(loadedFilter.beta, currentFilter.beta, beforeFilter.beta);
		preserve(loadedFilter.dCutoff, currentFilter.dCutoff, beforeFilter.dCutoff);
	};
	filter(loaded.headFilterParams, current.headFilterParams, beforeRead.headFilterParams);
	filter(loaded.driftFilterParams, current.driftFilterParams, beforeRead.driftFilterParams);
}

std::string EncodeProfile(const CalibrationContext &profile)
{
	std::ostringstream stream;
	WriteProfile(profile, stream);
	return stream.str();
}

bool DecodeSettings(const std::string &json, bool &autoAcquire, std::string &error)
{
	if (json.empty())
	{
		autoAcquire = true;
		error.clear();
		return true;
	}
	picojson::value value;
	error = picojson::parse(value, json);
	if (!error.empty())
		return false;
	if (!value.is<picojson::object>())
	{
		error = "Expected a settings object.";
		return false;
	}
	const auto &object = value.get<picojson::object>();
	auto found = object.find("autoAcquire");
	autoAcquire = found == object.end() || !found->second.is<bool>() ? true : found->second.get<bool>();
	return true;
}

std::string EncodeSettings(bool autoAcquire)
{
	picojson::object settings;
	settings["autoAcquire"] = picojson::value(autoAcquire);
	return picojson::value(settings).serialize();
}

bool ApplyProfileRead(ReadResult result, const std::string &json, CalibrationContext &profile,
	const CalibrationContext &beforeRead, PersistenceState &storage, std::string &error)
{
	if (storage.resolved || result == ReadResult::Error)
		return false;
	CalibrationContext parsed = profile;
	if (DecodeProfile(result == ReadResult::Absent ? "" : json, parsed, error))
	{
		PreserveChangedSettings(parsed, profile, beforeRead);
		profile = std::move(parsed);
	}
	else
		ClearCommittedProfile(profile);
	storage.resolved = true;
	return true;
}

bool ApplySettingsRead(ReadResult result, const std::string &json, CalibrationContext &profile,
	PersistenceState &storage, std::string &error)
{
	if (storage.resolved || result == ReadResult::Error)
		return false;
	bool enabled = true;
	DecodeSettings(result == ReadResult::Absent ? "" : json, enabled, error);
	profile.autoAcquire = enabled;
	storage.resolved = true;
	return true;
}
