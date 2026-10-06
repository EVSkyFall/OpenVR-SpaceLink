// SPDX-License-Identifier: AGPL-3.0-only

#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "Version.h"

#include <string>
#include <vector>
#include <algorithm>
#include <imgui.h>

const ImGuiWindowFlags bareWindowFlags =
		ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar |
		ImGuiWindowFlags_NoScrollWithMouse;

const ImGuiWindowFlags modalWindowFlags =
		ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove;

void UserInterface::Render(bool runningInOverlay)
{
	auto textWithWidth = [](const char *label, const char *text, float width) {
		ImGui::BeginChild(label, ImVec2(width, ImGui::GetTextLineHeightWithSpacing()));
		ImGui::Text(text);
		ImGui::EndChild();
	};

	auto &io = ImGui::GetIO();
	ImGuiStyle &style = ImGui::GetStyle();

	auto statusText = [](const ImColor &color, const std::string &text) {
		ImGui::PushStyleColor(ImGuiCol_Text, color.Value);
		ImGui::TextWrapped("%s", text.c_str());
		ImGui::PopStyleColor();
	};

	auto progressBar = [&style](int progress, int target) {
		float fraction = target > 0 ? std::clamp((float)progress / (float)target, 0.0f, 1.0f) : 0.0f;
		ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor(0, 0, 0));
		ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), "");
		ImGui::PopStyleColor();
		if (target > 0)
		{
			ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetFontSize() - style.FramePadding.y * 2);
			ImGui::Text(" %d%%", (int)(fraction * 100));
		}
	};

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(io.DisplaySize);

	if (!ImGui::Begin("MainWindow", nullptr, bareWindowFlags))
	{
		ImGui::End();
		return;
	}

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_Button));

	if (ImGui::BeginTabBar("##tabs")) {

		if (ImGui::BeginTabItem("Calibration")) {

			VRState state;
			{
				auto &trackingSystems = state.trackingSystems;
				char buffer[vr::k_unMaxPropertyStringSize];

				for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
				{
					vr::ETrackedPropertyError err = vr::TrackedProp_Success;
					auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
					if (deviceClass == vr::TrackedDeviceClass_Invalid)
						continue;

					if (deviceClass != vr::TrackedDeviceClass_TrackingReference)
					{
						vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

						if (err == vr::TrackedProp_Success)
						{
							std::string system(buffer);
							auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
							if (existing != trackingSystems.end())
							{
								if (deviceClass == vr::TrackedDeviceClass_HMD)
								{
									trackingSystems.erase(existing);
									trackingSystems.insert(trackingSystems.begin(), system);
								}
							}
							else
							{
								trackingSystems.push_back(system);
							}

							VRDevice device;
							device.id = id;
							device.deviceClass = deviceClass;
							device.trackingSystem = system;

							vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
							device.model = std::string(buffer);

							vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
							device.serial = std::string(buffer);

							device.controllerRole = (vr::ETrackedControllerRole) vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &err);
							state.devices.push_back(device);
						}
						else
						{
							printf("failed to get tracking system name for id %d\n", id);
						}
					}
				}
			}

			const VRDevice *hmd = nullptr;
			const VRDevice *tracker = nullptr;
			for (auto &device : state.devices)
			{
				if (device.id == vr::k_unTrackedDeviceIndex_Hmd)
					hmd = &device;
				if (!CalCtx.trackerSerial.empty() && device.serial == CalCtx.trackerSerial)
					tracker = &device;
			}

			const ImColor gray(0.5f, 0.5f, 0.5f), green(0.2f, 0.7f, 0.2f), orange(0.9f, 0.6f, 0.1f);
			auto driverLink = GetDriverLinkStatus();
			auto headTracker = GetHeadTrackerState();
			auto acquire = GetAcquireStatus();

			if (!driverLink.connected)
			{
				std::string error = driverLink.lastError;
				error.erase(error.find_last_not_of(" \t\r\n") + 1);
				std::string text = "Waiting for the SpaceOverride driver.";
				if (!error.empty())
					text += " " + error;
				statusText(orange, text);
			}

			if (hmd)
				ImGui::Text("HMD: %s (%s)", hmd->serial.c_str(), hmd->trackingSystem.c_str());
			else
				ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "No HMD detected");

			if (headTracker != HeadTrackerState::Unbound)
			{
				const std::string &serial = CalCtx.trackerSerial;
				if (CalCtx.state == CalibrationState::WaitForTracker || CalCtx.state == CalibrationState::Sampling)
					statusText(green, "Calibrating head tracker " + serial + ".");
				else if (!CalCtx.validRelativeOffset)
					statusText(orange, "Head tracker " + serial + " needs calibration. Press Calibrate.");
				else if (headTracker == HeadTrackerState::Active)
				{
					std::string activeSerial = tracker ? tracker->serial : serial;
					std::string activeSystem = tracker ? tracker->trackingSystem : CalCtx.targetTrackingSystem;
					statusText(green, "Override active: HMD driven by " + activeSerial + (activeSystem.empty() ? "" : " (" + activeSystem + ")"));
				}
				else if (!driverLink.connected)
					statusText(gray, "Head tracker " + serial + " is bound. The override starts when the driver connects.");
				else if (CalCtx.fallbackToSlam)
					statusText(orange, "Head tracker " + serial + " is not tracking. The headset uses its own tracking until it is back.");
				else
					statusText(orange, "Head tracker " + serial + " is not tracking. Headset tracking is paused until it is back.");
			}
			else if (acquire.state == AcquireState::Off)
				statusText(gray, "No head tracker. Press Calibrate, then move your head to identify the headset tracker.");
			else
			{
				switch (acquire.state)
				{
				case AcquireState::NeedHands:
					statusText(gray, "Looking for your head tracker. Turn on hand tracking and hold your lighthouse controllers.");
					break;
				case AcquireState::Syncing:
					// handPairs counts synced pairs, so it can be 0 here.
					if (acquire.handPairs > 0)
						statusText(gray, "Hands found (" + std::to_string(acquire.handPairs) + "). Move your hands around a little.");
					else
						statusText(gray, "Hands found. Move your hands around a little.");
					break;
				case AcquireState::Searching:
					statusText(gray, "Hands synced. Looking for the tracker on your head. Look around naturally.");
					break;
				case AcquireState::Calibrating:
					statusText(green, "Found head tracker " + acquire.trackerSerial + ". Look around naturally to finish.");
					progressBar(acquire.progress, acquire.target);
					break;
				case AcquireState::ProfileUnreadable:
					statusText(orange, "Saved calibration could not be read yet. Retrying.");
					break;
				default:
					break;
				}
			}

			bool autoAcquire = CalCtx.autoAcquire;
			if (ImGui::Checkbox("Find head tracker automatically", &autoAcquire))
				SetAutoAcquire(autoAcquire);
			if (ImGui::BeginItemTooltip())
			{
				ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
				ImGui::TextUnformatted("Uses hand tracking and the lighthouse controllers in your hands to find the tracker on your headset, then calibrates it without a button. Turn this off to stop it.");
				ImGui::PopTextWrapPos();
				ImGui::EndTooltip();
			}

			ImGui::Text("");

			if (CalCtx.state == CalibrationState::None)
			{
				float buttonWidth = ImGui::GetContentRegionAvail().x;
				if (CalCtx.validProfile)
					buttonWidth = (buttonWidth - style.ItemSpacing.x * 2.0f) / 3.0f;

				if (ImGui::Button("Calibrate", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
				{
					ImGui::OpenPopup("Calibration Progress");
					StartCalibration();
				}

				if (CalCtx.validProfile)
				{
					ImGui::SameLine();
					if (ImGui::Button("Edit Calibration", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
					{
						CalCtx.state = CalibrationState::Editing;
					}

					ImGui::SameLine();
					if (ImGui::Button("Remove Calibration", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
					{
						RemoveCalibration();
					}
				}

				/*
				float chapWidth = ImGui::GetContentRegionAvail().x;
				if (CalCtx.chaperone.valid)
					chapWidth = (chapWidth - style.ItemSpacing.x) / 2.0f;

				ImGui::Text("");
				if (ImGui::Button("Copy Chaperone Bounds to profile", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
				{
					LoadChaperoneBounds();
					SaveProfile(CalCtx);
				}

				if (CalCtx.chaperone.valid)
				{
					ImGui::SameLine();
					if (ImGui::Button("Paste Chaperone Bounds", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
					{
						ApplyChaperoneBounds();
					}

					if (ImGui::Checkbox(" Paste Chaperone Bounds automatically when geometry resets", &CalCtx.chaperone.autoApply))
					{
						SaveProfile(CalCtx);
					}
				}

				ImGui::Text("");
				*/
			}
			else if (CalCtx.state == CalibrationState::Editing)
			{
				float width = ImGui::GetContentRegionAvail().x / 3.0f - style.FramePadding.x;
				float widthF = width - style.FramePadding.x;

				textWithWidth("YawLabel", "Yaw", width);
				ImGui::SameLine();
				textWithWidth("PitchLabel", "Pitch", width);
				ImGui::SameLine();
				textWithWidth("RollLabel", "Roll", width);

				ImGui::PushItemWidth(widthF);
				ImGui::InputDouble("##Yaw", &CalCtx.calibratedRotation(1), 0.1, 1.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Pitch", &CalCtx.calibratedRotation(2), 0.1, 1.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Roll", &CalCtx.calibratedRotation(0), 0.1, 1.0, "%.8f");

				textWithWidth("XLabel", "X", width);
				ImGui::SameLine();
				textWithWidth("YLabel", "Y", width);
				ImGui::SameLine();
				textWithWidth("ZLabel", "Z", width);

				ImGui::InputDouble("##X", &CalCtx.calibratedTranslation(0), 1.0, 10.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Y", &CalCtx.calibratedTranslation(1), 1.0, 10.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Z", &CalCtx.calibratedTranslation(2), 1.0, 10.0, "%.8f");

				textWithWidth("ScaleLabel", "Scale", width);
				ImGui::SameLine();
				textWithWidth("HmdScaleLabel", "HMD Scale", width);

				ImGui::InputDouble("##Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##HmdScale", &CalCtx.hmdScale, 0.0001, 0.01, "%.8f");
				ImGui::PopItemWidth();

				if (ImGui::Button("Save Profile", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
				{
					SaveProfile(CalCtx);
					CalCtx.state = CalibrationState::None;
				}
			}
			else if (acquire.state != AcquireState::Calibrating)
			{
				if (ImGui::Button("Calibration in progress...", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
					ImGui::OpenPopup("Calibration Progress");
			}

			float footerHeight = ImGui::GetTextLineHeightWithSpacing() * (runningInOverlay ? 2.0f : 1.0f);
			ImGui::SetCursorPos(ImVec2(10.0f, ImGui::GetWindowHeight() - footerHeight - style.WindowPadding.y));
			ImGui::BeginChild("##bottom_line", ImVec2(ImGui::GetWindowWidth() - 20.0f, footerHeight), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
			ImGui::Text("OpenVR-SpaceOverride v" SPACECAL_VERSION_STRING " - by Nyabsi (Special thanks to tach/pushrax for OpenVR-SpaceCalibrator)");
			if (runningInOverlay)
			{
				ImGui::Text("close VR overlay to use mouse");
			}
			ImGui::EndChild();

			ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f));
			ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x - 40.0f, io.DisplaySize.y - 40.0f));
			if (ImGui::BeginPopupModal("Calibration Progress", nullptr, modalWindowFlags))
			{
				// Calibrate runs also write their status into the log; show it once.
				bool statusLogged = std::any_of(CalCtx.messages.begin(), CalCtx.messages.end(), [](const CalibrationContext::Message &message) {
					return message.type == CalibrationContext::Message::String && message.str == CalCtx.statusLine;
				});
				if (!CalCtx.statusLine.empty() && !statusLogged)
					ImGui::TextWrapped("%s", CalCtx.statusLine.c_str());

				for (auto &message : CalCtx.messages)
				{
					switch (message.type)
					{
					case CalibrationContext::Message::String:
						ImGui::TextWrapped("%s", message.str.c_str());
						break;
					case CalibrationContext::Message::Progress:
						ImGui::Text("");
						progressBar(message.progress, message.target);
						break;
					}
				}

				ImGui::Text("");
				if (CalCtx.state != CalibrationState::None && CalCtx.state != CalibrationState::Editing)
				{
					if (ImGui::Button("Cancel", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
					{
						CancelCalibration();
						// Keep the popup until the run has actually stopped.
						if (CalCtx.state == CalibrationState::None)
							ImGui::CloseCurrentPopup();
					}
				}
				else if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
					ImGui::CloseCurrentPopup();

				ImGui::EndPopup();
			}

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Smoothing"))
		{
			ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "NOTE: Changes here take effect instantly, no need to re-calibrate.");
			ImGui::Spacing();
			ImGui::TextWrapped(
				"These settings smooth out tracking so your view and devices look steady instead of shaky. "
				"If something looks shaky, add more smoothing. If it feels laggy or floaty when you move, "
				"ease off. Not sure? Hover over a slider for tips.");
			ImGui::Spacing();

			const double cutoffMin = 0.1, cutoffMax = 5.0;
			const double betaMin = 0.0, betaMax = 2.0;

			auto paramSliders = [&](const char* id, protocol::OneEuroParams& p) {
				bool c = false;
				ImGui::PushID(id);

				ImGui::Text("minCutoff");
				ImGui::SameLine(170);
				ImGui::SetNextItemWidth(-1);
				c |= ImGui::SliderScalar("##minCutoff", ImGuiDataType_Double, &p.minCutoff, &cutoffMin, &cutoffMax, "%.3f Hz");
				ImGui::SetItemTooltip(
					"How steady things look when you are not moving.\n"
					"Drag left to remove shaking for a calmer image; drag right if things start to feel laggy or floaty."
				);

				ImGui::Text("beta");
				ImGui::SameLine(170);
				ImGui::SetNextItemWidth(-1);
				c |= ImGui::SliderScalar("##beta", ImGuiDataType_Double, &p.beta, &betaMin, &betaMax, "%.3f Hz");
				ImGui::SetItemTooltip(
					"How quickly tracking keeps up when you move fast.\n"
					"Drag right if fast movements feel delayed or laggy; drag left if they look shaky."
				);

				ImGui::Text("dCutoff");
				ImGui::SameLine(170);
				ImGui::SetNextItemWidth(-1);
				c |= ImGui::SliderScalar("##dCutoff", ImGuiDataType_Double, &p.dCutoff, &cutoffMin, &cutoffMax, "%.3f Hz");
				ImGui::SetItemTooltip(
					"Most people can leave this alone.\n"
					"It fine-tunes how the smoothing reacts as your movement speed changes."
				);

				ImGui::PopID();
				return c;
			};

			bool changed = false;

			changed |= ImGui::Checkbox("Smooth headset tracker", &CalCtx.headFilterEnabled);
			ImGui::SetItemTooltip("Steadies what you see through the headset to reduce shaking. This can add a tiny bit of delay, so if your view feels laggy when you move quickly, adjust the sliders below.");

			ImGui::Spacing();
			ImGui::SeparatorText("Headset Tracker");
			ImGui::BeginDisabled(!CalCtx.headFilterEnabled);
			changed |= paramSliders("head", CalCtx.headFilterParams);
			ImGui::EndDisabled();

			ImGui::Spacing();
			ImGui::SeparatorText("Relative Calibration");
			ImGui::TextWrapped(
				"Keeps your controllers and other tracked devices lined up with your real space, and "
				"steadies your view if the headset briefly loses tracking. Add more smoothing if they "
				"look shaky; ease off if they are slow to line up.");
			ImGui::Spacing();
			changed |= paramSliders("drift", CalCtx.driftFilterParams);

			if (changed)
				SendOneEuroParams();

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Settings"))
		{
			ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "NOTE: All settings below require re-calibration to be applied");
			ImGui::Spacing();
			ImGui::Text("Tip: hover over the settings to see additional information.");
			ImGui::Spacing();

			float halfWidth = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) * 0.5f;

			ImGui::BeginChild("##left_panel", ImVec2(halfWidth, 0), true);

			ImGui::Checkbox("Fallback to SLAM", &CalCtx.fallbackToSlam);
			ImGui::SetItemTooltip(
				"Temporarily uses HMD (SLAM) tracking if the headset tracker loses line of sight.");

			ImGui::Checkbox("Enable Angular Velocity", &CalCtx.enableAngularVelocity);
			ImGui::SetItemTooltip(
				"Enables angular velocity reporting, as it may cause issues with some devices, it is disabled by default.");

			ImGui::Checkbox("Relative Calibration", &CalCtx.continuousSync);
			ImGui::SetItemTooltip(
				"Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space\n"
				"in the background. The headset's raw SLAM pose is compared against the tracker-driven\n"
				"pose to measure SLAM drift, and the correction is applied gradually and automatically.");

			ImGui::Checkbox("Discard Calibrated Offset", &CalCtx.enableNative);
			ImGui::SetItemTooltip(
				"Discards all SLAM tracking (even as fallback) and feeds only raw tracker data with the offset applied.\n"
				"Downside: a yaw orientation mismatch may occur. This only works in Local tracking space, where\n"
				"re-centering adjusts the yaw - Stage tracking space never re-centers yaw, so the mismatch cannot\n"
				"be corrected there.\n"
				"Will not work on all devices - tested only on Pico.");

			ImGui::EndChild();

			ImGui::SameLine();

			ImGui::BeginChild("##right_panel", ImVec2(0, 0), true);

			ImGui::Text("Prediction Time");
			ImGui::SameLine();
			ImGui::SliderFloat("##prediction_time", &CalCtx.predictionTime, 0.0f, 10.0f, "%.1f");
			ImGui::SetItemTooltip(
				"How many frames of prediction SteamVR applies to the tracker.\n"
				"Some wireless solutions may need more prediction to feel smooth.");

			ImGui::Spacing();

			ImGui::Text("Calibration Speed");

			auto speed = CalCtx.calibrationSpeed;

			if (ImGui::RadioButton("Fast", speed == CalibrationContext::FAST))
				CalCtx.calibrationSpeed = CalibrationContext::FAST;

			if (ImGui::RadioButton("Slow", speed == CalibrationContext::SLOW))
				CalCtx.calibrationSpeed = CalibrationContext::SLOW;

			if (ImGui::RadioButton("Very Slow", speed == CalibrationContext::VERY_SLOW))
				CalCtx.calibrationSpeed = CalibrationContext::VERY_SLOW;

			ImGui::SetItemTooltip("Controls how long calibration deltas are collected.");

			ImGui::EndChild();

			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	ImGui::PopStyleColor();
	ImGui::End();
}
