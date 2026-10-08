// SPDX-License-Identifier: AGPL-3.0-only

#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "EmbeddedFiles.h"
#include "SystemFonts.h"
#include "Version.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <imgui.h>

using Page = UserInterface::Page;

namespace
{
	// Phosphor icon code points, as UTF-8 bytes so the source stays ASCII.
	namespace Icon
	{
		constexpr const char *VirtualReality = "\xEE\x9E\xB8";
		constexpr const char *UserFocus = "\xEE\x9B\xBC";
		constexpr const char *RadioButton = "\xEE\xAC\x88";
		constexpr const char *Crosshair = "\xEE\x87\x96";
		constexpr const char *CompassTool = "\xEE\xA8\x8E";
		constexpr const char *Hand = "\xEE\x8A\x98";
		constexpr const char *HandGrabbing = "\xEE\x95\xBC";
		constexpr const char *Link = "\xEE\x8B\xA2";
		constexpr const char *LinkBreak = "\xEE\x8B\xA4";
		constexpr const char *CheckCircle = "\xEE\x86\x84";
		constexpr const char *Check = "\xEE\x86\x82";
		constexpr const char *WarningCircle = "\xEE\x93\xA2";
		constexpr const char *Plugs = "\xEE\xAD\x96";
		constexpr const char *ArrowsClockwise = "\xEE\x82\x94";
		constexpr const char *ClockCounterClockwise = "\xEE\x86\xA0";
		constexpr const char *FloppyDisk = "\xEE\x89\x88";
		constexpr const char *PencilSimple = "\xEE\x8E\xB4";
		constexpr const char *Trash = "\xEE\x92\xA6";
		constexpr const char *GearSix = "\xEE\x89\xB2";
		constexpr const char *WaveSine = "\xEE\xAA\x9A";
		constexpr const char *Pulse = "\xEE\x80\x80";
		constexpr const char *PushPin = "\xEE\x8F\xA2";
		constexpr const char *Info = "\xEE\x8B\x8E";
		constexpr const char *X = "\xEE\x93\xB6";
	}

	constexpr float Pi = 3.14159265f;
	constexpr const char *ProgressPopup = "Calibration Progress";
	constexpr const char *Credits = "SpaceLink v" SPACECAL_VERSION_STRING
		" \xC2\xB7 Based on OpenVR-SpaceOverride by Nyabsi \xC2\xB7 Thanks to tach and pushrax for OpenVR-SpaceCalibrator";
	constexpr const char *OverlayHint = "Close the SteamVR dashboard to use the mouse.";

	// ---------------------------------------------------------------- Look

	// Palette in sRGB.
	namespace Color
	{
		constexpr uint32_t Base = 0x101317;
		constexpr uint32_t Sidebar = 0x0B0D10;
		constexpr uint32_t Card = 0x161A20;
		constexpr uint32_t Raised = 0x1E232B;
		constexpr uint32_t Line = 0x272D37;
		constexpr uint32_t Text = 0xE8ECF2;
		constexpr uint32_t Muted = 0x9AA3B2;
		constexpr uint32_t Faint = 0x5F6776;
		constexpr uint32_t Accent = 0x3D9BF5;
		constexpr uint32_t AccentDeep = 0x2F6FE4;
		constexpr uint32_t White = 0xFFFFFF;
		constexpr uint32_t Good = 0x3DD68C;
		constexpr uint32_t Warn = 0xF5B13D;
		constexpr uint32_t Bad = 0xF0616D;
	}

	// Sizes are for the 1200 x 800 window, which is also what the SteamVR dashboard shows.
	constexpr float TextSize = 19.0f, SmallSize = 17.0f, LabelSize = 14.0f, HeadingSize = 28.0f, TitleSize = 30.0f;
	constexpr float CardRadius = 16.0f, ControlRadius = 10.0f, RowPadding = 16.0f;

	struct Fonts
	{
		ImFont *regular = nullptr, *medium = nullptr, *semibold = nullptr, *mono = nullptr, *icons = nullptr;
	} Font;

	float ToLinear(float c)
	{
		return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
	}

	ImVec4 Srgb(uint32_t rgb, float alpha = 1.0f)
	{
		return ImVec4(((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f, alpha);
	}

	// Colors are written in sRGB. The overlay draws into an sRGB target, which expects linear values.
	ImU32 Col(uint32_t rgb, float alpha = 1.0f)
	{
		ImVec4 c = Srgb(rgb, alpha);
		if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_IsSRGB)
			c = ImVec4(ToLinear(c.x), ToLinear(c.y), ToLinear(c.z), c.w);
		return ImGui::ColorConvertFloat4ToU32(c);
	}

	ImU32 Alpha(ImU32 color, float alpha)
	{
		ImU32 a = static_cast<ImU32>(((color >> IM_COL32_A_SHIFT) & 0xFF) * std::clamp(alpha, 0.0f, 1.0f) + 0.5f);
		return (color & ~IM_COL32_A_MASK) | (a << IM_COL32_A_SHIFT);
	}

	ImU32 Mix(ImU32 a, ImU32 b, float t)
	{
		auto channel = [&](int shift) {
			float from = static_cast<float>((a >> shift) & 0xFF), to = static_cast<float>((b >> shift) & 0xFF);
			return static_cast<ImU32>(from + (to - from) * t + 0.5f) << shift;
		};
		return channel(IM_COL32_R_SHIFT) | channel(IM_COL32_G_SHIFT) | channel(IM_COL32_B_SHIFT) | channel(IM_COL32_A_SHIFT);
	}

	enum class Tone { Idle, Busy, Good, Warn, Bad };

	ImU32 ToneColor(Tone tone)
	{
		switch (tone)
		{
		case Tone::Busy: return Col(Color::Accent);
		case Tone::Good: return Col(Color::Good);
		case Tone::Warn: return Col(Color::Warn);
		case Tone::Bad: return Col(Color::Bad);
		default: return Col(Color::Muted);
		}
	}

	// ---------------------------------------------------------------- State

	struct Snapshot
	{
		DriverLinkStatus driver;
		HeadTrackerState headTracker = HeadTrackerState::Unbound;
		AcquireStatus acquire;
		SpaceRestoreStatus restore;
		VRState vr;
		const VRDevice *hmd = nullptr;
	};

	struct Status
	{
		Tone tone = Tone::Idle;
		const char *icon = Icon::RadioButton;
		std::string word, title, detail, note;
		float progress = -1.0f;
		// Current step of the automatic setup (Hands, Sync, Find, Calibrate, Linked); 5 when done, -1 when not on it.
		int step = -1;
		bool bound = false, calibrating = false;
	};

	struct Context
	{
		Snapshot snap;
		Status status;
		Page *page = nullptr;
		bool *openProgress = nullptr;
		bool overlay = false;
	};

	const VRDevice *FindDevice(const Snapshot &snap, const std::string &serial)
	{
		if (serial.empty())
			return nullptr;
		for (const auto &device : snap.vr.devices)
			if (device.serial == serial)
				return &device;
		return nullptr;
	}

	float LatestProgress()
	{
		for (auto it = CalCtx.messages.rbegin(); it != CalCtx.messages.rend(); ++it)
			if (it->type == CalibrationContext::Message::Progress && it->target > 0)
				return std::clamp(static_cast<float>(it->progress) / it->target, 0.0f, 1.0f);
		return -1.0f;
	}

	Status Describe(const Snapshot &snap)
	{
		Status s;
		const std::string &serial = CalCtx.trackerSerial;
		const bool running = CalCtx.state == CalibrationState::WaitForTracker || CalCtx.state == CalibrationState::Sampling;
		const bool attempt = running || CalCtx.state == CalibrationState::Begin || CalCtx.state == CalibrationState::Detect;
		s.bound = snap.headTracker != HeadTrackerState::Unbound;
		s.calibrating = attempt;
		if (snap.acquire.state == AcquireState::Calibrating && snap.acquire.target > 0)
			s.progress = std::clamp(static_cast<float>(snap.acquire.progress) / snap.acquire.target, 0.0f, 1.0f);
		else if (attempt)
			s.progress = LatestProgress();

		if (s.bound)
		{
			if (running && snap.acquire.tiltRecalibration)
			{
				s.tone = Tone::Busy;
				s.icon = Icon::ArrowsClockwise;
				s.word = "Recalibrating";
				s.title = "Recalibrating";
				s.detail = "The head tracker moved on the headset. Look around naturally for a few seconds.";
				s.note = CalCtx.statusLine;
			}
			else if (running)
			{
				s.tone = Tone::Busy;
				s.icon = Icon::Crosshair;
				s.word = "Calibrating";
				s.title = "Calibrating";
				s.detail = CalCtx.statusLine.empty() ? "Calibrating head tracker " + serial + "." : CalCtx.statusLine;
			}
			else if (!CalCtx.validRelativeOffset)
			{
				s.tone = Tone::Warn;
				s.icon = Icon::WarningCircle;
				s.word = "Needs calibration";
				s.title = "Needs calibration";
				s.detail = "Head tracker " + serial + " needs calibration. Press Calibrate.";
			}
			else if (snap.headTracker == HeadTrackerState::Active)
			{
				s.tone = Tone::Good;
				s.icon = Icon::Link;
				s.word = "Linked";
				s.title = "Linked";
				s.detail = "Your headset follows head tracker " + serial + ".";
				s.step = 5;
			}
			else if (!snap.driver.connected)
			{
				s.icon = Icon::Plugs;
				s.word = "Waiting";
				s.title = "Starts when the driver connects";
				s.detail = "Head tracker " + serial + " is saved. Your headset switches to it as soon as the driver connects.";
			}
			// A tracker missing from this session keeps the override off, so the headset keeps its own tracking.
			else if (!CalCtx.fallbackToSlam && CalCtx.targetID < vr::k_unMaxTrackedDeviceCount)
			{
				s.tone = Tone::Warn;
				s.icon = Icon::LinkBreak;
				s.word = "Paused";
				s.title = "Head tracker not tracking";
				s.detail = "Head tracker " + serial + " is not tracking. Headset tracking is paused until it is back.";
			}
			else
			{
				s.tone = Tone::Warn;
				s.icon = Icon::LinkBreak;
				s.word = "Not tracking";
				s.title = "Head tracker not tracking";
				s.detail = "Head tracker " + serial + " is not tracking. The headset uses its own tracking until it is back.";
			}
			return s;
		}

		switch (snap.acquire.state)
		{
		case AcquireState::Off:
			s.icon = Icon::RadioButton;
			s.word = "Not linked";
			s.title = "No head tracker yet";
			s.detail = "Press Calibrate, then move your head to identify the head tracker.";
			s.note = "Automatic search is off. You can turn it on in Settings.";
			break;
		case AcquireState::NeedHands:
			s.tone = Tone::Busy;
			s.icon = Icon::Hand;
			s.word = "Searching";
			s.title = "Looking for your head tracker";
			s.detail = "Turn on hand tracking and hold your lighthouse controllers.";
			s.step = 0;
			break;
		case AcquireState::Syncing:
			s.tone = Tone::Busy;
			s.icon = Icon::HandGrabbing;
			s.word = "Searching";
			s.title = "Looking for your head tracker";
			s.detail = "Hands found. Move your hands around a little.";
			s.step = 1;
			break;
		case AcquireState::Searching:
			s.tone = Tone::Busy;
			s.icon = Icon::UserFocus;
			s.word = "Searching";
			s.title = "Looking for your head tracker";
			s.detail = "Hands synced. Look around naturally so SpaceLink can spot the tracker on your head.";
			s.step = 2;
			break;
		case AcquireState::Calibrating:
			s.tone = Tone::Busy;
			s.icon = Icon::Crosshair;
			s.word = "Calibrating";
			s.title = "Head tracker found";
			s.detail = "Found head tracker " + snap.acquire.trackerSerial + ". Look around naturally to finish.";
			s.step = 3;
			break;
		case AcquireState::ProfileUnreadable:
			s.tone = Tone::Warn;
			s.icon = Icon::ArrowsClockwise;
			s.word = "Retrying";
			s.title = "Reading your saved calibration";
			s.detail = "Saved calibration could not be read yet. Retrying.";
			break;
		default:
			s.tone = Tone::Busy;
			s.icon = Icon::Crosshair;
			s.word = "Calibrating";
			s.title = "Calibrating";
			s.detail = CalCtx.statusLine.empty() ? "Calibration in progress." : CalCtx.statusLine;
			break;
		}
		return s;
	}

	std::string Percent(float fraction)
	{
		return std::to_string(static_cast<int>(fraction * 100.0f)) + "%";
	}

	std::string Trimmed(std::string text)
	{
		text.erase(text.find_last_not_of(" \t\r\n") + 1);
		return text;
	}

	struct DeviceInfo
	{
		const char *icon, *role;
		std::string name, serial, system, state;
		Tone tone;
	};

	DeviceInfo HeadsetInfo(const Snapshot &snap)
	{
		if (!snap.hmd)
			return { Icon::VirtualReality, "Headset", "No headset detected", "", "", "Missing", Tone::Bad };
		bool tracking = CalCtx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid;
		return { Icon::VirtualReality, "Headset", snap.hmd->model.empty() ? std::string("Headset") : snap.hmd->model,
			snap.hmd->serial, snap.hmd->trackingSystem, tracking ? "Tracking" : "Not tracking", tracking ? Tone::Good : Tone::Warn };
	}

	DeviceInfo TrackerInfo(const Context &ctx)
	{
		const auto &snap = ctx.snap;
		std::string serial = CalCtx.trackerSerial.empty() ? snap.acquire.trackerSerial : CalCtx.trackerSerial;
		if (serial.empty() && ctx.status.calibrating)
			return { Icon::RadioButton, "Head tracker", "Not found yet", "", "", "Identifying", Tone::Busy };
		if (serial.empty())
			return { Icon::RadioButton, "Head tracker", "Not found yet", "", "", CalCtx.autoAcquire ? "Searching" : "Not set", CalCtx.autoAcquire ? Tone::Busy : Tone::Idle };
		const VRDevice *device = FindDevice(snap, serial);
		DeviceInfo info{ Icon::RadioButton, "Head tracker", device && !device->model.empty() ? device->model : std::string("Head tracker"), serial,
			device ? device->trackingSystem : CalCtx.targetTrackingSystem, "", Tone::Idle };
		if (ctx.status.calibrating)
			info.state = "Calibrating", info.tone = Tone::Busy;
		else if (snap.headTracker == HeadTrackerState::Active)
			info.state = "Tracking", info.tone = Tone::Good;
		else if (snap.headTracker == HeadTrackerState::Waiting)
			info.state = "Not tracking", info.tone = Tone::Warn;
		else
			info.state = "Found", info.tone = Tone::Busy;
		return info;
	}

	struct RestoreView
	{
		Tone tone;
		const char *title;
		std::string text, detail;
	};

	// What became of the alignment saved at the end of the last session.
	RestoreView DescribeRestore(const SpaceRestoreStatus &restore)
	{
		char amount[96];
		snprintf(amount, sizeof amount, "%.1f\xC2\xB0 of yaw and %.2f m between the headset's tracking space and your lighthouse space.",
			restore.yawDegrees, restore.translationMeters);
		const std::string &result = restore.result;
		if (result.empty())
			return { Tone::Busy, "Restores at start", "When VR starts, SpaceLink lines your spaces up the way they were last time.", "" };
		if (result == "applied")
			return { Tone::Good, "Restored at start", "VR started with your spaces lined up the way they were last session.",
				std::string("Restored alignment: ") + amount };
		if (result == "same-session")
			return { Tone::Good, "Still aligned", "SteamVR kept running, so the alignment from earlier this session still holds.",
				std::string("Current alignment: ") + amount };
		if (result == "absent" || result == "not-bound")
			return { Tone::Busy, "Saved for next time", "Nothing was saved from an earlier session yet. When VR closes, this alignment is kept for the next start.", "" };
		if (result == "basis-mismatch")
			return { Tone::Busy, "Lining up fresh", "The saved alignment was from an older calibration, so SpaceLink lines up again once the tracker tracks.", "" };
		if (result == "profile-pending")
			return { Tone::Busy, "Restoring", "The last alignment comes back once your saved calibration is read.", "" };
		return { Tone::Warn, "Not restored this time", "SpaceLink lines your spaces up again once the tracker tracks.", "" };
	}

	const char *const StepTitles[] = { "Hands", "Sync", "Find tracker", "Calibrate", "Linked" };
	const char *const StepIcons[] = { Icon::Hand, Icon::HandGrabbing, Icon::UserFocus, Icon::Crosshair, Icon::Link };

	std::string StepHint(int index, const Context &ctx)
	{
		const int pairs = ctx.snap.acquire.handPairs;
		switch (index)
		{
		case 0: return "Hold your controllers";
		case 1: return pairs > 0 && ctx.status.step > 1 ? std::to_string(pairs) + (pairs == 1 ? " pair synced" : " pairs synced") : "Move your hands";
		case 2: return "Look around";
		case 3: return ctx.status.step == 3 && ctx.status.progress >= 0.0f ? Percent(ctx.status.progress) + " collected" : "Keep looking around";
		default: return "Headset follows it";
		}
	}

	// ---------------------------------------------------------------- Drawing

	struct Rect
	{
		float x = 0, y = 0, w = 0, h = 0;
		ImVec2 Min() const { return ImVec2(x, y); }
		ImVec2 Max() const { return ImVec2(x + w, y + h); }
		ImVec2 Center() const { return ImVec2(x + w * 0.5f, y + h * 0.5f); }
		float Right() const { return x + w; }
		float Bottom() const { return y + h; }
	};

	ImDrawList *Canvas() { return ImGui::GetWindowDrawList(); }

	ImVec2 Measure(ImFont *font, float size, const char *text, float wrap = 0.0f)
	{
		return font->CalcTextSizeA(size, FLT_MAX, wrap, text);
	}

	ImVec2 Measure(ImFont *font, float size, const std::string &text, float wrap = 0.0f)
	{
		return Measure(font, size, text.c_str(), wrap);
	}

	float Text(ImFont *font, float size, ImVec2 pos, ImU32 color, const std::string &text, float wrap = 0.0f)
	{
		Canvas()->AddText(font, size, ImVec2(std::floor(pos.x), std::floor(pos.y)), color, text.c_str(), nullptr, wrap);
		return Measure(font, size, text, wrap).y;
	}

	void TextRight(ImFont *font, float size, ImVec2 rightTop, ImU32 color, const std::string &text)
	{
		Text(font, size, ImVec2(rightTop.x - Measure(font, size, text).x, rightTop.y), color, text);
	}

	void TextCentered(ImFont *font, float size, ImVec2 center, ImU32 color, const std::string &text)
	{
		ImVec2 s = Measure(font, size, text);
		Text(font, size, ImVec2(center.x - s.x * 0.5f, center.y - s.y * 0.5f), color, text);
	}

	// Wraps text to `wrap` and centers each line on `centerX`; returns the height.
	float TextBlockCentered(ImFont *font, float size, float centerX, float y, ImU32 color, const std::string &text, float wrap)
	{
		const char *start = text.c_str(), *end = start + text.size();
		float top = y;
		while (start < end)
		{
			const char *lineEnd = font->CalcWordWrapPosition(size, start, end, wrap);
			if (lineEnd == start)
				lineEnd = start + 1;
			const char *visible = lineEnd;
			while (visible > start && (visible[-1] == ' ' || visible[-1] == '\n'))
				--visible;
			float width = font->CalcTextSizeA(size, FLT_MAX, 0.0f, start, visible).x;
			Canvas()->AddText(font, size, ImVec2(std::floor(centerX - width * 0.5f), std::floor(y)), color, start, visible);
			y += size;
			start = lineEnd;
			while (start < end && (*start == ' ' || *start == '\n'))
				++start;
		}
		return y - top;
	}

	// Letter-spaced capitals; only used for ASCII labels.
	float Caps(ImFont *font, float size, ImVec2 pos, ImU32 color, const std::string &text, float tracking)
	{
		float x = std::floor(pos.x);
		for (char ch : text)
		{
			char glyph[2] = { static_cast<char>(std::toupper(static_cast<unsigned char>(ch))), 0 };
			Canvas()->AddText(font, size, ImVec2(x, std::floor(pos.y)), color, glyph);
			x += font->CalcTextSizeA(size, FLT_MAX, 0.0f, glyph).x + tracking;
		}
		return text.empty() ? 0.0f : x - std::floor(pos.x) - tracking;
	}

	void IconAt(ImVec2 center, float size, ImU32 color, const char *icon)
	{
		ImVec2 s = Measure(Font.icons, size, icon);
		Canvas()->AddText(Font.icons, size, ImVec2(std::floor(center.x - s.x * 0.5f + 0.5f), std::floor(center.y - s.y * 0.5f + 0.5f)), color, icon);
	}

	void Fill(const Rect &r, ImU32 color, float rounding = 0.0f)
	{
		Canvas()->AddRectFilled(r.Min(), r.Max(), color, rounding);
	}

	void Outline(const Rect &r, ImU32 color, float rounding = 0.0f, float thickness = 1.0f)
	{
		Canvas()->AddRect(r.Min(), r.Max(), color, rounding, 0, thickness);
	}

	// Recolors the vertices added since `start` along p0 -> p1, keeping their coverage alpha.
	void Shade(int start, ImVec2 p0, ImVec2 p1, ImU32 c0, ImU32 c1)
	{
		ImDrawList *list = Canvas();
		ImVec2 d(p1.x - p0.x, p1.y - p0.y);
		float length2 = std::max(d.x * d.x + d.y * d.y, 1e-6f);
		for (int i = start; i < list->VtxBuffer.Size; ++i)
		{
			ImDrawVert &v = list->VtxBuffer[i];
			float t = std::clamp(((v.pos.x - p0.x) * d.x + (v.pos.y - p0.y) * d.y) / length2, 0.0f, 1.0f);
			ImU32 color = Mix(c0, c1, t);
			ImU32 alpha = (((v.col >> IM_COL32_A_SHIFT) & 0xFF) * ((color >> IM_COL32_A_SHIFT) & 0xFF) + 127) / 255;
			v.col = (color & ~IM_COL32_A_MASK) | (alpha << IM_COL32_A_SHIFT);
		}
	}

	void FillGradient(const Rect &r, ImU32 from, ImU32 to, float rounding)
	{
		int start = Canvas()->VtxBuffer.Size;
		Fill(r, IM_COL32_WHITE, rounding);
		Shade(start, r.Min(), ImVec2(r.x, r.Bottom()), from, to);
	}

	void Shadow(const Rect &r, float rounding, float size, ImU32 color)
	{
		const int layers = 10;
		for (int i = layers; i >= 1; --i)
		{
			float d = size * i / layers;
			Fill({ r.x - d, r.y - d + size * 0.4f, r.w + d * 2, r.h + d * 2 }, Alpha(color, 1.0f / layers), rounding + d);
		}
	}

	void Arc(ImVec2 center, float radius, float from, float to, ImU32 color, float thickness)
	{
		if (to - from < 0.001f)
			return;
		ImDrawList *list = Canvas();
		int segments = std::max(12, static_cast<int>((to - from) * radius / 4.0f));
		list->PathArcTo(center, radius, from, to, segments);
		list->PathStroke(color, ImDrawFlags_None, thickness);
		list->AddCircleFilled(ImVec2(center.x + std::cos(from) * radius, center.y + std::sin(from) * radius), thickness * 0.5f, color, 20);
		list->AddCircleFilled(ImVec2(center.x + std::cos(to) * radius, center.y + std::sin(to) * radius), thickness * 0.5f, color, 20);
	}

	void Spinner(ImVec2 center, float radius, ImU32 color, float thickness)
	{
		float t = static_cast<float>(ImGui::GetTime()) * 3.6f;
		Arc(center, radius, t, t + 1.7f, color, thickness);
	}

	void Pulse(ImVec2 center, float radius, float spread, ImU32 color)
	{
		float t = static_cast<float>(ImGui::GetTime());
		for (int i = 0; i < 3; ++i)
		{
			float phase = std::fmod(t * 0.5f + i / 3.0f, 1.0f);
			Canvas()->AddCircle(center, radius + spread * phase, Alpha(color, (1.0f - phase) * 0.55f), 96, 2.0f);
		}
	}

	// ---------------------------------------------------------------- Widgets

	struct Hit
	{
		bool clicked = false, hovered = false, held = false;
	};

	Hit Interact(const char *id, const Rect &r, bool enabled = true)
	{
		ImGui::SetCursorScreenPos(r.Min());
		ImGui::BeginDisabled(!enabled);
		Hit hit;
		hit.clicked = ImGui::InvisibleButton(id, ImVec2(std::max(r.w, 1.0f), std::max(r.h, 1.0f)));
		hit.hovered = ImGui::IsItemHovered();
		hit.held = ImGui::IsItemActive();
		ImGui::EndDisabled();
		if (hit.hovered)
			ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
		return hit;
	}

	float Animate(const char *id, float target, float speed = 10.0f)
	{
		float *value = ImGui::GetStateStorage()->GetFloatRef(ImGui::GetID((std::string(id) + "#anim").c_str()), target);
		float step = ImGui::GetIO().DeltaTime * speed;
		*value = *value < target ? std::min(target, *value + step) : std::max(target, *value - step);
		return *value;
	}

	void Tooltip(const std::string &text)
	{
		if (ImGui::BeginItemTooltip())
		{
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
			ImGui::TextUnformatted(text.c_str());
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
	}

	// A tooltip over an area that does nothing when clicked.
	void TooltipArea(const char *id, const Rect &r, const std::string &text)
	{
		ImGui::SetCursorScreenPos(r.Min());
		ImGui::InvisibleButton(id, ImVec2(r.w, r.h));
		Tooltip(text);
	}

	enum class Kind { Primary, Secondary, Danger };

	bool Button(const char *id, const Rect &r, const char *label, const char *icon = nullptr, Kind kind = Kind::Secondary)
	{
		Hit hit = Interact(id, r);
		ImU32 fill, border = 0, ink = Col(Color::Text);
		if (kind == Kind::Primary)
		{
			fill = hit.held ? Col(Color::AccentDeep) : Mix(Col(Color::Accent), Col(Color::White), hit.hovered ? 0.1f : 0.0f);
			ink = Col(Color::White);
		}
		else if (kind == Kind::Danger)
		{
			fill = Col(Color::Bad, hit.hovered ? 0.14f : 0.0f);
			border = Col(Color::Bad, 0.45f);
			ink = Col(Color::Bad);
		}
		else
		{
			fill = Col(hit.hovered ? Color::Line : Color::Raised);
			border = Col(Color::Line);
		}
		if (fill & IM_COL32_A_MASK)
			Fill(r, fill, ControlRadius);
		if (border & IM_COL32_A_MASK)
			Outline(r, border, ControlRadius);

		const float iconSize = TextSize * 1.15f, gap = icon ? 10.0f : 0.0f;
		const float total = Measure(Font.medium, TextSize, label).x + (icon ? iconSize + gap : 0.0f);
		float x = std::floor(r.Center().x - total * 0.5f), cy = r.Center().y;
		if (icon)
		{
			IconAt(ImVec2(x + iconSize * 0.5f, cy), iconSize, ink, icon);
			x += iconSize + gap;
		}
		Text(Font.medium, TextSize, ImVec2(x, cy - TextSize * 0.5f), ink, label);
		return hit.clicked;
	}

	const ImVec2 SwitchSize(46.0f, 26.0f);

	// t runs from 0 (off) to 1 (on) while the knob slides.
	void Switch(ImVec2 pos, float t, bool hovered, bool enabled)
	{
		Rect r{ pos.x, pos.y, SwitchSize.x, SwitchSize.y };
		const float radius = SwitchSize.y * 0.5f;
		Fill(r, Mix(Col(hovered ? Color::Line : Color::Raised), Col(Color::Accent), t), radius);
		if (t < 1.0f)
			Outline(r, Col(Color::Line, 1.0f - t), radius);
		if (!enabled)
			Fill(r, Col(Color::Card, 0.6f), radius);
		const float knob = radius - 3.0f, dim = enabled ? 1.0f : 0.4f;
		ImVec2 center(r.x + radius + (SwitchSize.x - SwitchSize.y) * t, r.Center().y);
		Canvas()->AddCircleFilled(ImVec2(center.x, center.y + 1.0f), knob + 0.5f, Col(0x000000, 0.25f * dim), 32);
		Canvas()->AddCircleFilled(center, knob, Col(Color::White, enabled ? 1.0f : 0.5f), 32);
	}

	// A setting with a title and description; the whole row toggles. Returns the row height.
	float ToggleRow(const char *id, float x, float y, float w, const char *title, const char *description, bool &value, bool &changed, bool enabled = true)
	{
		const float textW = w - RowPadding * 3.0f - SwitchSize.x;
		const float descH = description ? Measure(Font.regular, SmallSize, description, textW).y : 0.0f;
		const float h = RowPadding + TextSize + (description ? 6.0f + descH : 0.0f) + RowPadding;
		Rect r{ x, y, w, h };
		Hit hit = Interact(id, r, enabled);
		if (hit.clicked)
		{
			value = !value;
			changed = true;
		}
		if (hit.hovered)
			Fill(r, Col(Color::Raised, 0.6f), ControlRadius);
		float t = Animate(id, value ? 1.0f : 0.0f);
		float dim = enabled ? 1.0f : 0.45f;
		Text(Font.medium, TextSize, ImVec2(x + RowPadding, y + RowPadding), Col(Color::Text, dim), title);
		if (description)
			Text(Font.regular, SmallSize, ImVec2(x + RowPadding, y + RowPadding + TextSize + 6.0f), Col(Color::Muted, dim), description, textW);
		Switch(ImVec2(x + w - RowPadding - SwitchSize.x, y + RowPadding + TextSize * 0.5f - SwitchSize.y * 0.5f), t, hit.hovered, enabled);
		return h;
	}

	bool Segmented(const char *id, const Rect &r, const char *const *labels, int count, int &selected)
	{
		bool changed = false;
		Fill(r, Col(Color::Raised), ControlRadius);
		const float inset = 4.0f, segment = (r.w - inset * 2.0f) / count, inner = ControlRadius - inset;
		ImGui::PushID(id);
		for (int i = 0; i < count; ++i)
		{
			Rect s{ r.x + inset + segment * i, r.y + inset, segment, r.h - inset * 2.0f };
			Hit hit = Interact((std::string("##") + labels[i]).c_str(), s);
			if (hit.clicked && selected != i)
			{
				selected = i;
				changed = true;
			}
			const bool on = selected == i;
			if (on)
			{
				Fill(s, Col(Color::Card), inner);
				Outline(s, Col(Color::Line), inner);
			}
			else if (hit.hovered)
				Fill(s, Col(Color::Line, 0.6f), inner);
			TextCentered(Font.medium, SmallSize + 1.0f, s.Center(), Col(on ? Color::Text : Color::Muted), labels[i]);
		}
		ImGui::PopID();
		return changed;
	}

	bool Slider(const char *id, const Rect &r, double &value, double min, double max, double step, bool enabled = true)
	{
		Hit hit = Interact(id, r, enabled);
		const float knob = 10.0f, track = 6.0f;
		const float x0 = r.x + knob, x1 = r.Right() - knob;
		bool changed = false;
		if (hit.held && enabled)
		{
			float t = std::clamp((ImGui::GetIO().MousePos.x - x0) / (x1 - x0), 0.0f, 1.0f);
			double v = std::clamp(std::round((min + t * (max - min)) / step) * step, min, max);
			if (v != value)
			{
				value = v;
				changed = true;
			}
		}
		const float t = static_cast<float>(std::clamp((value - min) / (max - min), 0.0, 1.0));
		const float cy = std::floor(r.Center().y) + 0.5f, kx = x0 + (x1 - x0) * t;
		const float dim = enabled ? 1.0f : 0.4f;
		Rect bar{ x0 - track * 0.5f, cy - track * 0.5f, x1 - x0 + track, track };
		Fill(bar, Col(Color::Raised, dim), track * 0.5f);
		Fill({ bar.x, bar.y, kx - bar.x + track * 0.5f, track }, Col(Color::Accent, dim), track * 0.5f);
		if (hit.hovered || hit.held)
			Canvas()->AddCircleFilled(ImVec2(kx, cy), knob + 6.0f, Col(Color::Accent, 0.18f), 32);
		Canvas()->AddCircleFilled(ImVec2(kx, cy + 1.0f), knob + 0.5f, Col(0x000000, 0.3f * dim), 32);
		Canvas()->AddCircleFilled(ImVec2(kx, cy), knob, Col(Color::White, enabled ? 1.0f : 0.5f), 32);
		return changed;
	}

	void ProgressBar(const Rect &r, float fraction)
	{
		fraction = std::clamp(fraction, 0.0f, 1.0f);
		Fill(r, Col(Color::Raised), r.h * 0.5f);
		if (fraction > 0.0f)
			Fill({ r.x, r.y, std::max(r.h, r.w * fraction), r.h }, Col(Color::Accent), r.h * 0.5f);
	}

	void Card(const Rect &r)
	{
		Fill(r, Col(Color::Card), CardRadius);
		Outline(r, Col(Color::Line), CardRadius);
	}

	// Draws content first and the card behind it once its height is known.
	struct CardScope
	{
		CardScope() { Canvas()->ChannelsSplit(2); Canvas()->ChannelsSetCurrent(1); }
		void End(const Rect &r) { Canvas()->ChannelsSetCurrent(0); Card(r); Canvas()->ChannelsMerge(); }
	};

	float SectionLabel(float x, float y, const char *text)
	{
		Caps(Font.medium, LabelSize, ImVec2(x, y), Col(Color::Muted), text, 1.3f);
		return LabelSize;
	}

	float ChipWidth(const std::string &word)
	{
		return Measure(Font.medium, LabelSize + 1.0f, word).x + 34.0f;
	}

	void Chip(float x, float y, Tone tone, const std::string &word)
	{
		const ImU32 color = ToneColor(tone);
		const float size = LabelSize + 1.0f;
		Rect r{ x, y, ChipWidth(word), size + 14.0f };
		Fill(r, Alpha(color, 0.16f), r.h * 0.5f);
		Canvas()->AddCircleFilled(ImVec2(r.x + 15.0f, r.Center().y), 4.0f, color, 16);
		Text(Font.medium, size, ImVec2(r.x + 26.0f, r.Center().y - size * 0.5f), color, word);
	}

	// ---------------------------------------------------------------- Shared sections

	float DriverAlert(const Context &ctx, float x, float y, float w)
	{
		if (ctx.snap.driver.connected)
			return 0.0f;
		const std::string error = Trimmed(ctx.snap.driver.lastError);
		const float pad = 16.0f, iconW = 40.0f;
		const float textW = w - pad * 2.0f - iconW;
		const float errorH = error.empty() ? 0.0f : 4.0f + Measure(Font.regular, SmallSize, error, textW).y;
		Rect r{ x, y, w, pad * 2.0f + TextSize + errorH };
		Fill(r, Col(Color::Warn, 0.1f), ControlRadius);
		Outline(r, Col(Color::Warn, 0.4f), ControlRadius);
		IconAt(ImVec2(r.x + pad + 13.0f, r.y + pad + TextSize * 0.5f), TextSize * 1.2f, Col(Color::Warn), Icon::Plugs);
		Text(Font.medium, TextSize, ImVec2(r.x + pad + iconW, r.y + pad), Col(Color::Text), "Waiting for the SpaceLink driver");
		if (!error.empty())
			Text(Font.regular, SmallSize, ImVec2(r.x + pad + iconW, r.y + pad + TextSize + 4.0f), Col(Color::Muted), error, textW);
		return r.h;
	}

	enum class ActionId { Calibrate, Edit, Remove, ShowProgress };

	struct Action
	{
		ActionId id;
		const char *label, *icon;
		Kind kind;
	};

	std::vector<Action> Actions(const Context &ctx)
	{
		if (CalCtx.state == CalibrationState::None)
		{
			std::vector<Action> list{ { ActionId::Calibrate, "Calibrate", Icon::Crosshair, Kind::Primary } };
			if (CalCtx.validProfile)
			{
				list.push_back({ ActionId::Edit, "Edit Calibration", Icon::PencilSimple, Kind::Secondary });
				list.push_back({ ActionId::Remove, "Remove Calibration", Icon::Trash, Kind::Danger });
			}
			return list;
		}
		if (CalCtx.state == CalibrationState::Editing)
			return {};
		// Calibrate also replaces an automatic run with a careful one.
		if (ctx.snap.acquire.state == AcquireState::Calibrating)
			return { { ActionId::Calibrate, "Calibrate", Icon::Crosshair, Kind::Primary } };
		return { { ActionId::ShowProgress, "Show Progress", Icon::Pulse, Kind::Secondary } };
	}

	void Run(const Context &ctx, ActionId id)
	{
		switch (id)
		{
		case ActionId::Calibrate:
			StartCalibration();
			*ctx.openProgress = true;
			break;
		case ActionId::Edit:
			CalCtx.state = CalibrationState::Editing;
			break;
		case ActionId::Remove:
			RemoveCalibration();
			break;
		case ActionId::ShowProgress:
			*ctx.openProgress = true;
			break;
		}
	}

	void ActionButton(const Context &ctx, const Action &action, const Rect &r)
	{
		if (Button((std::string("##") + action.label).c_str(), r, action.label, action.icon, action.kind))
			Run(ctx, action.id);
	}

	// Calibration values, editable while the driver applies them live.
	float Editor(float x, float y, float w)
	{
		const float pad = 24.0f;
		CardScope card;
		float cy = y + pad;
		Text(Font.semibold, TextSize + 4.0f, ImVec2(x + pad, cy), Col(Color::Text), "Edit Calibration");
		cy += TextSize + 10.0f;
		cy += Text(Font.regular, SmallSize, ImVec2(x + pad, cy), Col(Color::Muted), "Changes apply as you type. Save keeps them.", w - pad * 2.0f) + 18.0f;

		struct Field { const char *label; double *value; double step, fast; const char *format; };
		struct Group { const char *title; Field fields[3]; int count; };
		const Group groups[] = {
			{ "Rotation (degrees)", { { "Yaw", &CalCtx.calibratedRotation(1), 0.1, 1.0, "%.4f" }, { "Pitch", &CalCtx.calibratedRotation(2), 0.1, 1.0, "%.4f" }, { "Roll", &CalCtx.calibratedRotation(0), 0.1, 1.0, "%.4f" } }, 3 },
			{ "Position (cm)", { { "X", &CalCtx.calibratedTranslation(0), 1.0, 10.0, "%.4f" }, { "Y", &CalCtx.calibratedTranslation(1), 1.0, 10.0, "%.4f" }, { "Z", &CalCtx.calibratedTranslation(2), 1.0, 10.0, "%.4f" } }, 3 },
			{ "Scale", { { "Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.5f" }, { "HMD Scale", &CalCtx.hmdScale, 0.0001, 0.01, "%.5f" }, {} }, 2 },
		};

		const float labelW = std::min(220.0f, w * 0.24f);
		const float gap = 14.0f;
		const float fieldW = (w - pad * 2.0f - labelW - gap * 2.0f) / 3.0f;
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 8.0f));
		for (const auto &group : groups)
		{
			ImGui::PushID(group.title);
			Text(Font.medium, SmallSize, ImVec2(x + pad, cy + SmallSize + 10.0f), Col(Color::Text), group.title, labelW - 12.0f);
			for (int i = 0; i < group.count; ++i)
			{
				const Field &field = group.fields[i];
				float fx = x + pad + labelW + (fieldW + gap) * i;
				Text(Font.regular, SmallSize - 1.0f, ImVec2(fx, cy), Col(Color::Muted), field.label);
				ImGui::SetCursorScreenPos(ImVec2(fx, cy + SmallSize + 4.0f));
				ImGui::SetNextItemWidth(fieldW);
				ImGui::PushFont(Font.mono, SmallSize);
				ImGui::InputDouble((std::string("##") + field.label).c_str(), field.value, field.step, field.fast, field.format);
				ImGui::PopFont();
			}
			cy += SmallSize + 4.0f + SmallSize + 16.0f + 18.0f;
			ImGui::PopID();
		}
		ImGui::PopStyleVar();

		Rect save{ x + w - pad - 200.0f, cy + 4.0f, 200.0f, 46.0f };
		if (Button("##save", save, "Save", Icon::FloppyDisk, Kind::Primary))
		{
			SaveProfile(CalCtx);
			CalCtx.state = CalibrationState::None;
		}
		cy = save.Bottom() + pad;
		card.End({ x, y, w, cy - y });
		return cy - y;
	}

	// An icon with a title and a short explanation; returns the height.
	float Feature(float x, float y, float w, const char *icon, ImU32 color, const char *title, const std::string &text)
	{
		const float iconBox = 40.0f;
		Fill({ x, y, iconBox, iconBox }, Alpha(color, 0.14f), ControlRadius);
		IconAt(ImVec2(x + iconBox * 0.5f, y + iconBox * 0.5f), 22.0f, color, icon);
		const float tx = x + iconBox + 14.0f;
		Text(Font.medium, SmallSize + 1.0f, ImVec2(tx, y), Col(Color::Text), title);
		float h = SmallSize + 5.0f + Text(Font.regular, SmallSize - 1.0f, ImVec2(tx, y + SmallSize + 5.0f), Col(Color::Muted), text, w - (tx - x));
		return std::max(h, iconBox);
	}

	// ---------------------------------------------------------------- Settings and Smoothing

	// Scrollable page body; returns the top-left corner and width to lay out in.
	ImVec2 BeginPage(const Rect &area, float &width)
	{
		ImGui::SetCursorScreenPos(area.Min());
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
		ImGui::BeginChild("##page", ImVec2(area.w, area.h), ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
		const float scrollbar = ImGui::GetScrollMaxY() > 0.0f ? ImGui::GetStyle().ScrollbarSize + 8.0f : 0.0f;
		width = area.w - scrollbar;
		return ImGui::GetCursorScreenPos();
	}

	void EndPage(ImVec2 origin, float width, float height)
	{
		ImGui::SetCursorScreenPos(origin);
		ImGui::Dummy(ImVec2(width, height));
		ImGui::EndChild();
		ImGui::PopStyleVar();
	}

	float CardHeader(float x, float y, float w, const char *title, const char *caption)
	{
		float h = SectionLabel(x, y, title);
		if (caption)
		{
			h += 8.0f;
			IconAt(ImVec2(x + 9.0f, y + h + SmallSize * 0.5f), SmallSize, Col(Color::Faint), Icon::Info);
			h += Text(Font.regular, SmallSize - 1.0f, ImVec2(x + 26.0f, y + h), Col(Color::Faint), caption, w - 26.0f);
		}
		return h;
	}

	const char *const NextCalibration = "Applies at the next calibration.";

	float SettingsSearch(float x, float y, float w)
	{
		CardScope card;
		float cy = y + RowPadding;
		cy += CardHeader(x + RowPadding, cy, w - RowPadding * 2.0f, "Head tracker search", nullptr) + 4.0f;
		bool autoAcquire = CalCtx.autoAcquire, changed = false;
		cy += ToggleRow("##auto", x, cy, w, "Find head tracker automatically",
			"Uses hand tracking and the lighthouse controllers in your hands to find the tracker on your headset, then calibrates it without a button.",
			autoAcquire, changed);
		if (changed)
			SetAutoAcquire(autoAcquire);
		cy += RowPadding * 0.5f;
		card.End({ x, y, w, cy - y });
		return cy - y;
	}

	float SettingsTracking(float x, float y, float w)
	{
		CardScope card;
		float cy = y + RowPadding;
		cy += CardHeader(x + RowPadding, cy, w - RowPadding * 2.0f, "Tracking", NextCalibration) + 4.0f;
		bool changed = false;
		cy += ToggleRow("##slam", x, cy, w, "Fallback to SLAM",
			"If the head tracker loses tracking, the headset's own tracking takes over until it is back.", CalCtx.fallbackToSlam, changed);
		cy += ToggleRow("##relative", x, cy, w, "Relative Calibration",
			"Corrects drift so devices the headset tracks, like its controllers or your hands, stay lined up with your lighthouse space.",
			CalCtx.continuousSync, changed);
		cy += RowPadding * 0.5f;
		card.End({ x, y, w, cy - y });
		return cy - y;
	}

	float SettingsAdvanced(float x, float y, float w)
	{
		CardScope card;
		float cy = y + RowPadding;
		cy += CardHeader(x + RowPadding, cy, w - RowPadding * 2.0f, "Advanced", NextCalibration) + 4.0f;
		bool changed = false;
		cy += ToggleRow("##angular", x, cy, w, "Enable Angular Velocity",
			"Passes the tracker's angular velocity to SteamVR. Off by default because some devices misbehave with it.", CalCtx.enableAngularVelocity, changed);
		cy += ToggleRow("##native", x, cy, w, "Discard Calibrated Offset",
			"Uses the tracker alone, with no headset tracking even as a fallback. A yaw mismatch can remain that only re-centering in Local tracking space fixes. Tested on Pico only.",
			CalCtx.enableNative, changed);
		cy += RowPadding * 0.5f;
		card.End({ x, y, w, cy - y });
		return cy - y;
	}

	float SettingsCalibration(float x, float y, float w)
	{
		CardScope card;
		float cy = y + RowPadding;
		cy += CardHeader(x + RowPadding, cy, w - RowPadding * 2.0f, "Calibration", NextCalibration) + 14.0f;

		const float inner = w - RowPadding * 2.0f;
		Text(Font.medium, TextSize, ImVec2(x + RowPadding, cy), Col(Color::Text), "Prediction Time");
		char value[32];
		snprintf(value, sizeof value, "%.1f frames", CalCtx.predictionTime);
		TextRight(Font.mono, SmallSize, ImVec2(x + w - RowPadding, cy + 2.0f), Col(Color::Accent), value);
		cy += TextSize + 6.0f;
		cy += Text(Font.regular, SmallSize, ImVec2(x + RowPadding, cy), Col(Color::Muted), "How many frames of prediction SteamVR applies to the tracker. Some wireless setups need more to feel smooth.", inner) + 10.0f;
		double prediction = CalCtx.predictionTime;
		if (Slider("##prediction", { x + RowPadding, cy, inner, 30.0f }, prediction, 0.0, 10.0, 0.1))
			CalCtx.predictionTime = static_cast<float>(prediction);
		cy += 30.0f + 22.0f;

		Text(Font.medium, TextSize, ImVec2(x + RowPadding, cy), Col(Color::Text), "Calibration Speed");
		cy += TextSize + 6.0f;
		cy += Text(Font.regular, SmallSize, ImVec2(x + RowPadding, cy), Col(Color::Muted), "How many samples a calibration collects before it solves: Fast 100, Slow 250, Very Slow 500.", inner) + 12.0f;
		const char *const speeds[] = { "Fast", "Slow", "Very Slow" };
		int speed = static_cast<int>(CalCtx.calibrationSpeed);
		if (Segmented("##speed", { x + RowPadding, cy, inner, 44.0f }, speeds, 3, speed))
			CalCtx.calibrationSpeed = static_cast<CalibrationContext::Speed>(speed);
		cy += 44.0f + RowPadding;
		card.End({ x, y, w, cy - y });
		return cy - y;
	}

	void SettingsPage(const Rect &area)
	{
		float width = 0.0f;
		ImVec2 origin = BeginPage(area, width);
		const float gap = 20.0f, colW = (width - gap) * 0.5f;
		float left = SettingsSearch(origin.x, origin.y, colW);
		left += gap + SettingsTracking(origin.x, origin.y + left + gap, colW);
		float right = SettingsCalibration(origin.x + colW + gap, origin.y, colW);
		right += gap + SettingsAdvanced(origin.x + colW + gap, origin.y + right + gap, colW);
		EndPage(origin, width, std::max(left, right) + 8.0f);
	}

	float SmoothingSliders(const char *id, float x, float y, float w, protocol::OneEuroParams &p, bool enabled, bool &changed)
	{
		struct Row { const char *label, *hint, *tip, *unit; double *value; double min, max; };
		const Row rows[] = {
			{ "Min cutoff", "Lower is steadier when you hold still", "How steady things look when you are not moving.\nDrag left to remove shaking for a calmer image; drag right if things start to feel laggy or floaty.", " Hz", &p.minCutoff, 0.1, 5.0 },
			{ "Beta", "Higher keeps up better with fast moves", "How quickly tracking keeps up when you move fast.\nDrag right if fast movements feel delayed or laggy; drag left if they look shaky.", "", &p.beta, 0.0, 2.0 },
			{ "Derivative cutoff", "Most people can leave this alone", "Most people can leave this alone.\nIt fine-tunes how the smoothing reacts as your movement speed changes.", " Hz", &p.dCutoff, 0.1, 5.0 },
		};
		const float dim = enabled ? 1.0f : 0.45f;
		float cy = y;
		ImGui::PushID(id);
		for (const auto &row : rows)
		{
			Text(Font.medium, TextSize - 1.0f, ImVec2(x + RowPadding, cy), Col(Color::Text, dim), row.label);
			char value[32];
			snprintf(value, sizeof value, "%.3f%s", *row.value, row.unit);
			TextRight(Font.mono, SmallSize, ImVec2(x + w - RowPadding, cy + 2.0f), Col(Color::Accent, dim), value);
			cy += TextSize + 2.0f;
			Text(Font.regular, SmallSize - 1.0f, ImVec2(x + RowPadding, cy), Col(Color::Muted, dim), row.hint);
			cy += SmallSize + 6.0f;
			changed |= Slider((std::string("##") + id + " " + row.label).c_str(), { x + RowPadding, cy, w - RowPadding * 2.0f, 28.0f }, *row.value, row.min, row.max, 0.001, enabled);
			Tooltip(row.tip);
			cy += 28.0f + 16.0f;
		}
		ImGui::PopID();
		return cy - y;
	}

	float SmoothingHeadset(float x, float y, float w, bool &changed)
	{
		CardScope card;
		float cy = y + RowPadding;
		cy += CardHeader(x + RowPadding, cy, w - RowPadding * 2.0f, "Headset Tracker", nullptr) + 4.0f;
		cy += ToggleRow("##headfilter", x, cy, w, "Smooth headset tracker",
			"Steadies what you see through the headset. It can add a little delay; if your view lags when you move quickly, adjust the sliders below.",
			CalCtx.headFilterEnabled, changed);
		cy += 10.0f;
		cy += SmoothingSliders("head", x, cy, w, CalCtx.headFilterParams, CalCtx.headFilterEnabled, changed);
		card.End({ x, y, w, cy - y });
		return cy - y;
	}

	float SmoothingRelative(float x, float y, float w, bool &changed)
	{
		CardScope card;
		float cy = y + RowPadding;
		cy += CardHeader(x + RowPadding, cy, w - RowPadding * 2.0f, "Relative Calibration", nullptr) + 10.0f;
		cy += Text(Font.regular, SmallSize, ImVec2(x + RowPadding, cy), Col(Color::Muted),
			"Keeps your controllers and other tracked devices lined up with your real space, and steadies your view if the headset briefly loses tracking. Add more smoothing if they look shaky; ease off if they are slow to line up.",
			w - RowPadding * 2.0f) + 18.0f;
		cy += SmoothingSliders("drift", x, cy, w, CalCtx.driftFilterParams, true, changed);
		card.End({ x, y, w, cy - y });
		return cy - y;
	}

	float SmoothingIntro(float x, float y, float w)
	{
		float h = 0.0f;
		IconAt(ImVec2(x + 10.0f, y + SmallSize * 0.5f + 1.0f), SmallSize + 2.0f, Col(Color::Good), Icon::CheckCircle);
		h += Text(Font.medium, SmallSize, ImVec2(x + 28.0f, y), Col(Color::Good), "Changes apply right away, no recalibration needed.") + 8.0f;
		h += Text(Font.regular, SmallSize, ImVec2(x, y + h), Col(Color::Muted),
			"Smoothing makes your view and devices look steady instead of shaky. If something looks shaky, add more smoothing. If it feels laggy or floaty when you move, ease off. Hover a slider for tips.",
			w);
		return h;
	}

	void SmoothingPage(const Rect &area)
	{
		float width = 0.0f;
		ImVec2 origin = BeginPage(area, width);
		const float gap = 20.0f, colW = (width - gap) * 0.5f;
		bool changed = false;
		float y = origin.y;
		y += SmoothingIntro(origin.x, y, width) + gap;
		float left = SmoothingHeadset(origin.x, y, colW, changed);
		float right = SmoothingRelative(origin.x + colW + gap, y, colW, changed);
		y += std::max(left, right);
		if (changed)
			SendOneEuroParams();
		EndPage(origin, width, y - origin.y + 8.0f);
	}

	// ---------------------------------------------------------------- Calibration progress window

	float LogLineHeight(const std::string &text, float width)
	{
		return Measure(Font.medium, TextSize + 2.0f, Trimmed(text), width).y + 12.0f;
	}

	void ProgressWindow()
	{
		const ImVec2 display = ImGui::GetIO().DisplaySize;
		const float pad = 32.0f, width = 740.0f;
		const float textW = width - pad * 2.0f;
		// Calibrate runs also write their status into the log; show it once.
		const bool statusLogged = std::any_of(CalCtx.messages.begin(), CalCtx.messages.end(), [](const CalibrationContext::Message &message) {
			return message.type == CalibrationContext::Message::String && message.str == CalCtx.statusLine;
		});
		float logH = !CalCtx.statusLine.empty() && !statusLogged ? LogLineHeight(CalCtx.statusLine, textW) : 0.0f;
		for (const auto &message : CalCtx.messages)
			logH += message.type == CalibrationContext::Message::String ? LogLineHeight(message.str, textW) : 34.0f;
		const ImVec2 size(width, std::clamp(pad * 2.0f + 78.0f + logH + 16.0f + 52.0f, 300.0f, display.y - 80.0f));
		ImGui::SetNextWindowPos(ImVec2(std::floor((display.x - size.x) * 0.5f), std::floor((display.y - size.y) * 0.5f)));
		ImGui::SetNextWindowSize(size);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
		ImGui::PushStyleColor(ImGuiCol_PopupBg, IM_COL32(0, 0, 0, 0));
		if (ImGui::BeginPopupModal(ProgressPopup, nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings))
		{
			const ImVec2 pos = ImGui::GetWindowPos();
			Rect r{ pos.x, pos.y, size.x, size.y };
			// The shadow reaches past the window, so lift the window clip while drawing the frame.
			Canvas()->PushClipRect(ImVec2(0.0f, 0.0f), display, false);
			Shadow(r, CardRadius, 24.0f, Col(0x000000, 0.45f));
			Fill(r, Col(Color::Card), CardRadius);
			Outline(r, Col(Color::Line), CardRadius);
			Canvas()->PopClipRect();

			const bool running = CalCtx.state != CalibrationState::None && CalCtx.state != CalibrationState::Editing;
			float y = r.y + pad;
			const ImVec2 badge(r.x + pad + 26.0f, y + 26.0f);
			const ImU32 tone = running ? Col(Color::Accent) : Col(Color::Good);
			Canvas()->AddCircleFilled(badge, 26.0f, Alpha(tone, 0.16f), 48);
			IconAt(badge, 26.0f, tone, running ? Icon::Crosshair : Icon::CheckCircle);
			if (running)
				Spinner(badge, 32.0f, tone, 3.0f);
			const float tx = r.x + pad + 72.0f;
			Text(Font.semibold, HeadingSize, ImVec2(tx, y), Col(Color::Text), "Calibration");
			Text(Font.regular, SmallSize, ImVec2(tx, y + HeadingSize + 4.0f), Col(Color::Muted), running ? "Keep the headset on and follow the steps." : "You can close this window.");
			y += 52.0f + 26.0f;

			ImGui::SetCursorScreenPos(ImVec2(r.x + pad, y));
			ImGui::BeginChild("##log", ImVec2(textW, r.Bottom() - pad - 52.0f - 16.0f - y), ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
			const ImVec2 origin = ImGui::GetCursorScreenPos();
			float ly = origin.y;
			if (!CalCtx.statusLine.empty() && !statusLogged)
				ly += Text(Font.medium, TextSize + 2.0f, ImVec2(origin.x, ly), Col(Color::Text), CalCtx.statusLine, textW) + 12.0f;
			for (const auto &message : CalCtx.messages)
			{
				if (message.type == CalibrationContext::Message::String)
					ly += Text(Font.medium, TextSize + 2.0f, ImVec2(origin.x, ly), Col(Color::Text), Trimmed(message.str), textW) + 12.0f;
				else
				{
					float fraction = message.target > 0 ? std::clamp(static_cast<float>(message.progress) / message.target, 0.0f, 1.0f) : 0.0f;
					ProgressBar({ origin.x, ly + 6.0f, textW - 80.0f, 12.0f }, fraction);
					if (message.target > 0)
						TextRight(Font.mono, TextSize, ImVec2(origin.x + textW, ly), Col(Color::Text), Percent(fraction));
					ly += 34.0f;
				}
			}
			ImGui::SetCursorScreenPos(origin);
			ImGui::Dummy(ImVec2(textW, ly - origin.y));
			ImGui::EndChild();

			Rect button{ r.Right() - pad - 220.0f, r.Bottom() - pad - 52.0f, 220.0f, 52.0f };
			if (running)
			{
				if (Button("##cancel", button, "Cancel", Icon::X, Kind::Secondary))
				{
					CancelCalibration();
					// Keep the popup until the run has actually stopped.
					if (CalCtx.state == CalibrationState::None)
						ImGui::CloseCurrentPopup();
				}
			}
			else if (Button("##close", button, "Close", Icon::Check, Kind::Primary))
				ImGui::CloseCurrentPopup();
			ImGui::EndPopup();
		}
		ImGui::PopStyleColor();
		ImGui::PopStyleVar();
	}

	// ---------------------------------------------------------------- Calibration page

	const Page Pages[] = { Page::Calibration, Page::Smoothing, Page::Settings };

	const char *PageTitle(Page page)
	{
		switch (page)
		{
		case Page::Smoothing: return "Smoothing";
		case Page::Settings: return "Settings";
		default: return "Calibration";
		}
	}

	const char *PageIcon(Page page)
	{
		switch (page)
		{
		case Page::Smoothing: return Icon::WaveSine;
		case Page::Settings: return Icon::GearSix;
		default: return Icon::Crosshair;
		}
	}

	const char *PageSubtitle(Page page)
	{
		switch (page)
		{
		case Page::Smoothing: return "Steadier tracking, applied instantly.";
		case Page::Settings: return "Head tracker search, tracking and calibration options.";
		default: return "Find, calibrate and keep your head tracker.";
		}
	}

	float Stepper(const Context &ctx, float x, float y, float w)
	{
		const float pad = 24.0f;
		CardScope card;
		SectionLabel(x + pad, y + pad, "Automatic setup");
		TextRight(Font.regular, SmallSize - 2.0f, ImVec2(x + w - pad, y + pad - 1.0f), Col(Color::Faint), "Uses hand tracking and your lighthouse controllers");
		const float col = (w - pad * 2.0f) / 5.0f, cy = y + pad + 58.0f, radius = 20.0f;
		for (int i = 0; i < 5; ++i)
		{
			const float cx = x + pad + col * (i + 0.5f);
			const bool done = i < ctx.status.step, current = i == ctx.status.step;
			if (i < 4)
				Canvas()->AddLine(ImVec2(cx + radius + 8.0f, cy), ImVec2(cx + col - radius - 8.0f, cy), done ? Col(Color::Accent) : Col(Color::Line), 2.0f);
			if (done)
			{
				Canvas()->AddCircleFilled(ImVec2(cx, cy), radius, Col(Color::Accent), 48);
				IconAt(ImVec2(cx, cy), 20.0f, Col(Color::White), Icon::Check);
			}
			else if (current)
			{
				Pulse(ImVec2(cx, cy), radius, 16.0f, Col(Color::Accent));
				Canvas()->AddCircleFilled(ImVec2(cx, cy), radius, Col(Color::Accent, 0.16f), 48);
				Canvas()->AddCircle(ImVec2(cx, cy), radius, Col(Color::Accent), 48, 2.0f);
				IconAt(ImVec2(cx, cy), 20.0f, Col(Color::Accent), StepIcons[i]);
			}
			else
			{
				Canvas()->AddCircle(ImVec2(cx, cy), radius, Col(Color::Line), 48, 2.0f);
				IconAt(ImVec2(cx, cy), 20.0f, Col(Color::Faint), StepIcons[i]);
			}
			TextCentered(Font.medium, SmallSize, ImVec2(cx, cy + radius + 22.0f), Col(done || current ? Color::Text : Color::Muted), StepTitles[i]);
			TextBlockCentered(Font.regular, SmallSize - 2.0f, cx, cy + radius + 34.0f, Col(current ? Color::Accent : Color::Faint), StepHint(i, ctx), col - 10.0f);
		}
		const float h = cy - y + radius + 34.0f + (SmallSize - 2.0f) * 2.0f + pad - 6.0f;
		card.End({ x, y, w, h });
		return h;
	}

	void DeviceCard(const DeviceInfo &info, float x, float y, float w, float h)
	{
		const float pad = 22.0f;
		Card({ x, y, w, h });
		Rect tile{ x + pad, y + pad, 48.0f, 48.0f };
		Fill(tile, Col(Color::Raised), ControlRadius);
		IconAt(tile.Center(), 26.0f, Col(Color::Muted), info.icon);
		const float tx = tile.Right() + 16.0f;
		Caps(Font.medium, LabelSize - 1.0f, ImVec2(tx, y + pad), Col(Color::Faint), info.role, 1.2f);
		Text(Font.semibold, TextSize + 1.0f, ImVec2(tx, y + pad + 18.0f), Col(Color::Text), info.name);
		std::string meta = info.serial;
		if (!info.system.empty())
			meta += (meta.empty() ? "" : "  \xC2\xB7  ") + info.system;
		if (!meta.empty())
			Text(Font.mono, SmallSize - 1.0f, ImVec2(x + pad, tile.Bottom() + 16.0f), Col(Color::Muted), meta);
		Chip(x + w - pad - ChipWidth(info.state), y + pad, info.tone, info.state);
	}

	const char *const BindingTitle = "Kept until you remove it";
	const char *const BindingText = "The head tracker stays bound through restarts, tracking loss and failed runs.";

	// Why the link lasts: the saved binding, and what happened to the alignment saved last session.
	float MemoryCard(const Context &ctx, float x, float y, float w)
	{
		const float pad = 24.0f, gap = 16.0f, half = (w - gap) * 0.5f;
		const RestoreView restore = DescribeRestore(ctx.snap.restore);
		CardScope card;
		float left = Feature(x + pad, y + pad, half - pad - 16.0f, Icon::PushPin, Col(Color::Accent), BindingTitle, BindingText);
		float right = Feature(x + half + gap + 8.0f, y + pad, half - pad - 8.0f, Icon::ClockCounterClockwise, ToneColor(restore.tone), restore.title, restore.text);
		const float h = std::max(left, right) + pad * 2.0f;
		if (!restore.detail.empty())
			TooltipArea("##restore", { x + half + gap, y, w - half - gap, h }, restore.detail);
		card.End({ x, y, w, h });
		return h;
	}

	void CalibrationPage(const Context &ctx, const Rect &area)
	{
		const Status &s = ctx.status;
		float y = area.y;
		float alert = DriverAlert(ctx, area.x, y, area.w);
		if (alert > 0.0f)
			y += alert + 16.0f;

		// Status card
		const float pad = 28.0f, badge = 34.0f;
		const auto actions = Actions(ctx);
		const float actionsW = actions.empty() ? 0.0f : 214.0f;
		const float tx = area.x + pad + badge * 2.0f + 24.0f;
		const float textW = area.Right() - pad - (actionsW > 0.0f ? actionsW + 28.0f : 0.0f) - tx;
		float textH = TitleSize + 10.0f + Measure(Font.regular, TextSize, s.detail, textW).y;
		if (!s.note.empty())
			textH += 10.0f + Measure(Font.regular, SmallSize, s.note, textW).y;
		if (s.progress >= 0.0f)
			textH += 32.0f;
		const float actionsH = actions.size() * 46.0f + (actions.empty() ? 0.0f : (actions.size() - 1) * 10.0f);
		Rect hero{ area.x, y, area.w, std::max({ textH, actionsH, badge * 2.0f }) + pad * 2.0f };
		Card(hero);
		const ImVec2 center(area.x + pad + badge, hero.y + pad + badge);
		const ImU32 tone = ToneColor(s.tone);
		Canvas()->AddCircleFilled(center, badge, Alpha(tone, 0.14f), 64);
		IconAt(center, 32.0f, tone, s.icon);
		if (s.tone == Tone::Busy)
			Spinner(center, badge + 6.0f, tone, 3.0f);
		float ty = hero.y + pad - 4.0f;
		Text(Font.semibold, TitleSize, ImVec2(tx, ty), Col(Color::Text), s.title);
		ty += TitleSize + 10.0f;
		ty += Text(Font.regular, TextSize, ImVec2(tx, ty), Col(Color::Muted), s.detail, textW);
		if (!s.note.empty())
			ty += 10.0f + Text(Font.regular, SmallSize, ImVec2(tx, ty + 10.0f), Col(Color::Faint), s.note, textW);
		if (s.progress >= 0.0f)
		{
			ProgressBar({ tx, ty + 20.0f, textW - 64.0f, 8.0f }, s.progress);
			TextRight(Font.mono, SmallSize, ImVec2(tx + textW, ty + 13.0f), Col(Color::Text), Percent(s.progress));
		}
		for (size_t i = 0; i < actions.size(); ++i)
			ActionButton(ctx, actions[i], { hero.Right() - pad - actionsW, hero.y + pad + i * 56.0f, actionsW, 46.0f });
		y = hero.Bottom() + 18.0f;

		if (CalCtx.state == CalibrationState::Editing)
		{
			Editor(area.x, y, area.w);
			return;
		}
		if (s.step >= 0 && s.step < 5)
			y += Stepper(ctx, area.x, y, area.w) + 18.0f;
		const float gap = 16.0f, cardW = (area.w - gap) * 0.5f, cardH = 136.0f;
		DeviceCard(TrackerInfo(ctx), area.x, y, cardW, cardH);
		DeviceCard(HeadsetInfo(ctx.snap), area.x + cardW + gap, y, cardW, cardH);
		y += cardH + gap;
		if (s.bound)
			MemoryCard(ctx, area.x, y, area.w);
	}

	void Footer(const Context &ctx, float x, float y, float w)
	{
		Text(Font.regular, 14.0f, ImVec2(x, y), Col(Color::Faint), Credits);
		if (ctx.overlay)
			TextRight(Font.regular, 14.0f, ImVec2(x + w, y), Col(Color::Muted), OverlayHint);
	}

	void Shell(Context &ctx)
	{
		const ImVec2 display = ImGui::GetIO().DisplaySize;
		const float side = 236.0f;
		Fill({ 0, 0, display.x, display.y }, Col(Color::Base));
		Fill({ 0, 0, side, display.y }, Col(Color::Sidebar));
		Canvas()->AddLine(ImVec2(side, 0), ImVec2(side, display.y), Col(Color::Line));

		Rect mark{ 22, 26, 42, 42 };
		FillGradient(mark, Col(Color::Accent), Col(Color::AccentDeep), 12.0f);
		IconAt(mark.Center(), 26.0f, Col(Color::White), Icon::CompassTool);
		Text(Font.semibold, 23.0f, ImVec2(mark.Right() + 14.0f, mark.y + 1.0f), Col(Color::Text), "SpaceLink");
		Text(Font.regular, 14.0f, ImVec2(mark.Right() + 14.0f, mark.y + 27.0f), Col(Color::Faint), "v" SPACECAL_VERSION_STRING);

		float ny = 112.0f;
		for (Page page : Pages)
		{
			Rect r{ 14, ny, side - 28, 48 };
			Hit hit = Interact((std::string("##") + PageTitle(page)).c_str(), r);
			if (hit.clicked)
				*ctx.page = page;
			const bool on = *ctx.page == page;
			if (on)
			{
				Fill(r, Col(Color::Accent, 0.13f), ControlRadius);
				Fill({ r.x, r.y + 12.0f, 3.0f, r.h - 24.0f }, Col(Color::Accent), 2.0f);
			}
			else if (hit.hovered)
				Fill(r, Col(Color::Raised), ControlRadius);
			IconAt(ImVec2(r.x + 28.0f, r.Center().y), 22.0f, Col(on ? Color::Accent : Color::Muted), PageIcon(page));
			Text(Font.medium, 19.0f, ImVec2(r.x + 52.0f, r.Center().y - 9.5f), Col(on ? Color::Text : Color::Muted), PageTitle(page));
			ny += 54.0f;
		}

		Rect status{ 14, display.y - 116, side - 28, 92 };
		Fill(status, Col(Color::Card), ControlRadius + 2.0f);
		Outline(status, Col(Color::Line), ControlRadius + 2.0f);
		const ImU32 tone = ToneColor(ctx.status.tone);
		Canvas()->AddCircleFilled(ImVec2(status.x + 22.0f, status.y + 28.0f), 9.0f, Alpha(tone, 0.18f), 24);
		Canvas()->AddCircleFilled(ImVec2(status.x + 22.0f, status.y + 28.0f), 4.5f, tone, 16);
		Text(Font.medium, 17.0f, ImVec2(status.x + 38.0f, status.y + 18.0f), Col(Color::Text), ctx.status.word);
		std::string serial = CalCtx.trackerSerial.empty() ? (ctx.snap.acquire.trackerSerial.empty() ? "No head tracker" : ctx.snap.acquire.trackerSerial) : CalCtx.trackerSerial;
		Text(Font.mono, 14.0f, ImVec2(status.x + 18.0f, status.y + 50.0f), Col(Color::Muted), serial);
		Text(Font.regular, 13.0f, ImVec2(status.x + 18.0f, status.y + 68.0f), Col(Color::Faint), ctx.snap.driver.connected ? "Driver connected" : "Driver not connected");

		const float x = side + 40.0f, w = display.x - side - 80.0f;
		Text(Font.semibold, HeadingSize, ImVec2(x, 34.0f), Col(Color::Text), PageTitle(*ctx.page));
		Text(Font.regular, SmallSize, ImVec2(x, 34.0f + HeadingSize + 6.0f), Col(Color::Muted), PageSubtitle(*ctx.page));
		switch (*ctx.page)
		{
		case Page::Calibration:
			CalibrationPage(ctx, { x, 112.0f, w, display.y - 112.0f - 52.0f });
			Footer(ctx, x, display.y - 34.0f, w);
			break;
		case Page::Smoothing:
			SmoothingPage({ x, 104.0f, w, display.y - 104.0f - 24.0f });
			break;
		case Page::Settings:
			SettingsPage({ x, 104.0f, w, display.y - 104.0f - 24.0f });
			break;
		}
	}

	// ---------------------------------------------------------------- Setup

	// Loads a bundled font. Text fonts also get Windows' fonts merged in for scripts they lack, scaled so
	// their em matches the bundled font's; Dear ImGui then reads those glyphs only when text needs them.
	ImFont *LoadFont(const unsigned char *data, std::size_t size, bool fallbacks)
	{
		ImFontAtlas *atlas = ImGui::GetIO().Fonts;
		ImFontConfig config;
		config.FontDataOwnedByAtlas = false;
		ImFont *font = atlas->AddFontFromMemoryTTF(const_cast<unsigned char *>(data), static_cast<int>(size), 20.0f, &config);
		if (!fallbacks)
			return font;
		const float height = LineHeightInEms(data, size);
		for (const SystemFont &system : SystemFallbackFonts())
		{
			ImFontConfig merge;
			merge.MergeMode = true;
			merge.FontDataOwnedByAtlas = false;
			const float systemHeight = LineHeightInEms(system.data, system.size);
			if (height > 0.0f && systemHeight > 0.0f)
				merge.ExtraSizeScale = systemHeight / height;
			atlas->AddFontFromMemoryTTF(const_cast<unsigned char *>(system.data), static_cast<int>(system.size), 20.0f, &merge);
		}
		return font;
	}

	void ApplyStyle()
	{
		ImGuiStyle &style = ImGui::GetStyle();
		style = ImGuiStyle();
		ImGui::StyleColorsDark(&style);

		style.WindowPadding = ImVec2(14.0f, 10.0f);
		style.WindowRounding = 0.0f;
		style.WindowBorderSize = 0.0f;
		style.ChildBorderSize = 0.0f;
		style.PopupRounding = ControlRadius;
		style.PopupBorderSize = 1.0f;
		style.FramePadding = ImVec2(10.0f, 8.0f);
		style.FrameRounding = ControlRadius;
		style.ItemSpacing = ImVec2(8.0f, 8.0f);
		style.ItemInnerSpacing = ImVec2(6.0f, 6.0f);
		style.ScrollbarSize = 10.0f;
		style.ScrollbarRounding = 6.0f;
		style.GrabRounding = ControlRadius;
		style.FontSizeBase = TextSize;

		ImVec4 *colors = style.Colors;
		colors[ImGuiCol_Text] = Srgb(Color::Text);
		colors[ImGuiCol_TextDisabled] = Srgb(Color::Faint);
		colors[ImGuiCol_WindowBg] = Srgb(Color::Base);
		colors[ImGuiCol_ChildBg] = Srgb(0, 0.0f);
		colors[ImGuiCol_PopupBg] = Srgb(Color::Raised, 0.98f);
		colors[ImGuiCol_Border] = Srgb(Color::Line);
		colors[ImGuiCol_FrameBg] = Srgb(Color::Raised);
		colors[ImGuiCol_FrameBgHovered] = Srgb(Color::Line);
		colors[ImGuiCol_FrameBgActive] = Srgb(Color::Line);
		colors[ImGuiCol_Button] = Srgb(Color::Raised);
		colors[ImGuiCol_ButtonHovered] = Srgb(Color::Line);
		colors[ImGuiCol_ButtonActive] = Srgb(Color::Accent, 0.5f);
		colors[ImGuiCol_ScrollbarBg] = Srgb(0, 0.0f);
		colors[ImGuiCol_ScrollbarGrab] = Srgb(Color::Line);
		colors[ImGuiCol_ScrollbarGrabHovered] = Srgb(Color::Faint);
		colors[ImGuiCol_ScrollbarGrabActive] = Srgb(Color::Muted);
		colors[ImGuiCol_CheckMark] = Srgb(Color::Accent);
		colors[ImGuiCol_SliderGrab] = Srgb(Color::Accent);
		colors[ImGuiCol_SliderGrabActive] = Srgb(Color::Accent);
		colors[ImGuiCol_TextSelectedBg] = Srgb(Color::Accent, 0.35f);
		colors[ImGuiCol_InputTextCursor] = Srgb(Color::Accent);
		colors[ImGuiCol_NavCursor] = Srgb(Color::Accent);
		colors[ImGuiCol_ModalWindowDimBg] = Srgb(0x000000, 0.6f);

		// Dear ImGui doesn't handle sRGB render targets, so the sRGB style colors are converted to linear.
		// https://github.com/ocornut/imgui/issues/8271#issuecomment-2564954070
		// Remove when these are merged:
		//  https://github.com/ocornut/imgui/pull/8110
		//  https://github.com/ocornut/imgui/pull/8111
		if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_IsSRGB)
			for (int i = 0; i < ImGuiCol_COUNT; i++)
			{
				ImVec4 &col = style.Colors[i];
				col = ImVec4(ToLinear(col.x), ToLinear(col.y), ToLinear(col.z), col.w);
			}
		ImGui::GetIO().FontDefault = Font.regular;
	}
}

void UserInterface::Setup()
{
	Font.regular = LoadFont(Font_Geist_Regular, Font_Geist_Regular_size, true);
	Font.medium = LoadFont(Font_Geist_Medium, Font_Geist_Medium_size, true);
	Font.semibold = LoadFont(Font_Geist_SemiBold, Font_Geist_SemiBold_size, true);
	Font.mono = LoadFont(Font_GeistMono_Regular, Font_GeistMono_Regular_size, true);
	Font.icons = LoadFont(Font_Phosphor, Font_Phosphor_size, false);
	ApplyStyle();
}

void UserInterface::Render(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();

	Context ctx;
	ctx.page = &page;
	ctx.openProgress = &openProgress;
	ctx.overlay = runningInOverlay;
	ctx.snap.driver = GetDriverLinkStatus();
	ctx.snap.headTracker = GetHeadTrackerState();
	ctx.snap.acquire = GetAcquireStatus();
	ctx.snap.restore = GetSpaceRestoreStatus();
	if (page == Page::Calibration)
		ctx.snap.vr = ReadVRState();
	for (const auto &device : ctx.snap.vr.devices)
		if (device.id == vr::k_unTrackedDeviceIndex_Hmd)
			ctx.snap.hmd = &device;
	ctx.status = Describe(ctx.snap);

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(io.DisplaySize);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar
		| ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
	if (!ImGui::Begin("MainWindow", nullptr, flags))
	{
		ImGui::End();
		ImGui::PopStyleVar();
		return;
	}
	ImGui::PopStyleVar();
	ImGui::PushFont(Font.regular, TextSize);

	Shell(ctx);

	if (openProgress)
	{
		ImGui::OpenPopup(ProgressPopup);
		openProgress = false;
	}
	ProgressWindow();

	ImGui::PopFont();
	ImGui::End();
}
