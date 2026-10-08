// SPDX-License-Identifier: AGPL-3.0-only

// Clicks through the UI the way a user would and checks that every control still does its job.
// Items are found through Dear ImGui's test engine hooks, so the UI code needs no test hooks of its own.

#include "PreviewState.h"
#include "SoftwareRenderer.h"
#include "UserInterface.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace
{
	struct Item
	{
		std::string label;
		ImRect rect, clip;
	};

	std::map<ImGuiID, Item> Items;
}

void ImGuiTestEngineHook_ItemAdd(ImGuiContext *ctx, ImGuiID id, const ImRect &bb, const ImGuiLastItemData *)
{
	Items[id].rect = bb;
	Items[id].clip = ctx->CurrentWindow ? ctx->CurrentWindow->ClipRect : bb;
}
void ImGuiTestEngineHook_ItemInfo(ImGuiContext *, ImGuiID id, const char *label, ImGuiItemStatusFlags) { Items[id].label = label ? label : ""; }
void ImGuiTestEngineHook_Log(ImGuiContext *, const char *, ...) {}
const char *ImGuiTestEngine_FindItemDebugLabel(ImGuiContext *, ImGuiID) { return nullptr; }

namespace
{
	class Session
	{
	public:
		Session(const std::string &sceneName, bool openProgress)
		{
			for (const auto &scene : preview::Scenes())
				if (scene.name == sceneName)
				{
					preview::Reset();
					scene.setup();
					page = scene.page;
					progress = scene.progress && openProgress;
					overlay = scene.overlay;
				}
			ImGui::CreateContext();
			ImGuiIO &io = ImGui::GetIO();
			io.IniFilename = nullptr;
			io.ConfigFlags |= ImGuiConfigFlags_IsSRGB;
			io.DisplaySize = ImVec2(1200.0f, 800.0f);
			io.DeltaTime = 1.0f / 90.0f;
			ImGui::GetCurrentContext()->TestEngineHookItems = true;
			SoftwareRenderer::Install();
			ui.Setup();
			ui.ShowPage(page);
			if (progress)
				ui.ShowCalibrationProgress();
			Frame(4);
		}

		~Session()
		{
			SoftwareRenderer::Uninstall();
			ImGui::DestroyContext();
		}

		void Frame(int count = 1)
		{
			for (int i = 0; i < count; ++i)
			{
				Items.clear();
				ImGui::NewFrame();
				ui.Render(overlay);
				ImGui::Render();
				SoftwareRenderer::UpdateTextures(ImGui::GetDrawData());
			}
		}

		const Item *Find(const std::string &label) const
		{
			for (const auto &entry : Items)
				if (entry.second.label == label && entry.second.rect.GetWidth() > 0.0f)
					return &entry.second;
			return nullptr;
		}

		bool Has(const std::string &label) const { return Find(label) != nullptr; }

		// Scrolls the page with the mouse wheel until the item is in view, as a user would.
		// Dear ImGui only reports labels of visible items, so an unseen item is searched for downwards first.
		const Item *Reveal(const std::string &label)
		{
			ImGuiIO &io = ImGui::GetIO();
			for (int attempt = 0; attempt < 60; ++attempt)
			{
				const Item *item = Find(label);
				if (item && item->clip.Contains(item->rect.GetCenter()))
					return item;
				ImVec2 at = item ? item->clip.GetCenter() : ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f);
				float wheel = item ? (item->rect.GetCenter().y > item->clip.Max.y ? -1.0f : 1.0f) : (attempt < 30 ? -1.0f : 1.0f);
				io.AddMousePosEvent(at.x, at.y);
				io.AddMouseWheelEvent(0.0f, wheel);
				Frame(2);
			}
			return nullptr;
		}

		// Presses at a point inside the item, given as fractions of its size.
		bool Press(const std::string &label, float fx = 0.5f, float fy = 0.5f)
		{
			const Item *item = Reveal(label);
			if (!item)
				return false;
			ImVec2 at(item->rect.Min.x + item->rect.GetWidth() * fx, item->rect.Min.y + item->rect.GetHeight() * fy);
			ImGuiIO &io = ImGui::GetIO();
			io.AddMousePosEvent(at.x, at.y);
			Frame();
			io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
			Frame();
			io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
			Frame();
			io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
			Frame(2);
			return true;
		}

	private:
		UserInterface ui;
		UserInterface::Page page = UserInterface::Page::Calibration;
		bool progress = false, overlay = false;
	};

	// Every character of the text has a glyph in the default font or one merged into it, so none shows as '?'.
	bool EveryGlyphPresent(const char *text)
	{
		ImFont *font = ImGui::GetIO().FontDefault;
		for (const char *c = text; *c;)
		{
			unsigned int codepoint = 0;
			int length = ImTextCharFromUtf8(&codepoint, c, nullptr);
			if (length <= 0)
				return false;
			c += length;
			if (codepoint >= ' ' && !font->IsGlyphInFont(static_cast<ImWchar>(codepoint)))
				return false;
		}
		return true;
	}

	struct Check
	{
		const char *scene;
		const char *what;
		std::function<bool(Session &)> run;
		bool openProgress = true;
	};

	std::vector<Check> Checks()
	{
		using preview::Count;
		return {
			{ "searching", "Calibrate starts a calibration and opens the progress window", [](Session &s) {
				return s.Press("##Calibrate") && Count.startCalibration == 1 && s.Has("##close");
			} },
			{ "searching", "Close shuts the progress window", [](Session &s) {
				return s.Press("##Calibrate") && s.Press("##close") && !s.Has("##close");
			} },
			{ "auto-calibrating", "Calibrate works during an automatic run", [](Session &s) {
				return s.Press("##Calibrate") && Count.startCalibration == 1;
			} },
			{ "progress", "Show Progress reopens the progress window", [](Session &s) {
				return !s.Has("##cancel") && s.Press("##Show Progress") && s.Has("##cancel");
			}, false },
			{ "tilt-recalibrating", "Show Progress opens the window during a tilt recalibration", [](Session &s) {
				return !s.Has("##cancel") && s.Press("##Show Progress") && s.Has("##cancel");
			}, false },
			{ "progress", "Cancel cancels and keeps the window until the run stops", [](Session &s) {
				return s.Press("##cancel") && Count.cancelCalibration == 1 && s.Has("##cancel");
			} },
			{ "progress-done", "Close appears once the run is over", [](Session &s) {
				return s.Has("##close") && !s.Has("##cancel") && s.Press("##close") && !s.Has("##close");
			} },
			{ "active", "Edit Calibration opens the editor and Save stores it", [](Session &s) {
				return s.Press("##Edit Calibration") && CalCtx.state == CalibrationState::Editing && s.Press("##save")
					&& Count.saveProfile == 1 && CalCtx.state == CalibrationState::None;
			} },
			{ "edit", "The editor's step buttons change one value by one step", [](Session &s) {
				auto values = [] {
					return std::vector<double>{ CalCtx.calibratedRotation(0), CalCtx.calibratedRotation(1), CalCtx.calibratedRotation(2),
						CalCtx.calibratedTranslation(0), CalCtx.calibratedTranslation(1), CalCtx.calibratedTranslation(2), CalCtx.calibratedScale, CalCtx.hmdScale };
				};
				const double steps[] = { 0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 0.0001, 0.0001 };
				auto before = values();
				if (!s.Press("+"))
					return false;
				auto after = values();
				int changed = 0;
				for (size_t i = 0; i < before.size(); ++i)
					if (after[i] != before[i])
						changed += std::abs(after[i] - before[i] - steps[i]) < 1e-9 ? 1 : 100;
				return changed == 1 && s.Has("##save");
			} },
			{ "active", "Remove Calibration removes it", [](Session &s) {
				return s.Press("##Remove Calibration") && Count.removeCalibration == 1;
			} },
			{ "active", "Navigation switches between the three pages", [](Session &s) {
				return s.Press("##Settings") && s.Has("##auto") && s.Press("##Smoothing") && s.Has("##headfilter")
					&& s.Press("##Calibration") && s.Has("##Calibrate");
			} },
			{ "settings", "Find head tracker automatically goes through SetAutoAcquire", [](Session &s) {
				return s.Press("##auto") && Count.setAutoAcquire == 1 && !CalCtx.autoAcquire;
			} },
			{ "settings", "The four tracking switches toggle their settings", [](Session &s) {
				return s.Press("##slam") && !CalCtx.fallbackToSlam && s.Press("##relative") && !CalCtx.continuousSync
					&& s.Press("##angular") && CalCtx.enableAngularVelocity && s.Press("##native") && CalCtx.enableNative;
			} },
			{ "settings", "Calibration Speed picks Slow and Very Slow", [](Session &s) {
				return s.Press("##Slow") && CalCtx.calibrationSpeed == CalibrationContext::SLOW
					&& s.Press("##Very Slow") && CalCtx.calibrationSpeed == CalibrationContext::VERY_SLOW;
			} },
			{ "settings", "Prediction Time follows the slider", [](Session &s) {
				return s.Press("##prediction", 0.5f) && std::abs(CalCtx.predictionTime - 5.0f) < 0.15f;
			} },
			{ "smoothing", "Smooth headset tracker toggles and sends the filter", [](Session &s) {
				return s.Press("##headfilter") && CalCtx.headFilterEnabled && Count.sendOneEuroParams >= 1;
			} },
			{ "smoothing", "Headset sliders stay locked until smoothing is on, then send the filter", [](Session &s) {
				int before = Count.sendOneEuroParams;
				return s.Press("##head Min cutoff", 0.0f) && CalCtx.headFilterParams.minCutoff == 5.0 && Count.sendOneEuroParams == before
					&& s.Press("##headfilter") && s.Press("##head Min cutoff", 0.0f) && std::abs(CalCtx.headFilterParams.minCutoff - 0.1) < 1e-6
					&& Count.sendOneEuroParams >= before + 2;
			} },
			{ "smoothing", "Relative Calibration sliders change the drift filter and send it", [](Session &s) {
				int before = Count.sendOneEuroParams;
				return s.Press("##drift Beta", 0.999f) && std::abs(CalCtx.driftFilterParams.beta - 2.0) < 1e-6 && Count.sendOneEuroParams > before;
			} },
			{ "korean-error", "Every character of the Korean driver error has a glyph", [](Session &) {
				return EveryGlyphPresent(preview::KoreanDriverError);
			} },
		};
	}
}

int RunChecks()
{
	int failed = 0, passed = 0;
	for (const auto &check : Checks())
	{
		bool ok = false;
		{
			Session session(check.scene, check.openProgress);
			ok = check.run(session);
		}
		printf("%s %-19s %s\n", ok ? "PASS" : "FAIL", check.scene, check.what);
		ok ? ++passed : ++failed;
	}
	printf("%d passed, %d failed\n", passed, failed);
	return failed ? 1 : 0;
}
