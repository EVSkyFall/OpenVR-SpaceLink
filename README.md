# OpenVR-SpaceOverride

OpenVR-SpaceOverride aligns SLAM-tracked headsets (Pico, Galaxy XR and similar) with lighthouse-tracked devices, without the drift that plagues traditional playspace calibration. Instead of computing a one-time offset between two tracking systems that slowly slide apart, it uses a Vive tracker rigidly mounted to the headset: after calibration, the driver stops using the headset's own pose and builds it from the tracker instead.

This puts the headset and all your other lighthouse devices on the same tracking system, so there's nothing left to drift against. The SLAM tracking is still there underneath, you just get proper alignment on top of it.

> [!NOTE]
> If you find bugs or issues, let me know over e-mail at `nyabsi@sovellus.cc` I will be responding to you when I have time. This software will receive updates on irregular basis but each update is guaranteed to improve the software in a way or another, thanks for using it even on it's current state.

> [!TIP]
> OpenVR-SpaceOverride **is not** OpenVR-SpaceCalibrator nor a *fork*, it uses a completely novel technique starting from v9. Older vesions had code used from space cal to workaround a technical limitation which has been resolved.

## What this fork changes

### The head tracker stays bound

Once a tracker has been calibrated as the head tracker, it stays bound until you press **Remove Calibration**. Losing tracking, turning the tracker off, restarting SteamVR or the overlay, and a failed or cancelled calibration all keep the binding.

- While the bound tracker is not tracking, the headset falls back to its own tracking with the last correction (or stops, if "Fallback to SLAM" is off) and picks the tracker up again as soon as it tracks.
- **Calibrate** with a bound tracker recalibrates that same tracker. It never switches to another one, and it waits if the tracker is not tracking yet.
- Calibration no longer aborts on its own. Tracking interruptions pause it, and low quality keeps collecting. Use **Cancel** in the progress window to stop it; the previous calibration stays in place.

### Finding the head tracker automatically

With **Find head tracker automatically** on (the default) and no tracker bound, the overlay looks for the head tracker by itself:

1. Turn on hand tracking in your streaming app (for example Virtual Desktop) and hold your lighthouse controllers.
2. Move your hands around a little. The overlay pairs each tracked hand with the lighthouse controller it holds and lines up the two tracking spaces roughly.
3. Look around naturally. The tracker that moves with your head and sits on it is picked, and calibration starts on its own. SteamVR shows a notification when the tracker is found and when it is ready.

Remove Calibration with this setting on starts the search again. Turn the setting off to stop it.

### Known limitations

- Automatic search needs hand tracking and lighthouse controllers held in the same hands. Without them, use Calibrate.
- A tracker is only accepted when it moves rigidly with the headset in more than one direction (look up, down and around), sits close to it, and is not clearly below it. A chest tracker is not picked even if your neck stays still.
- The search waits until the hand pairs also pin down height, which needs some wrist rotation besides turning.
- Very old profiles without a stored tracker serial that also use a device scale different from 1 are mapped back to raw poses only approximately (about a centimeter or two) during the automatic search.

Problems with these changes belong to this fork, not to the original author.

## Requirements

- Lighthouse system (or other equivalent)
- Rigid Tracker (i.e., Vive Tracker 3.0 or equivalent)
- Headset with SLAM or other positional tracking system

## Compatibility

| Streamer | Status | Notes |
| --- | --- | --- |
| PICO Connect | ✅ Works | |
| Virtual Desktop | ✅ Works | |
| ALVR | ✅ Works | |
| Meta Quest Link | ⚠️ Unconfirmed | Unconfirmed, let me know, email: nyabsi@sovellus.cc |
| Air Link | ⚠️ Unconfirmed | Unconfirmed, let me know, email: nyabsi@sovellus.cc |
| Steam Link | ✅ Works | |
| Display Port powered SLAM devices | ✅ Works | Such as: Pimax, PSVR2, HP Reverb G2, etc. |
| VIVE Hub | ✅ Works | VIVE Focus Vision, via e-mail |

## Troubleshooting

### My controllers jump, then settle back

Expected behavior, not a bug. 

Your controllers use the headset's inside-out tracking, which drifts relative to your lighthouse space, the driver corrects that drift and shifts back into place.

### My full body tracking seems weird in VRChat!

This is expected, there is issues with the VRChat IK system that is not obvious from first glance.

Usually Space Calibrator is not precise enough for these issues to show up, but with SpaceOverride you will see these issues.

Each Lighthouse HMD suffers from the same issues and this is unfortunately the reality we live in.

I am unable to solve these issues as they are not caused by SpaceOverride but I am working on potential workarounds.

## FAQ

### How is this different from TrackingOverride?

This is **not** TrackingOverride. TrackingOverride simply substitutes one device's pose for another's, which leaves you to deal with the offset between the tracker and the headset yourself. SpaceOverride instead **automatically calculates the proper offset** between the mounted tracker and the headset during calibration, then continuously reconstructs the headset pose from the tracker using that offset. The result is an aligned, drift-free pose rather than a raw pose swap.

### Does this conflict with OpenVR Space Calibrator?

No. SpaceOverride does not conflict with Space Calibrator, you can have both installed. They solve the alignment problem differently, but having Space Calibrator present won't break the override.

### Can wireless latency affect the pose?

Yes. The headset's display pipeline and the lighthouse tracker run on different clocks, so wireless streaming latency (and an unstable connection in general) can introduce a delay between your real head movement and the tracker-driven pose, which shows up as lag or swimming in your view. A solid Wi-Fi setup (or wired connection where possible) keeps this negligible.

### Does it drift?

No, all drift is induced by poor lighthouse performance, please ensure your lighthouse setup does not have interference.

You can use guide such as: [Link](https://www.notion.so/yeove/SteamVR-Hardware-Troubleshooting-Megathread-Setup-Guide-16fc956d336a8037b738d1b0b1ded2f0#1c0c956d336a8035b76dd1b87527d180)

## Acknowledgements

This project uses/used substancial parts of [OpenVR Space Calibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator). Huge thanks to [pushrax](https://github.com/pushrax) for their work, which this project used to build.

## License

Commits up to and including `1cc0583` are MIT (see [`LICENSE.MIT`](LICENSE.MIT)). Everything after is AGPLv3 (see [`LICENSE`](LICENSE)).
