# Pedro Pathing 2 -> 3 migration notes (BiobuzzPP3)

Status: compiles (`./gradlew :TeamCode:assembleDebug`). **Not yet run on the robot or simulator.**

## What changed

| Area | Pedro 2 | Pedro 3 |
| --- | --- | --- |
| Dependencies | `com.pedropathing:ftc:2.1.2`, `telemetry`, `org.solverslib:pedroPathing` | `com.pedropathing:revhub:3.0.1`, `tuning:1.0.1` (SolversLib core kept for commands) |
| Follower setup | `FollowerBuilder` + `FollowerConstants`/`MecanumConstants`/`PinpointConstants` | `new Follower(PinpointLocalizer, Mecanum, Foresight)` from `MecanumConfig`/`PinpointConfig`/`ForesightConfig` in `utils/Constants` |
| Pose | `com.pedropathing.geometry.Pose`, `getX()/getY()/getHeading()` | `com.pedropathing.math.Pose`, `x()/y()/heading()`; `Pose.zero()` instead of `new Pose()` |
| Paths | `pathBuilder().addPath(new BezierLine/Curve)...build()` -> `PathChain` | `Path`, built with `utils/PedroPaths.line/curve` (linear heading) or `lineTangent/curveTangent` |
| Follow a path | `follower.followPath(chain, holdEnd)`, `isBusy()` | `follower.holdEnd.set(..)`, `follower.follow(path)`, `following()`; wrapped by `commands/FollowPathCommand` |
| Teleop drive | `startTeleopDrive()`, `setTeleOpDrive(f, s, t, robotCentric)` | `follower.manual(f, s, t)` / `manual(ManualDrive.fieldCentric(f, s, t, heading))` |
| Start pose | `setStartingPose` | `setPose` |
| Tuning | `Tuning.java` (Pedro 2 OpModes) | `pedro/Tuning` + `pedro/procedures` (AutoTune web UI) |
| Simulator | Pedro 2 `Drivetrain`/`Localizer` subclasses | Pedro 3 `Drivetrain`/`Localizer` interfaces (`SimulatedMecanumDrivetrainForFollower`, `SimulatedLocalizer`) |

Removed: `commands/FollowPathCommandSLib` (unused), Pedro 2 `Drawing.drawDebug` / pose-history drawing.

## Tuning (required before competition)

Pedro 2 PIDF values, zero-power accelerations and velocities do **not** carry over. `Constants.foresightConfig` holds **placeholder** values estimated from the Pedro 2 tuning. Foresight has 12 required fields with no library default (an empty `new ForesightConfig(c -> {})` throws "config variable has not been set" at `ConfigVar.java:167`), so the placeholders only let the follower start; they are not tuned.

1. Build and install TeamCode, connect to the Robot Controller network, open <http://192.168.43.1:10158> (AutoTune).
2. Run **Mecanum Tuner**; paste the generated Java over `Constants.drivetrainConfig`.
3. Run **Pinpoint Tuner**; paste over `Constants.localizerConfig`. (Current values are carried over from Pedro 2: x offset 6.5, y offset -5, 4-bar pod, both directions FORWARD.)
4. Rebuild/install, run **Foresight Tuner**; paste over `Constants.foresightConfig`.
5. Rebuild/install, run **Tests** to verify localization, driving and path following.

Pedro 2 reference values (comment at the bottom of `Constants.java`): mass 7.17 kg, forward/lateral zero-power accel -42.92 / -52.68, xVelocity 80.73, yVelocity 58.54.

## On-robot checks

- [ ] **Teleop directions.** Positive strafe = left, positive turn = counter-clockwise (same as Pedro 2, values passed straight through). Verify strafe and turn aren't reversed in both field- and robot-centric modes.
- [ ] **`setGlobalMaxPower`.** Mapped to Foresight `maxPathSpeed` for that path only; units not confirmed. Check the 0.25 pickup paths crawl as before; adjust in `FollowPathCommand` if not.
- [ ] **Heading interpolation.** Paths with no heading set now use `.tangent()` (e.g. `parkFromStart` in the Raised Hive auto). Confirm the robot faces as expected.
- [ ] **Path end behavior.** `holdEnd=false` sets `follower.holdEnd` false; commands finish when `follower.following()` is false. Check parks and hand-offs between commands.
- [ ] **Start poses / alliance flip.** Autos call `follower.setPose(startPose)`. Confirm red and blue start poses.
- [ ] **Simulator.** Run the `*Sim` autos and `TestOpModeSim`. The sim drivetrain mirrors `Mecanum`'s wheel mixing; it picks up `foresightConfig` once tuned.
- [ ] **Panels.** Field drawing now draws the robot and a sampled path only (no pose history).
