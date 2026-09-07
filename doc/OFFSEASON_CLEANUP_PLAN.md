# FTC Shared-Code Improvement Plan (next season, 3 to 4 teams)

## Status and how to resume (read this first)

**Last updated 2026-09-07 (evening).** Repo: `/Users/georgemitchom/StudioProjects/FTC17651/FtcRobotController_Decode`
(GitHub fork `FTC16751/FtcRobotController_Decode`, default branch `master`). Work is on branch
`offseason/common-cleanup-2026`, open as [PR #1](https://github.com/FTC16751/FtcRobotController_Decode/pull/1).
Tag `pre-r6-reorg` marks the tree before the folder move.

| Step | State | Commits |
|---|---|---|
| R1 remove unreferenced Common code (DriveUtil2025 kept, slimmed) | done | d4b2708 |
| R2 Road Runner kept, moved to `roadrunner/` | done | d4b2708 |
| R4 one Alliance, one SharedState, CommonConstants | done | d4b2708 |
| R6 folder reorganization + Driver Station groups | done | b72f53d, 1d9f7a7 |
| R5 RobotConfig carries names + calibration; one config per chassis | done | c7cb5ca, 3cc8ac1 |
| **R3 delete superseded team code** | **ON HOLD by mentor decision** | |
| R7 LaunchController (P3, Skyline) + unit test harness | done | f054394, ba10258 |
| R8 aiming helpers (FlywheelVelocityModel, AimLed, VisionAim) + tests | done | 6c5e053 |
| R9 AutoSelector / AutoBase | **deferred**: build with the first new-season auto | |
| R10 TeleOpBase / ButtonEdge | **closed**, will not be done; gamepad layouts stay as drivers learned them | |
| R12 template: `teams/testteam2027` built as the template and test bed (see DriveUtil section) | done, untested on hardware | |
| R11 live defects (TeleOp-side items), R13-R15 | not started | |

**Hard rules learned from the mentor, do not violate:**
0. Demo-safe TeleOp defaults: launcher targeting starts in MANUAL/PRESET at the CLOSE setpoint, never
   vision; flywheel idle until the driver spins it up; vision is opt-in by a button. (Set 2026-09-07.
   GearGirls Bot 2 TeleOp fixed; P3 Bot 3 and Skyline V2 already complied.)
1. `@Disabled` means "hidden from the Driver Hub for event day," never "dead." Delete only files with
   zero callers or a newer replacement for the same robot, confirmed by diff, and ask first.
2. Prior-season robots are kept for demos. Road Runner, DriveUtil2025, the Into The Deep auto, the
   pushbot and StarterBot code all stay. "No callers in this season's code" is not grounds for deletion.
3. RobotConfig ("what the robot is") stays separate from each team's Constants ("how it operates").
   Two files per team. Never merge them.
4. R3 does not start until the mentor says so. Superseded files were moved with their teams
   (`old/`, `earlyideas/`, `bot1/`, `legacy/intothedeep/`) and are waiting.
5. **DriveUtil's reason to exist is a new programmer's first auto.** (Restated 2026-09-07.) It was
   started many seasons ago so helper functions would hide the complexity of moving a robot and a
   student could have a simple autonomous running quickly. Every change to it is judged by that:
   does the beginner's auto get simpler or stay simple? Direction in the name, inches, one speed,
   blocking with a time limit, a reached flag. The advanced idioms (`driveTo` waypoints, `drive_p3`,
   the tag approach) stay, but nobody should need them to write "drive forward 24, turn left 90."
6. **Keep the `OpMode` constructor parameter and `myOpMode` field in DriveUtil2026b, and keep the
   commented-out Pedro Pathing blocks and their imports.** (Mentor, 2026-09-07.) The OpMode hook is
   deliberately reserved; every robot passes null today and that is fine. Pedro did not work last
   season and will be revisited this season, possibly immediately; the commented blocks in the
   constructor, `arcadeDrive`, `fieldCentricDrive`, and `update()` are the starting point for that.
   Neither is "dead code" for cleanup purposes.

**Open items from R5 for the mentor:** GGRobot (Bot 1) shares GGBot2Config; P3_Robot (Bot 2) shares
P3Bot3Config; every chassis still has the shared default Calibration (1.15 right-rear scale etc.) and
should measure its own; Skyline has no Constants class; no PushbotConfig (pushbots stay on
DriveUtil2025 with phantom `limelight`/`odo` config entries, mentor is fine with that).

**Hardware checks still owed:** see `doc/ROBOT_TEST_PLAN.md` (added 2026-09-07) for the full
checklist with expected values. In short: each competition robot drives with its RUN ME TeleOp
and shows the right `robot config` name in telemetry; auto with BLUE hands BLUE to the following TeleOp
and the Limelight targets the blue goal; a pushbot and a StarterBot show up under `Demo` and drive.

**Mechanics that matter when resuming:** compile with
`JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:assembleDebug`
from the repo root (Gradle's incremental compiler sometimes needs `:TeamCode:clean` after adding a
file). BSD sed lacks `\s` and `\b`; use perl. OpMode names contain parentheses, so any annotation
rewrite must be quote-aware. `gh pr` needs `--repo FTC16751/FtcRobotController_Decode` and a
`FTC16751:` head prefix because the repo is a fork.

## Where things stand (end of 2026-09-07 session)

DriveUtil2026b is reorganized by tier (Beginner, Intermediate, then the advanced idioms), down
from 1450 to 1186 lines, with 102 laptop unit tests covering its math and the tag approach.
Nothing from today has run on a robot yet; `doc/ROBOT_TEST_PLAN.md` is the ordered checklist and
`teams/testteam2027` is the test bed (its config matches Skyline's chassis, so those OpModes run
on Skyline's Control Hub unchanged). Remaining DriveUtil items, in order: the robot session; the
Advanced tier (`startPath`, `relocalizeFromTag`, `startMoveRelative`) once the Pinpoint square and
the tag-approach sign check pass; the Pedro revisit from the commented blocks (hard rule 6); R13
as a one-page table of the tiers. The detailed record of what was found and done is the section
below.

## Next focus (set 2026-09-07): analyze and improve `common/DriveUtil2026b`

The mentor's next session is a dedicated look at the shared drive utility. Start from these facts,
gathered by the 2026-09-06 audit and the R1/R5 work, rather than re-discovering them.

**What it is.** `common/DriveUtil2026b.java`, 1,407 lines, 94 public members, the only drive class
live team code uses (DriveUtil2025 is a 137-line legacy stub for the demo pushbots). Every robot
class builds it as `new DriveUtil2026b(hardwareMap, telemetry, null, config)` and calls
`drive.update()` each loop. It bundles four generations of ideas in one file:
1. TeleOp mixing: `moveRobot` (with `config.calibration.rightRearPowerScale`), `arcadeDrive`,
   `fieldCentricDrive`.
2. Encoder RUN_TO_POSITION moves: `drive_p3`, `driveRobotDistance*`, `rotateRobot`,
   `driveRobotToPosition` (Skyline's autos use only these).
3. Phil Malone "simplified odometry" blocking moves: `simplifiedOdometryDrive`/`strafe`/`turnTo`
   using `ProportionalControl` (extracted in R1) and 4-wheel encoder dead reckoning.
4. Pinpoint point-to-point PID: `driveTo(current, target, power, holdTime)` + `PinpointPIDLoop`
   (P/I/D, filtered D, asymmetric accel limit, heading wrap). This is the idiom the autos live on:
   GearGirls 347 `driveTo` calls, P3 135. Tuned per robot via `RobotConfig.pointToPointTuning`.
Plus `driveRelative`, `calculateAutoAimTurn`, `Angle`/`Vec2` helpers, and an `update()` state machine
that is mostly comments.

**Known problems, with evidence (line numbers as of commit 2b13e01; grep to confirm):**
- Blocking helpers (`driveRobotToPosition`, `simplifiedOdometryDrive`, `strafe`, `turnTo`) loop on
  `while (areMotorsBusy())` / `while (readSensors())` with no `opModeIsActive()` check, because the
  OpMode parameter is always null. A stalled motor hangs the OpMode until the Driver Station stops it.
- `driveToTagAsync` sets `driveState = ALIGNING_TO_APRILTAG` but `update()` has no handler for it
  (only a comment), so `isBusy()` stays true forever after one call.
- `getOdoPosition()` writes five telemetry lines every call; P3 autos call it up to 19 times per
  file as a plain getter. GearGirls bypass it and reach into the public `drive.pinpoint` field
  directly (571 accesses), so the odometry device is effectively public API.
- `arcadeDrive(strafe, drive, turn, rightStickY, speed)` changed meaning between generations
  (DriveUtil2025 negated Y internally; 2026b does not). Each TeleOp compensates differently: P3
  passes `-driveInput` at speed 0.25, Skyline negates turn at 0.80, GearGirls negates the stick at
  1.0 with a deadband. Not a bug, but a trap for a fourth team. Do NOT change gamepad layouts to fix it.
- Two turning-circle numbers for the same robot: `robotDiameterCm` 60 (used by `rotateRobot`) vs
  `turnCircumferenceIn` 27.5 (used by `drive_p3`); and two ticks-per-rev values (537 for the 312 rpm
  drive motors, 384.5 for the 435 rpm simplified-odometry path). Both now live in
  `RobotConfig.Calibration` but every robot still carries the shared defaults; nobody has measured.
- `rightRearPowerScale` 1.15 is applied in `moveRobot` for every robot; it was tuned on one.
- Inner class `SimplifiedOdoDriveUtilProportionalControl` (after `PinpointPIDLoop`) duplicates the
  extracted `ProportionalControl`; check whether anything uses it.
- Commented-out Pedro follower blocks and three Pedro imports remain (the only Common-to-Pedro coupling).
- Public mutable fields (`driveController`, `strafeController`, `yawController`, `pinpoint`, motors).
- `pidReset()` at class level is an empty stub.

**Constraints to respect:** Pinpoint is optional (`hasPinpoint()`, R5); encoder-only robots must
keep working; GG autos read `drive.pinpoint` directly so that field cannot silently go away; Java 8;
demo season, so behavior changes need a robot on a stand; do not change gamepad layouts.

**Testing angle:** the pure math (`PinpointPIDLoop.calculateAxisPID`, `Angle.normDelta`, the
`moveRobot` mixing, `drive_p3` tick math) is unit-testable today with the R7/R8 pattern once seams
exist; `driveTo` can be tested with a fake pose source. Tests first, then restructure.

**A reasonable shape to aim for** (decide in the session, not here): split by idiom into
`MecanumMixer` (TeleOp math), `EncoderMoves` (RUN_TO_POSITION), `PointToPointDrive` (Pinpoint PID),
with `DriveUtil2026b` kept as a thin facade so the 480+ call sites do not change.

### 2026-09-07 session: analysis results and the first two fixes

**Framing correction (mentor).** "Zero callers in the repo" does not mean dead. The repo holds about
one season of OpModes plus a few older ones; the utility is meant to serve past and future OpModes
with simple commands (drive forward N inches, turn, strafe) so new programmers come up to speed.
Cleanup means coding mistakes, optimizations, and improvements to that simple-command surface.
Remove code only when it is broken and unused (the Phil Malone simplified-odometry block) or a
plain duplicate. The facade split above is no longer the goal; drop it unless the file stays
unwieldy after the removals.

**What is live today (not `@Disabled`, not in `old/` or `earlyideas/`):** TeleOp mixing on all
three teams (GearGirls Bot1 TeleOp uses `fieldCentricDrive`, so that one needs Pinpoint heading);
Pinpoint `driveTo` in GearGirls and P3 autos (every call passes `drive.pinpoint.getPosition()`);
`drive_p3` in Skyline's four autos. Nothing live calls `turnTo`/`strafe`/`simplifiedOdometryDrive`
(the P3 `turnTo` lines are commented out; the one GearGirls early auto swapped its `strafe` call for
an encoder strafe), `driveToTagAsync` (one disabled caller), or `driveRelative`.

**Coding mistakes found:**
- `driveRobotDistanceForward` passed a tick count divided by 25.4 into `drive_p3` as inches, so
  "drive forward 12 inches" drove about 21. Backward and the two strafes were fine. **FIXED
  2026-09-07**: builds the tick array like its siblings.
- `driveRobotToPosition` spun in `while (areMotorsBusy())` with no interrupt check, no timeout, no
  sleep. The SDK interrupts the OpMode thread on Stop; the loop ignored it, so the stuck-OpMode
  watchdog would restart the app. Every simple encoder command and Skyline's autos go through it.
  **FIXED 2026-09-07**: checks `Thread.currentThread().isInterrupted()`, sleeps 10 ms per poll, and
  has a time limit (three times the ideal travel time at the requested power plus 2 s, never under
  3 s; a three-argument overload takes an explicit limit). Returns true if the targets were reached.
  Needs a robot on a stand: run one `drive_p3` move and confirm nothing got slower, then hold a
  wheel and confirm the step ends on its own. `drive_p3`, `rotateRobot`, and the
  `driveRobotDistance*` family now return that reached-or-timed-out flag (void before; callers
  that ignore it compile unchanged). `common/test/EncoderMoveCheck` is a one-button TeleOp for
  these checks and the calibration measurements; each team has a three-line subclass with its
  config (`GGEncoderMoveCheck`, `P3EncoderMoveCheck`, `SkylineEncoderMoveCheck`, and a StandardBot one).
- Simplified odometry has inverted feedback on two of three axes. `moveRobot` is positive-right
  for strafe and positive-clockwise for yaw (the `drive_p3` mixing and Javadoc agree). Phil's
  feedback side is positive-left (`updateMotion`) and the IMU is positive-counter-clockwise, so
  `strafe` and `turnTo` push away from the setpoint and never exit. The Pinpoint path knows this
  and flips signs in `driveTo`. Explains why it was never used. **DELETED 2026-09-07**:
  `simplifiedOdometryDrive`, `strafe`, `turnTo`, `readSensors`, `startMotion`, `updateMotion`, the
  three `ProportionalControl` fields and their constants, the duplicate inner
  `SimplifiedOdoDriveUtilProportionalControl`, `Calibration.odometryTicksPerRev`,
  `common/ProportionalControl` (no other user), and `common/test/SampleAuto_usingSimplifiedOdometry`.
  The disabled `GGAutonomous001` early auto now uses `drive_p3(-24, 0, 0, 0.5)` for its one call.
  The public `heading` field stays: ten P3 autos (including the live QueueBot3) show it as
  "imu heading" in telemetry, and since only `readSensors` ever wrote it, it has always read 0.
  Mentor decision 2026-09-07: leave those autos alone. No action planned; the same screens already
  show the Pinpoint heading via `getOdoPosition()`.
- `driveToTagAsync` sets a state `update()` never handles, so `isBusy()` stays true forever.
  Mentor 2026-09-07: this is wanted, not dead. The idea is non-blocking "drive to X inches in
  front of tag N" so other subsystems keep running, and FTC adds tags every season.
  **Step 1 DONE 2026-09-07:** `common/TagApproach` (pure math and state, 23 tests) plus the
  `common/TagSighting` interface (robot-frame view of one tag: forward inches, right inches,
  square-up degrees; signs match `moveRobot`). P per axis, clamp, optional min power, tolerance,
  hold time, coast on last powers when the tag drops out, lost timeout, hard max time; states
  IDLE/APPROACHING/HOLDING/DONE/LOST/TIMED_OUT.
  **Steps 2 and 3 DONE 2026-09-07:** `VisionUtil implements TagSighting` from the Limelight's
  target-pose-in-robot-space; the four axis-sign constants at the top of that section are the one
  thing needing a stand check (test plan H1) and the only place to fix a sign.
  `DriveUtil2026b.driveToTagAsync(sighting, tagId, standoffIn, holdSec)` starts a `TagApproach`,
  `update()` steps it under `ALIGNING_TO_APRILTAG` and feeds `moveRobot`; `isBusy()` reflects it;
  `cancelDriveToTag()`, `lastTagApproachSucceeded()`, `getTagApproach()` added; the six write-only
  fields and `holdTimer` deleted. Gains are `RobotConfig.tagApproach` (a `TagApproach.Settings`
  per chassis, default gentle). The old `driveToTagAsync(VisionUtil, ...)` call in the disabled
  GGAutonomous001 compiles unchanged because VisionUtil is a TagSighting. The
  relocalize-then-`driveTo` alternative already exists on GearGirls (`GGRobot2.resetOdometryToVision`).
  **Not yet on a robot.** First user: `teams/testteam2027` (below).

**R12 trial run, 2026-09-07: `teams/testteam2027` (robot `test2027bot`).** Built as the new-team
template and as the test bed for the day's Common work, without touching `common/` to make it fit.
`Test2027BotConfig` (numbered fill-in comments), `Test2027Constants`, `Test2027Robot` (drive +
vision, `update()`, `stopAll()`, `addTelemetry()`), `teleop/Test2027Teleop` (sticks, slow mode,
hold RB to run the tag approach interactively, Back resets Pinpoint), `auto/Test2027DriveSquareAuto`
(the `driveTo` waypoint idiom in its smallest form, with per-step timeouts),
`auto/Test2027TagApproachAuto` (start async approach, wait on `isBusy()`), and
`test/Test2027EncoderMoveCheck`. `teams/testteam2027/README.md` is the HOWTO-new-team the R12
item asked for: copy and rename, fill in the config in six numbered steps, drive, measure, prove
the Pinpoint, prove the tag approach, add the game. Driver Station groups `TestTeam2027` and
`TestTeam2027 Test`. This folder IS the template; `teams/_template/` is not needed as a separate
thing. Still owed: everything in doc/ROBOT_TEST_PLAN.md sections B, F, H on the actual chassis.
- Two turning circles: `rotateRobot` used `robotDiameterCm` 60 (74 in circumference), `drive_p3`
  uses `turnCircumferenceIn` 27.5. Factor 2.7 apart. Skyline's live Score3Preloads autos turn 30
  degrees through the 27.5, so that number was tuned against that auto; physically it implies a
  robot under 9 in across, so it is almost certainly a compensated value (a 14 x 14 in layout works
  out near 88). **DONE 2026-09-07:** one field, `turnCircumferenceIn`; `rotateRobot` delegates to
  `drive_p3`; `robotDiameterCm` gone. Default left at 27.5 by mentor decision so Skyline's live
  auto does not change; each chassis measures its own (test plan F). In the same pass the
  `driveRobotDistance*` family now converts through `encoderCountsPerInch` like `drive_p3`, and
  `encoderTicksPerRev`, `gearReduction`, `wheelDiameterCm` left the config. That does NOT assume
  every robot has the same encoders: each config carries its own counts-per-inch, and
  `Calibration.countsPerInch(ticksPerRev, gearReduction, wheelDiameterMm)` gives a starting value
  from any motor and wheel spec. No live behavior change: `rotateRobot` had no callers and the
  distance family's old arithmetic agreed with the new within a quarter percent.
- Small: `turnTo` never reset `holdTimer` before its loop (moot if deleted); `PinpointPIDLoop`
  returns 0 on the first call after every reset and `calculatePID` resets an axis whenever it is
  inside tolerance, so an axis that drifts back out loses one loop.

**Optimizations:**
- `getOdoPosition` writes five telemetry lines per call; P3's queue auto calls it 18 times per
  file. Move the lines into `addTelemetry`. **DONE 2026-09-07:** it now returns `getPose()` and
  writes nothing; the three position lines were already in `addTelemetry`, which every robot
  class calls once per loop; the two pod-direction lines are gone (config, not match, data).
- `rightRearPowerScale` 1.15 is applied before normalization, so at full stick the other three
  wheels cap at 87%. Fine if a relative correction is the intent; every config carries it.
- `driveTo(target, power, holdTime)` overload that reads its own pose: 491 live calls pass
  `drive.pinpoint.getPosition()`, and 494 of the 571 direct `pinpoint` accesses are that call.
  The public field stays.
- Duplicates: `stopRobot`/`stopMotors`, `setMotorMode`/`setMotorRunMode`, four angle-wrap
  implementations (`normalizeAngle`, `Angle.normDelta`, `Angle.normDeltaDeg`, the loop in
  `ProportionalControl`). **DONE 2026-09-07:** `stopMotors` is an alias of `stopRobot`,
  `setMotorRunMode` folded into `setMotorMode`, only `Angle.normDelta` remains (tested).
- Dead private code: `calculateEncoderCountsPerDegreeOfChassisRotation`, `calculateTankOutput`,
  `sensorDistance`, enums `DriveType`/`DriveMotor`, `errorR`, `DRIVE_SPEED`. **DONE 2026-09-07**,
  plus the empty class-level `pidReset`, `resetActionTimer`, the `pathComplete` triad, the
  never-checked `InBounds` enum (`inBounds` now returns a boolean), and the superseded
  commented-out copy of `driveTo`. Per hard rule 6 the `OpMode` hook and every Pedro block and
  import stay. `driveRelative` stays for the Advanced tier. File is 1186 lines, no behavior change.

**Improvements to the simple-command surface:**
- Write the sign convention once at class level: encoder commands and the TeleOp mixer are
  positive-right, positive-clockwise; the Pinpoint path is field-frame counter-clockwise like the SDK.
- Add `turnLeft(degrees, speed)` / `turnRight(degrees, speed)` delegating to `drive_p3` on the one
  turning-circle number, to match `driveRobotDistanceStrafeLeft`/`Right`. **DONE 2026-09-07.**
  Both take the size of the turn (sign ignored, so `turnLeft(-90)` still turns left). The Encoder
  Move Check's D-pad left/right now call them, so the stand test exercises the exact methods
  students will use. The testteam2027 README lists the beginner vocabulary.
- Every blocking command takes an optional timeout (the encoder path now does).
- Two heading sources: `getHeading` is IMU degrees, `getPinpointHeading` is Pinpoint radians;
  `fieldCentricDrive` uses the Pinpoint one, so a robot without a Pinpoint silently drives
  robot-centric. Pick one for TeleOp and say so.
- Testability without behavior change. **DONE 2026-09-07:** `common/PinpointPIDLoop` is its own
  class (moved out verbatim; 15 tests: first-call init, clamp, accel limit, unlimited braking,
  limited reversal, settle, integral cap, filtered D, dt floor). `common/MecanumMixer` holds
  `mix(drive, strafe, yaw, rightRearScale)` and `fieldToRobot(fieldForward, fieldLeft, heading)`;
  `moveRobot`, `fieldCentricDrive`, and `driveTo` call them (16 tests, including two that check
  the new rotation against the exact old formulas of both callers). `common/EncoderMoveMath.ticksFor`
  holds `drive_p3`'s arithmetic with its per-component truncation preserved (9 tests, expected
  values are the pre-change tick counts). `AngleTest` covers `Angle.normDelta` in place. 102 tests
  total. The sign conventions (forward, right, clockwise) are now stated by tests, not by comments.

**Beginner tier DONE 2026-09-07 (hard rule 5).** DriveUtil2026b opens with a class comment
pointing at a BEGINNER COMMANDS section placed right after the constructor: `driveForward`,
`driveBackward`, `strafeLeft`, `strafeRight`, `turnLeft`, `turnRight` (inches or degrees, sign
ignored, optional speed), `waitSeconds`, `stop`, and a blocking `driveToTag(vision, id, inches)`.
Default speeds come from `setDefaultSpeeds(drive, turn)`, which the robot class calls from the
team's Constants, keeping "how it operates" out of RobotConfig. The long-named
`driveRobotDistance*Inches` family stays underneath for the old autos. `Test2027BeginnerAuto` is
the eight-move LinearOpMode a new programmer copies first; the testteam2027 README has the
vocabulary table (Step 6b). Encoder Move Check bumpers use `driveForward`/`driveBackward`.

**Tier plan agreed 2026-09-07** (mentor: an advanced programmer is still a high schooler):
- Intermediate ("the robot knows where it is"): `move(fwd, right, turn)` naming `drive_p3`;
  `getX/getY/getHeadingDegrees`, `setPosition`, `resetPosition`; `startDriveTo(x, y, heading)` +
  `isBusy()` + `cancel()` reading its own pose with a timeout; `turnToHeading(deg)`;
  `startDriveToTag`; `resetFieldForward()` for field-centric TeleOp.
- Advanced ("follows a plan and proves it"): `startPath(List<Pose2D>)` with per-step timeouts
  (the R9 idea, inside the drive); `relocalizeFromTag(vision)` (GGRobot2.resetOdometryToVision
  promoted); `startMoveRelative`; the tuning surfaces; tests as the definition of correct;
  `moveRobot`/`setMotorPowers` documented once.
- Mentor tier already exists: pedropathing/, roadrunner/, PID internals, VisionUtil sign constants.
- One object `robot.drive`, file ordered by tier; R13 becomes a one-page table of the tiers.

**Intermediate tier DONE 2026-09-07.** New section in DriveUtil2026b after the beginner one:
`getX/getY/getHeadingDegrees/getPose`, `setPosition/resetPosition`, static `pose(x, y, heading)`,
`move(fwd, right, turn)` naming `drive_p3`, `startDriveTo(x, y, heading[, power])` (also `Pose2D`
forms and a full form with hold and timeout), `turnToHeading`, `startDriveToTag`, one `cancel()`
for either async move, `lastMoveSucceeded()`, `resetFieldForward()` (only `fieldCentricDrive`
honors the offset; autos keep the Pinpoint frame). `startDriveTo` defaults power from
`setDefaultSpeeds` and hold from `setDefaultHoldTime` (mentor decision), resets the three PID
loops, and has a time limit scaling with distance; `update()` now handles
`DRIVING_TO_POINT_PINPOINT` by calling the existing per-loop `driveTo`, so the 480 existing
call sites are untouched. `driveToTagAsync` now replaces a running move instead of being ignored.
`Test2027DriveSquareAuto` rewritten in this vocabulary; README Step 6c is the table. No Pinpoint:
getters return 0 and start* moves finish at once, failed.

**Suggested order for the rest:** `startPath` and `relocalizeFromTag` (Advanced tier) after the
Pinpoint square and the tag approach have been on a stand. Then the Pedro revisit the mentor
plans, starting from the commented blocks.

## Context

One software mentor supports three FTC teams (GearGirls, P3, Skyline) from a single TeamCode repo at
`/Users/georgemitchom/StudioProjects/FTC17651/FtcRobotController_Decode`. Shared code lives in
`utilities/Common`; each team has its own `utilities/<Team>`, `TeleOp/<Team>`, and `Auto/<Team>` folders.
The robots have different subsystems, but the mentor wants identical drive, similar vision, and a
similar structure so mentoring scales. A fourth team may be added next season.

The user asked for an analysis of Common and the team differences, with recommendations for next
season. This document is that analysis. No code was changed.

**How this was produced.** Six read-only agents each read one area in full (Common drive, Common
support, GearGirls, P3, Skyline, everything else) and verified claims with grep and diff. A second
pass of cross-team comparators and adversarial verifiers did not run because the session usage limit
was hit. Every claim below cites a file and line from the reader evidence, but the recommendations
have not had an independent second check. Confidence is high on "what exists and who calls it" and
medium on effort estimates.

**A note on `@Disabled`.** The mentor uses `@Disabled` deliberately to keep OpModes off the Driver
Hub before events so drive teams have fewer choices to think about. That is good practice and this
plan keeps it. `@Disabled` is therefore **not** treated as evidence that code is dead. The deletion
criteria used below are only: (a) a class or OpMode has zero callers and zero registered entry points,
or (b) it is an older copy of a file that a newer version has replaced (same purpose, same robot,
newer file is the one in use). Where a count below says "superseded," it was measured by diff and
call-site grep, not by the annotation. A file that is merely `@Disabled` for event hygiene is not on
any deletion list.

## The short version

The three teams already converge on one good architecture: a per-team Robot hub class that owns
Common's DriveUtil2026b, VisionUtil and LedUtil, built from a RobotConfig factory, with thin OpModes
that delegate to it. That is the right shape and should be kept. The problems are around it:

1. **Roughly half of the ~50,000 lines are older copies or have no callers.** A middle-generation
   drive class with zero callers (`DriveUtilDepricated`), two unused motion-planning stacks, and 30+
   OpModes that a newer file has replaced all still build and still appear in the IDE. (`DriveUtil2025`
   is a deliberate exception: it is still used by several OpModes and stays.)
2. **Versioning is done with filenames instead of git.** `DriveUtil2025` / `DriveUtilDepricated` /
   `DriveUtil2026b`, `IntakeSensorFusion` / `001` / `002`, `Score9_v4` / `_v5` / `_v7`, `old/` folders.
   A student opening the team folder sees several files with the same purpose and no marker for
   which one the robot runs.
3. **The same six things are hand-copied into every robot class and OpMode:** launcher state machine,
   flywheel lookup, LED aim indicator, alliance selector, TeleOp handler skeleton, SharedState.
   Copies have drifted into real bugs.
4. **Hardware names and fudge factors are hardcoded in Common,** so a fourth team must edit shared
   code and name its Control Hub devices exactly like GearGirls.
5. **Dependencies point the wrong way.** Common imports team code; Skyline imports P3 and GearGirls.

## Key numbers

| Measure | Value | Source |
|---|---|---|
| Total TeamCode lines | ~49,800 | wc over all .java |
| Common drive area with zero callers | ~3,380 of 7,843 lines | DriveUtilDepricated, EssentialMecanumRobot, SimplifiedOdometryRobot, EncoderOdometry, GBP2P/DriveToPoint (DriveUtil2025 excluded; it is still used) |
| prismled folder, used only by two @Disabled tests | 1,659 lines | Common/prismled |
| GearGirls auto code in older lineages that v7 replaced (Bot1/, BOT2/old/, earlyIdeas/) | 7,484 of 7,916 lines (94%) | diff against GGAutonomous_Score9_v7 |
| P3 code belonging to the Bot1/Bot2 lineages that Bot3 replaced, or with zero callers | ~6,500 of 10,600 lines | grep call sites; diff Bot1 vs Bot2 vs Bot3 |
| Road Runner quickstart, fully unused | ~2,100 lines + 4 Gradle deps | root package, messages/, tuning/ |
| Prior season, referenced by nothing | 1,778 lines | utilities/PriorSeason |
| OpModes in repo / enabled / enabled but not team code | 90 / 21 / 9-10 | grep @TeleOp, @Autonomous, @Disabled |
| `enum Alliance` definitions | 13 | grep |
| `SharedState.java` byte-identical copies | 3 | diff |
| Robot hub classes reimplementing the same glue | 6 | GGRobot, GGRobot2, P3_Robot, P3_Robot_Bot1, P3_Robot3, Skyline_Robot |
| Auto files with the same X=Blue/B=Red/Y=Close/A=Far init_loop block | 18 | grep |
| TeleOps with the same handler skeleton and IntakeState toggle | 15 | grep |

## What is working well (keep these)

- **Robot hub pattern.** `GGRobot2:38-48,98`, `P3_Robot3:30-38,101`, `Skyline_Robot:35-44` all build
  `new DriveUtil2026b(hw, tel, null, RobotConfig.createXConfig())` and expose `update()`, `stopAll()`,
  `addTelemetry()`. This is the de facto template.
- **RobotConfig injection** for motor directions, Pinpoint offsets, IMU orientation, and P2P tuning
  (`RobotConfig.java:26-126`). Right idea, just incomplete.
- **Non-blocking `driveTo` idiom** in autos: `if (robot.drive.driveTo(pose, wp, power, hold)) state = NEXT;`
  used 480+ times. Good teaching pattern.
- **GearGirls v7 auto** (`GGAutonomous_Score9_v7:38-55, 338-380`): one unified `PathState` enum plus
  `setPathWaypoints(alliance, location)`. Replaced four copied path methods. Best auto shape in the repo.
- **P3 Queue auto** (`P3Autonomous_Queue:33-107, 561-603`): a `Queue<PathState>` script with per-state
  timeouts and a global timeout forcing PARK. Robot-agnostic scaffold.
- **P3_Robot3 launch sequence** (`P3_Robot3:194-275`): spin-up timeout, velocity tolerance %, stall
  abort, shot counters. Most robust of the six copies.
- **ShotSequenceControllerV2** is a reusable non-blocking sequence built only on robot primitives.
- **Driver Station hygiene:** `(RUN ME)` suffix, per-team `group=`, `@Disabled` applied before
  events so drive teams see only what they need, `preselectTeleOp` on autos. Informal but effective,
  and it gets easier once there are fewer files to sweep.
- **Bench-test OpMode per subsystem** (IntakeUtilTestOpMode, LaunchFlippersTestOpMode, etc.).
- **VisionUtil** as the single Limelight facade; **InterpolatingLookupTable** already in Common.

## Recommendations

Priority: **P1** = before next season starts (mentor, off-season). **P2** = first weeks of the season
(mentor with students). **P3** = when convenient.

### P1. Cleanup and structure (mentor-led, off-season)

**R1. Delete the unreferenced Common drive and support code. DriveUtil2025 stays.** Medium effort,
one afternoon plus a compile.
`DriveUtil2025` is still used by OpModes the mentor runs (`TeleOp/Other/Coachbot`, `BasicDriveTeleop`,
`BasicOpMode_Linear1`, and `PriorSeason/Robot`), so it and those OpModes are **not** on this list.
Order matters because of one hidden dependency: `DriveUtil2026b:121-123`, `EssentialMecanumRobot:55-57`
and `SimplifiedOdometryRobot:53-55` all instantiate the package-private class
`DriveUtilProportionalControldepricated` defined at the bottom of `DriveUtilDepricated.java:1786`.
Steps:
1. Move that class into its own file `Common/ProportionalControl.java` and point DriveUtil2026b at it.
2. Delete `DriveUtilDepricated` (zero callers once step 1 is done; note it is newer than DriveUtil2025
   despite the name), `EssentialMecanumRobot`, `SimplifiedOdometryRobot`, `EncoderOdometry`,
   `GBP2P/DriveToPoint`, `GBPoint2Point` (empty), `RobotConfigorig` (zero callers, values have drifted
   from RobotConfig), `LimeLightVisionUtil` (zero callers), `Datalogger` (zero callers).
3. `utilities/PriorSeason/` has no callers outside itself. Confirm with the mentor before removing;
   if kept, it is fine as is since DriveUtil2025 is staying.
4. Delete `Common/prismled/` unless the Prism LED is planned for next season; if it is, keep only
   `GoBildaPrismDriver` and delete the duplicate `PrismI2c`, dead `LedPattern`, `PrismLedSubsystem`.
5. Remove the unused imports this exposes in `GGRobot2:17-19`.
6. Optional, since two drive utilities now coexist on purpose: add a one-paragraph header comment to
   each saying which robots and OpModes use it, so students pick the right one.
Risk: low. Every deleted class has zero callers per grep. Verify with the Gradle compile.

**R2. Keep Road Runner and Pedro Pathing; move them out of the way.** Small effort.
*Revised 2026-09-06 after the mentor's input.* Prior-season robots are kept for demos and still run
the Road Runner quickstart through `Auto/GearGirls/earlyIdeas/Score2SpecimenPinPointAuto` and the
Into The Deep TeleOp it preselects, so the quickstart (8 root-package classes, `messages/`, `tuning/`),
its four Gradle lines in `TeamCode/build.gradle:35-38`, and the `maven.brott.dev` repo stay. The teams
will not go back to Road Runner for competition; they use their own PID drive-to-point and may
revisit Pedro Pathing this season, so Pedro stays too.
What to do instead:
- **Done 2026-09-06:** the eight Road Runner classes plus `messages/` and `tuning/` now live under
  `roadrunner/` (packages `teamcode.roadrunner`, `.roadrunner.messages`, `.roadrunner.tuning`), and the
  one import in Score2SpecimenPinPointAuto was updated. The root teamcode package has no loose files.
  Build verified.
- Keep `pedroPathing/` as is for the possible re-attempt. If Pedro is dropped later, the only Common
  coupling left is the three imports at the top of `DriveUtil2026b` for commented-out follower code.
- Keep the bylazar Panels dep (`build.dependencies.gradle:22`); the enabled P3 TeleOp uses it.
Risk: low for Road Runner. Pedro removal requires first dropping `DriveUtil2026b`'s imports of
`pedroPathing.Constants` and `com.pedropathing` (`DriveUtil2026b:5-6,24`), the `RobotConfig.PedroPathingConfig`
field, and the `import static pedroPathing.Tuning.follower` at `DriveUtil2025:7`. That static is only
used by a private Kalman method DriveUtil2025 never calls (`:1562-1592`), so the method and the import
can go together without changing DriveUtil2025's behavior for the OpModes that use it.

**R3. Archive superseded team code in git, then delete it from the tree.** Medium effort.
The criterion is strictly "a newer file for the same robot replaced it" or "zero callers." Nothing is
listed here because it is `@Disabled`; keep using `@Disabled` for event-day hygiene exactly as today.
Tag the current state (`git tag decode-2026-season-end`) so nothing is lost, then delete:
- GearGirls: `Auto/GearGirls/Bot1/` (5 autos for the Bot1 hardware, replaced by the Bot2 v7 lineage), `Auto/GearGirls/BOT2/old/`,
  `Auto/GearGirls/earlyIdeas/`, `GearGirlsBot2_test`, `IntakeSensorFusion` and `001`, `IntakeUtil`
  (V1), `Spinner` (student scaffold with invalid servo position), `ShotSequenceController` (V1),
  `ShotVolleyController` (zero refs), `AutoAction`, `GGRobot` (Bot1) and `LaunchIndexer` if Bot1 is retired.
- P3: `TeleOp/P3Robot/old/` (8 files), `Auto/P3/Bot1/`, `Auto/P3/Bot2/`, `P3_Robot`, `P3_Robot_Bot1`,
  `P3_TurretUtil`, `P3_TurretUtil_Velocity`, `P3_IndexerUtil`, `P3_HoodServoUtil`, `P3_Teleop`,
  `TurretTeleOpExample`, `MinimalServoTest`, and `Auto/P3/.DS_Store`.
- Skyline: `SkylineAuto` (unmodified goBILDA StarterBot with tank-drive names, 26% of the team's code),
  `Skyline_Teleop` (V1), `utilities/Skyline/SharedState`.
- Other: `Auto/Other/Delete.java` (broken, NPE at init), `TeleOp/Concepts/PedroPathTeleopCoachTest*`,
  `IntakeSensorTest`, `ColorAveragingSensor`, `Test_IntakeSensorFusion` (tests a class the robot no
  longer uses), duplicate `PIDMotorTunerTutorial`.
Then adopt the rule: **no `old/` folders, no `_v7`, `V2`, `001`, `_test`, `_improved`, `_FORTEST`
suffixes.** A superseded file is deleted in the same commit that replaces it. Git history and tags
per event (`git tag qualifier-1`) are the archive.
Risk: a team may want to revive Bot1 or Bot2 hardware. Mitigation: the tag. Also confirm with each
team which robot is going to next season before deleting hub classes.

**R4. One Alliance, one SharedState, one place for game constants.** Small effort, high payoff.
*Done 2026-09-06, build verified.* `CommonConstants` now holds `Alliance`, `Location`, `METERS_TO_INCHES`,
`Field` (goal coordinates) and `Limelight` (pipeline contract). `SharedState` lives in Common; the three
team copies are deleted. The team-local and VisionUtil `Alliance`/`Location` enums and
`GGRobotConstants.GoalLocation` are deleted and every user repointed. The two Common-to-team imports are
gone. `MotifPattern` was deliberately left in VisionUtil because that class produces it. The four
remaining private `Alliance` enums are in StarterBot-derived samples and one early-ideas file (R3 scope).
- Keep `CommonConstants.Alliance` (`CommonConstants.java:11`). Delete the other 12 `enum Alliance`
  definitions (`VisionUtil:72`, `GGRobotConstants:78`, `P3RobotConstants:98`, four Skyline autos at
  `:51`, and the rest in dead files covered by R3). P3 currently mixes two incompatible Alliance
  types, which is why its autos need ternary glue.
- Move `SharedState` to `Common` (one class). Delete the three team copies. Today Skyline's TeleOps
  import P3's copy (`Skyline_TeleopV2:39`) and Skyline's own is never imported.
- Move into `CommonConstants`: goal AprilTag coordinates (duplicated at `VisionUtil:62-66` and
  `GGRobotConstants:24-28`), `Location {CLOSE, FAR}` (duplicated GG/P3), `METERS_TO_INCHES` (defined
  in 6 classes), `MotifPattern`, and the Limelight pipeline contract `0=motif, 1=red goal, 2=blue goal`
  (`VisionUtil:68-70`, currently an undocumented private constant).
- Remove Common-to-team imports: `VisionUtil:13` and `Skyline_Robot:13` both import
  `GGRobotConstants` and never use it. Add the rule: **Common never imports a team package.**
Risk: none functional. Mechanical rename; compile catches everything.

**R5. Put hardware names and per-robot fudge factors into RobotConfig, and move each team's factory
into the team's own Constants class.** Medium effort.
*DONE 2026-09-06 in two commits (c7cb5ca Common side, then the per-chassis files), build verified,
values verified equal to the old factories by script.* RobotConfig now has `robotName`, `HardwareNames`
(nullable pinpoint/limelight, led), `Calibration` (the 1.15 / 1.1 / 27.5 / 45.33 and wheel constants),
and named setters on `PointToPointTuning`. DriveUtil2026b reads everything from config and the
Pinpoint is optional (`hasPinpoint()`); VisionUtil takes the Limelight name. Config files:
`GGBot2Config`, `P3Bot1Config`, `P3Bot3Config`, `SkylineBotConfig`, `common/test/StandardBotConfig`.
Open items for the mentor: GGRobot (Bot 1) shares GGBot2Config; P3_Robot (Bot 2) shares P3Bot3Config;
every Calibration is still at the shared defaults, so each chassis should measure its own; Skyline
still has no Constants class; no PushbotConfig was created because the pushbots stay on DriveUtil2025.
- Add to `RobotConfig`: `robotName`, drive motor names, `imuName`, `pinpointName` (nullable), `limelightName`,
  `ledName`, wheel diameter, ticks per rev, and the four tuning literals currently baked into
  `DriveUtil2026b`: `rightBackPower * 1.15` (`:494`), turn circumference `27.5` (`:657`), strafe fudge
  `1.1` (`:651`), `ENCODER_COUNTS_PER_INCH 45.33` (`:37`). Also resolve the conflict between
  `ENCODER_RESOLUTION 537` (`:38`) and `COUNTS_PER_REV 384.5` (`:47`), both used.
- Replace the 10-positional-double `PointToPointTuning` constructor (`RobotConfig:85`) with named
  setters or a builder so students cannot transpose `yawD` and `yawI`.
- Make the Pinpoint optional in DriveUtil2026b and the Limelight optional in VisionUtil (`tryGet`
  plus a null check, skip setup when absent). Skyline's autos only use encoder `drive_p3` but the
  constructor requires an `odo` device (`DriveUtil2026b:174, 219-222`). A fourth team without a
  Pinpoint or Limelight cannot use the drive or vision classes today. The demo pushbots show the
  cost of not doing this: they have neither device and carry phantom `limelight` and `odo` entries in
  their Control Hub configs purely to get past `hardwareMap.get`. The mentor is fine living with that
  on DriveUtil2025, but Common should not require it of any new robot.
- Move `createDefaultGearGirlsConfig` etc. (`RobotConfig:157-330`) out of Common into the team's
  folder, but **keep them separate from the team's Constants class**. The mentor's design intent is
  "what the robot is" (RobotConfig: hardware names, directions, offsets, calibration) versus "how it
  operates" (Constants: speeds, timings, presets, waypoints). So each team gets two files:
  `GGRobotConfig.java` next to `GGRobotConstants.java`, `P3RobotConfig.java` next to
  `P3RobotConstants.java`, and for Skyline both `SkylineRobotConfig.java` and a new
  `SkylineConstants.java`. The `RobotConfig` *class* (the data structure) stays in Common; only the
  per-robot *instances* move. Adding a team then touches zero Common files, and the two-file split
  makes the template self-explaining: fill in RobotConfig to make it drive, fill in Constants to
  make it play. This also ends the merge conflicts from all teams editing one shared file and the
  commented-out tuning history at `RobotConfig:240-267`.
- Under that split, the new fields R5 adds all belong on the RobotConfig side: device names, wheel
  and encoder constants, and the fudge factors are physical facts about the robot, not operating
  choices.
- **One RobotConfig per physical robot, named after the chassis** (`GGBot2Config`, `P3Bot3Config`),
  not after the team or season. P3 already has two factories for two chassis; make that the rule.
  The mentor keeps prior-season robots for demos, so a config follows the chassis across teams and
  years, and season rollover only rewrites Constants. Add `robotName` so telemetry shows which config
  is loaded. Location options: (1) in the team folder beside Constants, simplest for the template;
  (2) a `robots/` package at the teamcode root with one file per chassis that team robot classes
  reference. Option 2 gives retired demo robots a home and is the recommendation for this mentor.
  Either way, the legacy DriveUtil2025 robots can then get their own config, which unblocks the
  migration ruled out on 2026-09-06.
- `robotName` fixes the bug at `P3_Robot3_TeleOp:170`, which compares `ROBOT_CONFIG == RobotConfig.createP3Robot2Config()`
  by reference and therefore always reports "Other".
Risk: medium. Every hub class constructor changes. Do it in one commit per team and compile between.

**R6. Adopt one folder layout and one OpMode naming convention.** Medium effort, mostly `git mv`.
Proposed layout under `org.firstinspires.ftc.teamcode`:
```
common/            DriveUtil, VisionUtil, LedUtil, RobotConfig, CommonConstants, SharedState, base classes (R7-R10)
teams/geargirls/   Robot.java, Constants.java, subsystems/, teleop/, auto/, test/
teams/p3/          same shape
teams/skyline/     same shape
teams/_template/   the 4th-team starting point (R12)
samples/           pristine vendor code only (goBILDA, FIRST, Limelight), all @Disabled, licenses intact
```
Fixes the current inconsistency (`GGRobot` vs `GearGirlsRobot` vs `GearGirls`; `P3Robot` vs `P3`;
`Bot1` / `BOT2` / `Bot3` casing) and gives `TeleOp/Other`, `TeleOp/Concepts`, `Concepts/Tests`,
`Auto/Other` a defined meaning. Rules to write down:
- OpModes live only under a team's `teleop/`, `auto/`, or `test/`. Today `FlywheelTunerTutorial` is
  an **enabled** `@TeleOp` inside `utilities/Common` and `TurretCalibration` is one inside `utilities/P3Robot`.
- `group=` is the team name. Keep the existing practice of `@Disabled` before events so each robot
  shows one `(RUN ME)` TeleOp and a small set of autos. The layout change helps that sweep: today
  9-10 of 21 enabled OpModes are samples, tests, and a lesson file scattered across four folders, and
  all 21 appear on every team's Driver Hub.
- Move `FTC_Learning` out of the TeamCode source tree (or into its own Gradle module). Its 49 lesson
  files declare `package org.firstinspires.ftc.teamcode`, so any pasted lesson registers as an OpMode
  (`L01_HelloWorld` is enabled with no group right now).
- Add `.DS_Store` to `.gitignore`.
Risk: large diff, but pure moves. Android Studio refactor-move keeps imports correct. Do it after R1-R3
so you are not moving files you are about to delete.

### P2. Lift the six duplicated patterns into Common (mentor with students, early season)

These do not force teams to share subsystem hardware. Each takes a small interface for the one
robot-specific step and keeps the rest shared. Keep them plain Java 8: abstract classes and small
interfaces, no generics beyond what exists, no reflection.

**R7. `common/LaunchController`.** *DONE 2026-09-07 for P3_Robot3 and Skyline_Robot, build verified.*
`common/LaunchController` (state machine lifted from P3_Robot3: spin-up timeout, ready fraction or
absolute minimum, stall abort, feed timer, cooldown, shot counters, `addTelemetry`), with two tiny
interfaces each team implements as inner classes: `common/Flywheel` (setVelocity/getVelocity) and
`common/Feeder` (start/stop). Settings are a fluent `LaunchController.Settings`. P3_Robot3 keeps every
public method (launchSequence, stopLaunchSequence, isLaunchSequenceBusy, the tolerance/stall/keepSpinning
setters and getters, shot counters, areFlywheelsReady) and delegates; its settings equal its old private
constants. Skyline_Robot keeps its 4-arg launchSequence, sets feedTimeSec per call and passes the absolute
minimum velocity; cooldown 0 and stall check off to match old behavior; the 2 s spin-up timeout is NEW
for Skyline (old code waited forever). GearGirls NOT migrated: GGRobot2's LaunchState is dead (never
assigned); its TeleOp fires flippers directly and its v7 auto uses ShotSequenceControllerV2, a
different (volley) pattern. Older classes P3_Robot, P3_Robot_Bot1, GGRobot untouched (R3 scope).
Follow-ups: move the P3 launch Settings numbers into P3RobotConstants; consider having
ShotSequenceControllerV2's ready-velocity gate use LaunchController.
Original recommendation follows. Start from `P3_Robot3:194-275, 400-517` (the most robust copy).
State machine IDLE → SPIN_UP → FEEDING → COOLDOWN with spin-up timeout, tolerance %, stall abort,
shot counters. Takes a `Feeder` interface (`start()`, `stop()`, `isDone()`) so GearGirls plugs in
flippers + spinner, P3 an indexer, Skyline two feeder servos. Replaces six hand-written copies with
drifted defaults (tolerance 2.0 vs 4.0, `lastKnownGoodVelocity` 0.0 vs 1000.0, no timeout at all in
`Skyline_Robot:89-96` and GearGirls).

**R8. `common/FlywheelVelocityModel`, `common/AimLed`, `common/VisionAim`.** *DONE 2026-09-07 with
tests (20 new, 34 total), build verified, tables verified equal to the old inline values by script.*
`common/AimTarget` is the interface (VisionUtil implements it; tests use FakeAimTarget).
FlywheelVelocityModel takes the team's `double[][]` table plus an initial fallback; AimLed takes led,
target, tolerance and a Colors bag; VisionAim is static `turnPower`/`onTarget`. Tables and aim
settings moved to `GGRobotConstants.Launcher/Aim`, `P3RobotConstants.Launcher/Aim`, and a NEW
`SkylineConstants` (Skyline's first Constants class). Migrated: GGRobot2, P3_Robot3, Skyline_Robot
(public method names unchanged), and the snap-to-target in P3_Robot3_TeleOp_SingleDriver,
Skyline_TeleopV2, GearGirlsBot2_improved, P3Autonomous_QueueBot3. Behavior change on purpose:
GearGirls and Skyline initial fallback velocity is now their close-range table value (1290 / 1254)
instead of 0, matching P3's approach. Older robot classes and OpModes untouched (R3 scope).
Original recommendation follows.
- `FlywheelVelocityModel(InterpolatingLookupTable)` with `update(vision)` and last-known-good fallback.
  Replaces `updateAndGetTargetVelocity` in 5 classes. Table data moves to each team's Constants as
  a `double[][]` instead of 12 `add()` calls inside a constructor.
- `AimLed(led, vision, toleranceDeg)` replaces six identical `updateLedStatus` methods.
- `VisionAim.turnPower(vision, kp, tolDeg)` replaces the TX snap-to-target P-controller copied into
  every TeleOp and both Queue autos.

**R9. `common/AutoSelector` and `common/AutoBase`.** *DEFERRED 2026-09-07 by the mentor: last
season's autos will probably not run again, so there is no value in refactoring them. Build the
selector and the Queue-style AutoBase when the FIRST new-season auto is written, using P3's Queue
pattern and GearGirls v7's setPathWaypoints as the models, and give them tests then. The old autos
stay in place until R3.* Original recommendation follows.
- `AutoSelector` wraps the init_loop block (X=Blue, B=Red, Y=Close, A=Far, dpad cycles) plus telemetry
  and writes `SharedState.alliance` on start. Replaces 18 copies and fixes the two teams whose enabled
  autos never write SharedState (P3 Bot3 `QueueBot3`, all Skyline autos).
- `AutoBase` (iterative OpMode) generalizes the P3 Queue sequencer: `Queue<Step>` with per-step and
  global timeouts, `robot.update()` each loop, PARK on timeout. Teams write `buildScript(alliance, location)`
  and a `Waypoints.forAlliance(alliance, location)` in their Constants, like GearGirls v7's
  `setPathWaypoints`. Also replaces Skyline's near/far copy-paste pairs (Park vs Park_Far differ in
  4 of 139 lines) with one auto and a `Location` parameter.

**R10. `common/TeleOpBase` and `common/ButtonEdge`.** *CLOSED 2026-09-07, will not be done.* The three
current TeleOps differ in exactly the things a driver feels (speed scalar 0.25 / 0.80 / 1.0, Skyline
negates turn, GearGirls has a deadband, alliance override is on a different button per robot because
the bumpers already do team-specific things). A base class would take all of those as parameters to
save ~15 readable lines per TeleOp, and would put Skyline's turn sign at risk during demo season.
`ButtonEdge` is unnecessary: the SDK's `xWasPressed()` family already exists and P3/Skyline use it.
**Gamepad layouts are NOT to be standardized**: drivers have learned them and changing buttons now
means relearning. R13 docs may record each robot's current layout as-is, nothing more. TeleOps stay
plain OpModes a student can read top to bottom; the R12 template TeleOp is the "base" by copying.
Original recommendation follows for the record.
- `TeleOpBase` provides `handleDriveControls` (arcade with field-centric toggle, slow mode, R3 TX snap),
  the LB/RB alliance override that reconfigures vision, and a telemetry footer. Teams override
  `handleSubsystemControls()`. Replaces the 15-file skeleton.
- `ButtonEdge` replaces the hand-rolled `xPressed` booleans (10 of them in `LaunchFlippersTestOpMode:29-38`)
  and the `WasPressed` vs raw-boolean drift between `GearGirlsBot1` and `Bot2_improved`.
- Agree one **gamepad map across teams** and write it in a table in the docs. A mentor coaching three
  drive teams benefits more from a shared control layout than from any code change.

**R11. Fix the live defects the readers found while doing R7-R10.**
| Defect | Location |
|---|---|
| `configureVisionForTeleOp(alliance)` ignores its argument, so the TeleOp alliance switch never reaches vision | `P3_Robot3:351-356` |
| All 10 P3 autos `preselectTeleOp` a TeleOp that is `@Disabled` | `Auto/P3/**` → `"P3: Teleop (Team Version)"` |
| Enabled autos never write `SharedState.alliance` | `P3Autonomous_QueueBot3`, all `Auto/Skyline/*` |
| `arcadeDrive` called twice per loop; diverter LEFT/RIGHT inverted | `GearGirlsBot1:259-264, 338-347` |
| `driveToTagAsync` sets a drive state with no handler, so `isBusy()` stays true forever | `DriveUtil2026b:1137-1153, 744-746` |
| `getOdoPosition()` writes five telemetry lines per call; P3 autos call it up to 19 times per loop | `DriveUtil2026b:331-339` |
| Blocking helpers loop with no `opModeIsActive()` check because the OpMode param is always `null` | `DriveUtil2026b:592, 996, 1043, 1070` |
| Autos label `drive()`/`rotate()` non-blocking but they busy-wait | `Auto/Skyline/*:131-138, 181-188` |
| Two OpModes registered with the same Driver Station name `FlywheelTunerTutorial` | `Common/FlywheelTunerTutorial:13`, `Concepts/PIDMotorTunerTutorial:12` |
| Kalman correction updates the Y filter with X (dead code, but do not port it) | `DriveUtil2025:1583`, `DriveUtilDepricated:1733` |

**R12. Build `teams/_template/` for the fourth team.** Small effort once R4-R10 exist.
Use Skyline's shape (it is the cleanest) after cleanup: `Robot.java` extending a small
`common/RobotBase` (owns drive, vision, led, launch controller, alliance), `Constants.java`
(`ROBOT_CONFIG`, flywheel table, `Waypoints.forAlliance`), one TeleOp extending `TeleOpBase`,
`JustPark` and `Score3Preloads` autos extending `AutoBase`, one bench-test OpMode. Include a
`HOWTO-new-team.md`: copy the folder, rename the package, fill in Constants, name the Control Hub
devices to match `ROBOT_CONFIG`, set `group=`. Target: a new team drives with vision in one session
without touching `common/`.

### P3. Nice to have

**R13. Documentation.** A `common/README.md` covering the drive API (`arcadeDrive` argument order
changed silently between generations; P3 still passes `-driveInput`), `RobotConfig` fields, the
Limelight pipeline contract, and the alliance handoff. A repo `CONTRIBUTING.md` with the naming and
`@Disabled` rules from R3 and R6. Javadoc on the public methods of `DriveUtil2026b`.

**R14. Generic actuator wrappers.** `common/SimpleMotor(name, direction, zeroPower)`,
`common/SimpleServo(name)`, `common/TimedServoAction` (the IDLE/ACTIVE/RETRACT-with-timer shape that
`LaunchIndexer`, `LaunchFlippers`, and `Spinner_FORTEST` each reimplement). Move `Turret.java` and
`TurretCalibration` to `common/` as optional subsystems; nothing in them is P3-specific except device
names. Add `common/MotifUtil` for the DECODE-specific motif-to-shot-plan logic that exists in five copies.

**R15. Test OpModes in one place per team.** The bench-test-per-subsystem pattern is good but scattered
across `Concepts/`, `Concepts/Tests/`, and `TeleOp/GGRobot`. Put them in `teams/<team>/test/` with
`group="<Team> Test"` and `@Disabled` by default; enable one when bench testing.

## Suggested order (revised 2026-09-06 with the mentor)

Done so far in PR #1: R1 (except DriveUtil2025, which stays), R2 (Road Runner moved, kept), R4.

1. Tag the current state. **R3 is deferred**: the mentor will not delete any superseded team code
   until the reorganization (R6) and the config/constants work (R5) are finished. Superseded files
   simply move along with everything else in R6 and get removed at the end, when the mentor can see
   the whole tree in its new shape. When R3 does happen: delete only files that a newer file for the
   same robot has replaced, confirmed by diff; nothing gets deleted for being old or `@Disabled`.
2. **R6 first, with a `demobots/` folder.** Layout: `common/`, `teams/<team>/`, `demobots/`, `samples/`.
   The six demo robots are two code sets, not six: four identical pushbots and two identical
   goBILDA StarterBots for this season. So `demobots/` has two subfolders:
   - `demobots/pushbot/`: the DriveUtil2025-based drive-only TeleOps (BasicDriveTeleop,
     BasicOpMode_Linear1, and Coachbot if it is still used). One folder, one TeleOp.
   - `demobots/starterbot/`: StarterBotTeleop, StarterBotTeleop_DEBUG, StarterBotAuto. This is
     goBILDA's standalone code with its own hardware names (`left_drive`, `right_drive`, `launcher`,
     `left_feeder`, `right_feeder`); it does not use DriveUtil or RobotConfig and should stay that
     way so goBILDA updates can be dropped in.
   Use `group="Demo: Pushbot"` and `group="Demo: StarterBot"`; keep them `@Disabled` except around a
   demo. Pure moves, compile, one PR with no logic changes.
   R5 consequence: identical chassis share one RobotConfig. Pushbots get one `PushbotConfig` (a
   second only if one is wired differently). StarterBots need no RobotConfig at all. Once
   `PushbotConfig` exists the pushbots can move to DriveUtil2026b and DriveUtil2025 can finally go.
3. **R5 after R6**, so the one-config-per-chassis files are created in their final home. Demo bots get
   their own RobotConfig here, which is what makes it safe for them to share DriveUtil2026b later.
4. **R3 now**, with the tree in its final shape: delete the superseded copies, mentor-reviewed file
   by file. (Still on hold as of 2026-09-07.)
5. R7 and R8 done in the off-season. R9 deferred until the first new-season auto exists. R10 closed.
   Remaining before or early in the season: the TeleOp items of R11, R12 template (a plain TeleOp
   carrying the demo-safe default), R13 docs (record layouts as-is).
6. R12 template as soon as R7-R10 are stable, before the fourth team's first meeting.
7. R13-R15 as time allows.

## R6 target tree (DONE 2026-09-06: commits b72f53d move, 1d9f7a7 groups; pushed to PR #1)

Mentor decisions: Q1 Score2SpecimenPinPointAuto stays with GearGirls (auto/earlyideas). Q2 Coachbot is
a pushbot TeleOp (demobots/pushbot). Q3 no Into The Deep demo bots remain; utilities/PriorSeason moved
to `legacy/intothedeep/` as a holding place for R3. Q4 L01_HelloWorld left untracked in TeleOp/Other.
Q5 lowercase packages. Tag `pre-r6-reorg` marks the state before the move. 147 files moved by
scratchpad script r6_move.py; 34 imports added where the old team packages were split into hub +
subsystems/. Build verified. Group commit: 88 OpModes now use one of nine groups (GearGirls, GearGirls
Test, P3, P3 Test, Skyline, Demo, Common Test, Sample, Pedro); the mentor chose a single Demo group.
Lesson learned: OpMode names contain parentheses, so any annotation rewrite must be quote-aware.

Packages become lowercase, per Java convention (`utilities.Common` → `common`, `TeleOp.GGRobot` →
`teams.geargirls.teleop`). Every file below is a move only; no logic changes in the R6 commit.
Superseded files move along with their team and wait for R3.

```
org.firstinspires.ftc.teamcode/
├── common/                         shared library code, NO team names inside
│   ├── CommonConstants  SharedState  RobotConfig  RobotConfigorig*  DriveUtil2026b  DriveUtil2025
│   ├── ProportionalControl  EncoderOdometry*  InterpolatingLookupTable  LedUtil  VisionUtil
│   ├── prismled/                   (as is)
│   └── test/                       OpModes that exercise Common code only
│       MechanumWheelTestDriveUtil  BasicAuto_Iterative  GBPinpointDriveToPoint
│       SampleAuto_usingSimplifiedOdometry  PrismArtboardTest  PrismLedTestOpMode
├── teams/
│   ├── geargirls/
│   │   ├── GGRobot  GGRobot2  GGRobotConstants          (R5 adds GGBot2Config here)
│   │   ├── subsystems/   IntakeUtil  IntakeUtilV2  IntakeSensorFusion  ...001  ...002
│   │   │                 LaunchFlippers  LaunchIndexer  LauncherMotors  Spinner  Spinner_FORTEST
│   │   │                 ShotSequenceController  ShotSequenceControllerV2  ShotVolleyController  AutoAction
│   │   ├── teleop/       GearGirlsBot1  GearGirlsBot2_improved  GearGirlsBot2_test
│   │   ├── auto/         bot2/GGAutonomous_Score9_v7   bot2/old/*   bot1/*   earlyideas/*
│   │   └── test/         GGCombinedSystemTest  GGFlywheelTestOpMode  IntakeUtilTestOpMode
│   │                     LaunchFlippersTestOpMode  FlywheelTunerTutorial (from Common)
│   │                     ColorSensorDiagnostic  Test_IntakeSensorFusion  IntakeSensorTest
│   │                     ColorAveragingSensor  ConceptVisionColorLocator_Circle
│   │                     PedroPathTeleopCoachTest  PedroPathTeleopCoachTestDriveOnly
│   ├── p3/
│   │   ├── P3_Robot  P3_Robot3  P3_Robot_Bot1  P3RobotConstants   (R5 adds P3Bot3Config here)
│   │   ├── subsystems/   P3_IntakeUtil  P3_LauncherUtil  P3_IndexerUtil  P3_RubberBandIndexerUtil
│   │   │                 P3_HoodServoUtil  Turret  P3_TurretUtil  P3_TurretUtil_Velocity
│   │   ├── teleop/       P3_Robot3_TeleOp  P3_Robot3_TeleOp_SingleDriver  P3_Teleop   old/*
│   │   ├── auto/         bot1/*  bot2/*  bot3/P3Autonomous_QueueBot3  P3PedroPathAuto
│   │   └── test/         TurretCalibration (from utilities)  TurretTeleOpExample  MinimalServoTest
│   │                     PIDMotorTunerTutorial (targets P3 turret_motor)
│   └── skyline/
│       ├── Skyline_Robot                                   (R5 adds SkylineConstants + config)
│       ├── subsystems/   Skyline_FeederUtil  Skyline_LauncherUtil
│       ├── teleop/       Skyline_Teleop  Skyline_TeleopV2
│       └── auto/         Skyline_Autonomous_Park  ..._Park_Far  ..._Score3Preloads  ..._FAR
│                         SkylineAuto  Delete (it is named "SKYLINE AUTO")
├── demobots/
│   ├── pushbot/          BasicDriveTeleop  BasicOpMode_Linear1   [Coachbot?  see Q2]
│   ├── starterbot/       StarterBotTeleop  StarterBotTeleop_DEBUG  StarterBotAuto
│   └── intothedeep/      utilities/PriorSeason/* (Robot + six helper classes)   [see Q3]
├── samples/              pristine vendor code, all @Disabled, licenses intact
│   DecodeRi3D  RobotTeleopMecanumFieldRelativeDrive  SensorColor  SensorGoBildaPinpointExample
│   SensorGoBildaPinpoint  SensorLimelight3A
├── pedropathing/         Constants  Drivetrain  Tuning  ExampleTeleOp  ExampleTeleOp_george
│                         AutoPedroPathExample  PedroPatchDrivetrainTest
└── roadrunner/           (as is, done)
```
`*` = R3 candidate that moves for now.

Driver Station `group=` convention (second, text-only commit inside R6 so the move commit stays pure):
`GearGirls` / `P3` / `Skyline` for competition OpModes, `<Team> Test` for test/, `Demo: Pushbot` /
`Demo: StarterBot`, `Sample`, `Pedro`. Sorts each robot's OpModes together on the Driver Hub.

Open questions for the mentor before moving:
- **Q1** Score2SpecimenPinPointAuto (Into The Deep auto on Road Runner, in GearGirls earlyIdeas): stay
  with GearGirls history, or go to `demobots/intothedeep/` with the PriorSeason robot class?
- **Q2** Coachbot builds GearGirls launcher/intake classes on DriveUtil2025. Is it a pushbot TeleOp, a
  GearGirls test, or unused?
- **Q3** Is there still an Into The Deep robot among the demo bots? If not, `demobots/intothedeep/`
  is just a holding place until R3.
- **Q4** L01_HelloWorld is untracked and sits in TeleOp/Other. It is a lesson file; suggest it stays
  out of the tree (FTC_Learning is already outside src).
- **Q5** Lowercase package names mean every import in the repo changes. Fine for a compile-verified
  move, but it will make PR #2 large. Alternative is keeping current casing and only moving folders.

Mechanics: `git mv` per folder, then rewrite `package` and `import` lines with perl (BSD sed lacks
`\s`/`\b`), compile, `git status` must show only R (rename) plus package/import diffs.

## Unit tests (added 2026-09-07)

TeamCode has a local JUnit 4 test source set at `TeamCode/src/test/java`, run on the laptop JVM with
no robot attached:

```bash
cd /Users/georgemitchom/StudioProjects/FTC17651/FtcRobotController_Decode && JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:testDebugUnitTest
```

or right-click a test class in Android Studio and choose Run. Results land in
`TeamCode/build/test-results/testDebugUnitTest/*.xml`. `testOptions.unitTests.returnDefaultValues`
is on so Android framework calls return defaults instead of throwing.

First test: `common/LaunchControllerTest` (14 tests) drives `LaunchController` with a fake Flywheel,
fake Feeder, and a hand-advanced fake `LaunchController.Clock`, covering ready threshold, absolute
minimum velocity, spin-up timeout (and disabling it), feed window, cooldown, keepSpinning, stall
abort (and disabling it), repeated fire, stop(), and counter reset. Note the controller starts the
feeder on the loop AFTER it sees the flywheel ready, and applies keepSpinning on the first COOLDOWN
loop, exactly as the original P3 code did; the tests document that.

**The testing pattern for Common:** put an interface in front of hardware (Flywheel, Feeder) and
of time (Clock), hand the class fakes in the test, assert what it commanded. Anything that moves
into Common in R8-R10 should arrive with a test written this way. OpModes and the per-team
adapters stay hardware-bound and are covered by the on-robot checks instead.

Simulators looked at 2026-09-07 (none adopted): Beta8397 virtual_robot (desktop JavaFX, paste
OpModes into its project), Pedro Pathing Visualizer and MeepMeep (path drawing, not physics),
Webots via team 6448's FTC bridge (full physics, heavy setup).

## Verification

After each step, run the command-line compile check confirmed working earlier in this session:

```bash
cd /Users/georgemitchom/StudioProjects/FTC17651/FtcRobotController_Decode && JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:assembleDebug
```

Then, on each robot: deploy from Android Studio, confirm the Driver Station shows only that team's
OpModes, run the `(RUN ME)` TeleOp and one auto. The alliance handoff (auto sets red or blue, TeleOp
starts with the same alliance and LED color) is the fastest end-to-end check that R4 and R9 landed.

## If you want the deeper verification pass

The workflow that produced this stopped at the comparator stage when the session limit hit. After
the limit resets (7:30pm Central), it can resume with the six reader results cached. The script needs
one fix first: the line that counts findings must skip comparators that returned null
(`compares.filter(c => c && c.result)`). Resuming would add the four cross-team comparisons and
three adversarial verifiers per recommendation, roughly 50 more agents.
