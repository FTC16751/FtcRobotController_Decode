# Robot Test Plan for the off-season changes

Everything on branch `offseason/common-cleanup-2026` was written and compiled without a robot.
This is the list of what to check the next time a robot is available, in the order that gets the
most coverage per minute. Tick the box, write the date and robot next to it, and note anything
that surprised you. Where a test needs a number, the expected value is given.

Last updated 2026-09-07. Each entry names the commit that made the change.

## Before you start

- [ ] Deploy the branch from Android Studio to each Control Hub you test.
- [ ] Have a tape measure, a masking-tape start line, and something to block a wheel (a hand on the
      tire is fine with the robot on a stand).
- [ ] Battery above 12.5 V for any distance measurement; low battery changes RUN_TO_POSITION timing.

## A. Every robot: it still boots and drives (R4, R5, R6)

Run on: every competition robot (GearGirls Bot 1 and Bot 2, P3 Bot 3, Skyline), plus one pushbot
and one StarterBot.

- [ ] **A1. Driver Station groups.** The OpMode list shows only that team's group plus `Demo`.
      Pushbot and StarterBot OpModes are under `Demo`. No OpMode named `L01_HelloWorld` or any
      other lesson file is visible. (b72f53d, 1d9f7a7)
- [ ] **A2. Config name in telemetry.** Run the `(RUN ME)` TeleOp. The telemetry line
      `robot config` shows the right name for the chassis. (c7cb5ca, 3cc8ac1)
- [ ] **A3. It drives.** Forward, strafe, turn all go the direction the driver expects. The gamepad
      layout has not changed, so a driver from last season should notice nothing. (d4b2708 and later)
- [ ] **A4. Alliance handoff.** Run any auto, select BLUE in init, start it, stop it. Start the
      TeleOp. It reports BLUE, the LED shows the blue alliance color, and the Limelight is targeting
      the blue goal. Repeat with RED. (d4b2708)

## B. Encoder moves: distance fix and stoppable loop (a107f3f)

These are the two DriveUtil2026b changes from 2026-09-07. Any mecanum robot on this branch works;
Skyline is the best choice because its live autos use these moves.

- [ ] **B1. Nothing got slower.** Robot on the floor at a tape line. Run `SKYLINE: PARK FAR` (it
      drives `drive_p3(24, 0, 0, 0.5)`). Time it by eye against last season. Expected: the same
      move, no pause at the end. The move now polls every 10 ms instead of spinning; if it looks
      hesitant at the end of the move, say so.
- [ ] **B2. Distance is right.** Same run. Measure the travel. Expected about 24 in, within the
      slip you saw last season. Record the number, it feeds section F.
- [ ] **B3. Stalled wheel ends the step.** Robot on a stand. Start `SKYLINE: PARK FAR`. During the
      24 in move, hold one wheel. Expected: the step ends on its own after about 4 to 5 s
      (the limit is three times the ideal travel time at 0.5 power, plus 2 s), the motors stop,
      and the auto continues to its next state. Before this change it would spin forever.
- [ ] **B4. Stop during a move does not restart the app.** Robot on a stand. Start the same auto,
      press Stop on the Driver Station while the wheels are moving. Expected: the OpMode stops
      within a second and the Robot Controller app does NOT restart. Note: Skyline's autos are
      iterative OpModes, so the time limit is what ends the move there, not the interrupt check.
      Both paths are covered by this test plus B3.
- [ ] **B5. Forward by inches drives the right distance.** No live OpMode calls
      `driveRobotDistanceForwardInches`, so use `common/test/BasicAuto_Iterative`: remove its
      `@Disabled`, uncomment the line `drive.driveRobotDistanceForwardInches(12, .5)`, deploy, run
      on the floor from a tape line. Expected: 12 in. Before the fix this drove about 21 in.
      Put `@Disabled` back afterward. (A dedicated one-button "encoder move check" OpMode would
      make this repeatable; ask for it if you want it.)

## C. LaunchController in Common (f054394, ba10258)

Run on: P3 Bot 3 (`P3: Robot 3 TeleOp (RUN ME)`) and Skyline (`SKYLINE: Teleop (V2 RUN ME)`).
The unit tests cover the state machine; this checks the wiring to real motors and servos.

- [ ] **C1. Normal shot.** Spin up, fire. The flywheel reaches speed, the feeder runs for the
      configured time, the shot counter goes up by one, the flywheel behaves as it did before
      (keep spinning or stop, per the robot's old setting).
- [ ] **C2. Repeat fire.** Fire three times in a row. Cooldown between shots feels the same as
      last season on P3. Skyline has cooldown 0, so back-to-back is immediate.
- [ ] **C3. Skyline spin-up timeout is NEW.** Skyline used to wait forever for the flywheel.
      It now feeds after 2 s even if the flywheel has not reached the target. With a fresh battery
      this should never trigger. If a shot feeds early on a weak battery, that is this timeout
      working; decide whether 2 s is the right number for Skyline.
- [ ] **C4. Stall abort (P3 only).** Fire, then block the flywheel briefly during spin-up.
      Expected: the sequence aborts instead of hanging, telemetry says why.

## D. Aiming helpers in Common (6c5e053)

Run on: GearGirls Bot 2, P3 Bot 3, Skyline, each with its `(RUN ME)` TeleOp.

- [ ] **D1. Flywheel fallback before the goal is seen.** Cover the Limelight, start the TeleOp,
      read the target-velocity telemetry. Expected: GearGirls 1290, Skyline 1254, P3 its own close
      value. Last season GearGirls and Skyline showed 0 here. This was a deliberate change.
- [ ] **D2. Table lookup.** Uncover the Limelight, stand the robot at a known distance from the
      goal (say 60 in). The target velocity matches the row in the team's Constants table for that
      distance, interpolated if between rows.
- [ ] **D3. Lose the goal.** Cover the Limelight again. The target velocity holds the last good
      value, it does not drop to the fallback.
- [ ] **D4. Aim LED.** Point the robot left of the goal, right of it, and at it. The LED shows the
      team's three colors in the right places, with the same tolerance as last season.
- [ ] **D5. Snap-to-target turns the right way.** Press the aim button with the goal off to the
      right. The robot turns right, and stops turning when centered. Repeat from the left. A sign
      error here shows up immediately as turning away from the goal.

## E. Demo-safe TeleOp defaults (d524e2d, 839f3f2)

Run on: GearGirls Bot 2 first (its default changed), then P3 Bot 3 and Skyline V2 to confirm they
already behaved this way.

- [ ] **E1. Starts in PRESET at CLOSE.** Start the TeleOp with the Limelight covered. Targeting
      mode telemetry says PRESET (or MANUAL), the launcher setpoint is the CLOSE value, and the
      flywheel is idle until the driver spins it up.
- [ ] **E2. Vision is opt-in.** On GearGirls Bot 2, D-pad left cycles the mode to AUTO. Then the
      launcher follows the Limelight distance. Cycling back returns to PRESET.
- [ ] **E3. A full demo shot with the goal not visible** works from PRESET at CLOSE range.

## F. Measurements to take while you have the robot (feeds the next DriveUtil step)

Every chassis still carries the shared default calibration. Take these numbers once per chassis
and write them in that robot's config file (`GGBot2Config`, `P3Bot3Config`, Skyline's config).
Each takes about two minutes with `drive_p3` from a tape line.

| Measurement | How | Feeds |
|---|---|---|
| Ticks per inch | `drive_p3(48, 0, 0, 0.4)`, measure actual travel. New value = 45.33 x 48 / measured | `encoderCountsPerInch` |
| Strafe slip | `drive_p3(0, 24, 0, 0.4)`, measure actual sideways travel. New scale = 1.1 x 24 / measured | `strafeScale` |
| Turning circle | `drive_p3(0, 0, 360, 0.4)`, note the actual rotation. New value = 27.5 x 360 / actual degrees | `turnCircumferenceIn` (then retire `robotDiameterCm`) |
| Right-rear correction | Drive straight at 0.5 for 8 ft with `rightRearPowerScale` set to 1.0 in the config; note the drift | Whether 1.15 belongs on this chassis at all |

Use `common/test/BasicAuto_Iterative` or `MechanumWheelTestDriveUtil` (remove `@Disabled` for the
day) as the vehicle for these calls; both already construct a DriveUtil2026b with the robot's config.

## G. Known defects that are NOT fixed yet (expect these to still misbehave)

Listed so a tester does not report them as regressions. They are R11 in the cleanup plan.

- P3 Bot 3: the TeleOp alliance switch never reaches the Limelight (`configureVisionForTeleOp`
  ignores its argument).
- All P3 autos preselect a TeleOp that is `@Disabled`, so the Driver Station will not auto-load
  the TeleOp after auto.
- `driveToTagAsync` (only reachable from a disabled GearGirls auto) marks the drive busy forever.

## Results

| Test | Date | Robot | Pass? | Notes |
|---|---|---|---|---|
| | | | | |
