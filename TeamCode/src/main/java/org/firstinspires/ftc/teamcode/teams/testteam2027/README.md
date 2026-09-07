# TestTeam2027 / test2027bot: how a new team gets up and running

This folder is a complete, minimal team: one robot, one TeleOp, two autos, one bench-test OpMode.
It exists to test the shared `common/` code on a spare chassis, and to be copied by the next real
team. Nothing in `common/` was changed to make it work, which is the point of the structure.

Time for a new team, once the robot is wired: about one meeting to drive, a second to measure and
tune. The steps below are in the order to do them.

## What is in the folder

```
teams/testteam2027/
  Test2027BotConfig.java      WHAT the robot is: device names, motor directions, IMU mounting,
                              Pinpoint pods, calibration, tuning. Edit when the robot is rewired.
  Test2027Constants.java      HOW it operates: speeds, waypoints, which tag to drive to. Edit when
                              the game changes or the drivers change their minds.
  Test2027Robot.java          The one object every OpMode creates. Owns drive and vision, updates
                              them each loop. Add game subsystems here as public fields.
  teleop/Test2027Teleop.java  Drive with the sticks; hold RB to drive to the test tag.
  auto/Test2027DriveSquareAuto.java   Pinpoint waypoints: drives a 24 in square. The auto template.
  auto/Test2027TagApproachAuto.java   Non-blocking AprilTag approach test.
  test/Test2027EncoderMoveCheck.java  Encoder moves on buttons, for calibration measurements.
  README.md                   This file.
```

Two rules from the cleanup plan that this layout follows: the config file says what the robot IS,
the constants file says how it OPERATES, and they are never merged. And `common/` never names a
specific robot; each team's three-line `EncoderMoveCheck` subclass is what that looks like.

## Step 1. Copy and rename (10 minutes)

1. Copy this folder to `teams/<yourteam>/`.
2. Replace the package line in every file: `teams.testteam2027` becomes `teams.<yourteam>`.
   Android Studio: right-click the folder, Refactor, Rename, and it rewrites the imports.
3. Rename the classes from `Test2027...` to your team's prefix. Android Studio Refactor, Rename
   on each class name updates every use.
4. Change the `group = "TestTeam2027"` in the four `@TeleOp` / `@Autonomous` annotations to your
   team name. That group is what the Driver Station shows.
5. Change `preselectTeleOp` in the two autos to your TeleOp's name.

Build from the terminal to catch a missed rename before it reaches a robot:

```bash
cd /Users/georgemitchom/StudioProjects/FTC17651/FtcRobotController_Decode && JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:assembleDebug
```

## Step 2. Fill in the config (20 minutes, at the robot)

Open your `...BotConfig.java`. Each numbered comment in it matches these:

1. **Device names.** On the Driver Station, open Configure Robot and the active configuration.
   Copy the four drive motor names, the IMU name, and the Pinpoint and Limelight names exactly.
   No Pinpoint? `.pinpoint(null)`. No Limelight? `.limelight(null)`. The shared code checks.
2. **Motor directions.** Leave the defaults, deploy, run the TeleOp, push the left stick forward.
   Any wheel that turns backward gets `REVERSE` in `DrivetrainConfig`. Then try strafe and turn.
3. **IMU mounting.** Which way the REV logo faces (UP, DOWN, FORWARD, ...) and which way the USB
   ports face. Wrong values make field-centric driving and the heading telemetry drift.
4. **Pinpoint pods.** Offsets in mm from the robot's center of rotation to each pod, and the
   count direction. Check by pushing the robot forward by hand with the TeleOp in init: the
   telemetry X should rise. Push it left: Y should rise. Turn it counter-clockwise: heading rises.
   Flip the pod direction that goes the wrong way.
5. **Calibration.** Leave the defaults for now; Step 4 measures them.
6. **Tuning.** Leave the defaults for now; Step 5 tunes them.

## Step 3. Drive it (first meeting)

Deploy. On the Driver Station the team's group shows four OpModes. Run `Teleop (RUN ME)`:

- Left stick drives and strafes, right stick turns, left bumper is slow mode.
- The telemetry footer shows the robot config name, the Pinpoint position, and whether a tag is
  visible. If the config name is wrong you are running another team's robot class.
- Back resets the Pinpoint to zero.

If the robot drives, Step 2 is done. Commit.

## Step 4. Measure the calibration (second meeting, tape measure)

Run `Encoder Move Check` and follow `doc/ROBOT_TEST_PLAN.md` section F. Four numbers, four
moves, two minutes each: ticks per inch, strafe slip, turning circle, and whether the right-rear
wheel needs a power correction. Write them into `Calibration` in your config. Now `drive_p3`
and the `driveRobotDistance*` commands move the distance they are told.

## Step 5. Prove the Pinpoint and tune driveTo (second meeting, floor space)

Run `Drive Square (Pinpoint)` from a tape mark. The robot drives a 24 in square and turns a
quarter turn; it should end within an inch of the mark. If it drifts, the pod offsets or
directions in Step 2 are off. If it oscillates or crawls into each corner, the point-to-point
gains in `PointToPointTuning` need work: lower `xyGains` P if it oscillates, raise it if it
crawls, and only then touch D. The telemetry line "steps that timed out" should read 0.

## Step 6. Prove the AprilTag approach (a tag taped to a wall)

Set `TagTest.TAG_ID` in your constants to the tag on the wall. Follow `doc/ROBOT_TEST_PLAN.md`
section H, in order: the sign check on a stand (H1), the wheel directions (H2), then
`Tag Approach Test` from 3 ft away (H3). The TeleOp's right bumper does the same approach
interactively, which is the quickest way to repeat H1 while someone adjusts the four sign
constants at the top of the TagSighting section of `common/VisionUtil.java`. Those constants
are shared by every robot, so once one robot has them right, every robot does.

## Step 7. Add the game

- A subsystem: a class in `teams/<yourteam>/subsystems/`, built in the robot class from device
  names you add to `HardwareNames` in your config. Look at Skyline's launcher and feeder.
- A launcher: plug into `common/LaunchController` with two tiny interfaces. Skyline_Robot shows
  the whole thing in 20 lines.
- Waypoints: add them to your constants, one `Pose2D` each, and sequence them the way
  `Drive Square` does. That is exactly how the GearGirls and P3 autos work.
- A tag to drive to: `robot.drive.driveToTagAsync(robot.vision, id, standoffInches, holdSec)`,
  then wait on `robot.drive.isBusy()` while the launcher spins up in the same loop.
- A first auto with no Pinpoint at all: the beginner commands, each one blocking until done and
  with the direction in its name: `driveRobotDistanceForwardInches(24, 0.4)`,
  `driveRobotDistanceStrafeLeftInches(12, 0.4)`, `turnLeft(90, 0.3)`, `turnRight(45, 0.3)`.
  They all read the calibration you measured in Step 4.

## Things that will bite

- `arcadeDrive`'s arguments are `(strafe, drive, turn, unused, speed)`. The stick's forward is
  negative, so drive is `-left_stick_y`. The TeleOp already does this; keep it.
- `@Disabled` hides an OpMode from the Driver Station; it does not delete it.
- Every blocking encoder move now has a time limit. If a wheel is held, the move ends on its own
  after a few seconds and returns false. The Pinpoint `driveTo` never blocks.
- The Driver Station only shows the groups of the OpModes in the APK. Four teams' OpModes are
  always all present; the group name is what keeps them apart.
