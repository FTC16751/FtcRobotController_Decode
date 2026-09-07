package org.firstinspires.ftc.teamcode.teams.testteam2027;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.TagApproach;

/**
 * WHAT THE test2027bot CHASSIS IS.
 *
 * This is the first file a new team fills in. Every value here is a physical fact about one
 * chassis: what its devices are called in the Control Hub configuration, which way the motors
 * spin, where the odometry pods sit, how the hub is mounted, and the numbers that make it drive
 * straight. Nothing in here changes when the game changes; that belongs in Test2027Constants.
 *
 * How to fill it in, in order (see README.md in this folder for the long version):
 *   1. Device names: open the Control Hub configuration on the Driver Station and copy the names
 *      exactly. Set pinpoint or limelight to null if the robot does not have one.
 *   2. Motor directions: with everything FORWARD, push the left stick forward in the TeleOp and
 *      REVERSE any wheel that spins backward.
 *   3. IMU mounting: which way the REV logo faces and which way the USB ports face.
 *   4. Pinpoint pod offsets and directions (skip if no Pinpoint).
 *   5. Calibration: start with the defaults, then measure with the Encoder Move Check OpMode.
 *   6. Tuning (point-to-point and tag approach): start with the defaults, then tune on the floor.
 */
public final class Test2027BotConfig {

    private Test2027BotConfig() {}

    public static RobotConfig create() {
        return new RobotConfig(
                // 2. Motor directions. Verified on the Skyline chassis 2026-09-07: forward, both
                //    strafes, and both turns all go the right way with the left side reversed. Which
                //    side needs reversing depends on how the motors face; test, do not assume.
                new RobotConfig.DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE,   // left front
                        DcMotorEx.Direction.FORWARD,   // right front
                        DcMotorEx.Direction.REVERSE,   // left rear
                        DcMotorEx.Direction.FORWARD    // right rear
                ),
                // 4. Pinpoint: pod offsets from the robot center in mm (X pod is the forward pod,
                //    Y pod the sideways pod) and the direction each pod counts positive.
                new RobotConfig.OdometryConfig(
                        0.0, -150.0,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                // 3. Control Hub mounting.
                new RobotConfig.ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ),
                // 6a. Pinpoint point-to-point PID (driveTo). These are the StandardBot defaults.
                new RobotConfig.PointToPointTuning()
                        .xyToleranceMm(15.5).yawToleranceRad(0.0349066)
                        .xyGains(0.01905, 0.000002, 0.00111).xyAccel(8.0)
                        .yawGains(5.0, 0.0, 0.0).yawAccel(20.0),
                null   // no Pedro Pathing on this robot
        )
        .named("test2027bot")
        // 1. Device names, exactly as in the Control Hub configuration.
        .withHardware(new RobotConfig.HardwareNames()
                .driveMotors("Front_Left", "Front_Right", "Rear_Left", "Rear_Right")
                .imu("imu")
                .pinpoint("odo")            // null if this robot has no Pinpoint
                .limelight("limelight")     // null if this robot has no Limelight
                .led(null))                 // no status LED on the test robot
        // 5. Calibration. Defaults were tuned on one robot years ago; measure this one.
        .withCalibration(new RobotConfig.Calibration()
                .rightRearPowerScale(1.0)   // start with no correction; measure drift first
                // Measured on the Skyline chassis 2026-09-07 with the Encoder Move Check:
                // strafe: commanded 24 in went 28 in with x1.1, so 1.1 x 24/28.
                // turn: commanded 360 turned about 135 with 27.5, so 27.5 x 360/135. Refine with a
                // commanded 90 (D-pad right): new = 73 x 90 / degrees actually turned.
                .strafeScale(0.94)
                .turnCircumferenceIn(73.0)
                // 312 rpm goBILDA motor (537.7 ticks/rev), direct drive, 140 mm goBILDA mecanum
                // wheel. Different motor or wheel? Change these three numbers; the Encoder Move
                // Check then measures the real value. (Found 2026-09-07: with 96 here, a commanded
                // 24 in drove 35 in, exactly the 140/96 wheel ratio.)
                .encoderCountsPerInch(RobotConfig.Calibration.countsPerInch(537.7, 1.0, 140)))
        // 6b. Tag approach (driveToTagAsync). Gentle on purpose: the robot is about to touch something.
        .withTagApproach(new TagApproach.Settings()
                .kpDrive(0.04).kpStrafe(0.04).kpYaw(0.015)
                .maxPower(0.3).minPower(0.08)
                .toleranceInches(1.0).toleranceDegrees(2.0)
                .lostTimeoutSec(1.5).maxTimeSec(6.0));
    }
}
