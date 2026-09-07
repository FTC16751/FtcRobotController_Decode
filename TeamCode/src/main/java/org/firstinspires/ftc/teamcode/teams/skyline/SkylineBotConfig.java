package org.firstinspires.ftc.teamcode.teams.skyline;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

/**
 * WHAT THE SKYLINE CHASSIS IS.
 *
 * Edit this file when the robot is rewired, a device is renamed in the Control Hub config, the
 * Pinpoint is moved, or the hub is remounted. Speeds, feed times and launcher presets belong in
 * a SkylineConstants class (Skyline does not have one yet; today they are inline in Skyline_Robot).
 *
 * Motor directions and calibration were measured on the robot 2026-09-07 (see the comments at
 * each). Skyline does not use Pedro Pathing, so that section is null.
 */
public final class SkylineBotConfig {

    private SkylineBotConfig() {}

    public static RobotConfig create() {
        return new RobotConfig(
                // 2026-09-07: the Control Hub configuration was corrected so each motor's name is on
                // its physical corner (they had been on the diagonal-opposite ports, which is why the
                // TeleOps used to negate the turn input). With the names right, the LEFT side reverses.
                new RobotConfig.DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE,   // left front
                        DcMotorEx.Direction.FORWARD,   // right front
                        DcMotorEx.Direction.REVERSE,   // left rear
                        DcMotorEx.Direction.FORWARD    // right rear
                ),
                new RobotConfig.OdometryConfig(
                        -0.0, -150.0,                  // Pinpoint pod offsets, mm (X pod, Y pod)
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                new RobotConfig.ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ),
                new RobotConfig.PointToPointTuning()
                        .xyToleranceMm(15.5).yawToleranceRad(0.0349066)
                        .xyGains(0.01905, 0.000002, 0.00111).xyAccel(8.0)
                        .yawGains(5.0, 0.0, 0.0).yawAccel(20.0),
                null // no Pedro Pathing on this robot
        )
        .named("Skyline")
        .withHardware(new RobotConfig.HardwareNames()
                // drive motors, imu, pinpoint and limelight use the default names
                .led("led_servo"))
        // Measured on this chassis 2026-09-07 with the Encoder Move Check (see
        // doc/ROBOT_TEST_PLAN.md section F). 140 mm goBILDA mecanum wheels on 312 rpm motors:
        // 48 in commanded measured 48. Strafe 24 went 28 at x1.1. A 360 turned about 135 at 27.5.
        // The old shared defaults (96 mm wheels, 27.5 in turn) made every auto drive 1.46x and turn
        // 2.7x short; last season's autos were tuned around that and will not run again.
        .withCalibration(new RobotConfig.Calibration()
                .rightRearPowerScale(1.0)   // not measured yet; start with no correction
                .strafeScale(0.94)
                .turnCircumferenceIn(73.0)
                .encoderCountsPerInch(RobotConfig.Calibration.countsPerInch(537.7, 1.0, 140)));
    }
}
