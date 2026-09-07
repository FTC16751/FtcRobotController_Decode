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
 * Values here are exactly what RobotConfig.createDefaultSkyLineConfig() held before 2026-09-06.
 * Skyline does not use Pedro Pathing, so that section is null.
 */
public final class SkylineBotConfig {

    private SkylineBotConfig() {}

    public static RobotConfig create() {
        return new RobotConfig(
                new RobotConfig.DrivetrainConfig(
                        DcMotorEx.Direction.FORWARD,   // left front
                        DcMotorEx.Direction.REVERSE,   // right front
                        DcMotorEx.Direction.FORWARD,   // left rear
                        DcMotorEx.Direction.REVERSE    // right rear
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
        // Calibration defaults are the values that were shared by every robot before 2026-09-06.
        // Measure this chassis and set its own: rightRearPowerScale, strafeScale, turnCircumferenceIn.
        .withCalibration(new RobotConfig.Calibration());
    }
}
