package org.firstinspires.ftc.teamcode.teams.geargirls;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

/**
 * WHAT THE GEARGIRLS BOT 2 CHASSIS IS.
 *
 * Edit this file when the robot is rewired, a device is renamed in the Control Hub config, the
 * Pinpoint is moved, or the hub is remounted. For speeds, feed times, launcher presets and
 * waypoints, edit GGRobotConstants instead.
 *
 * Values here are exactly what RobotConfig.createDefaultGearGirlsConfig() held before 2026-09-06.
 * GGRobot (Bot 1) also uses this config because the two chassis were never given separate
 * values; give Bot 1 its own file if it differs.
 */
public final class GGBot2Config {

    private GGBot2Config() {}

    public static RobotConfig create() {
        return new RobotConfig(
                new RobotConfig.DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE,   // left front
                        DcMotorEx.Direction.FORWARD,   // right front
                        DcMotorEx.Direction.REVERSE,   // left rear
                        DcMotorEx.Direction.FORWARD    // right rear
                ),
                new RobotConfig.OdometryConfig(
                        -0.0, -203.0,                  // Pinpoint pod offsets, mm (X pod, Y pod)
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                new RobotConfig.ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                ),
                new RobotConfig.PointToPointTuning()
                        .xyToleranceMm(32).yawToleranceRad(0.0349)
                        .xyGains(0.0035, 0.000003, 0.00125).xyAccel(10.0)
                        .yawGains(2.0, 0.0, 0.21).yawAccel(10.0),
                new RobotConfig.PedroPathingConfig(
                        5.0, -34.46, -64.23,
                        new PIDFCoefficients(0.01905, 0, 0.0035, 0.02), // translational
                        new PIDFCoefficients(0.5, 0, 0.03, 0.01),       // heading
                        16.45, 1.0, 86.71, 60.75,
                        new PathConstraints(0.99, 100, 1, 1)
                )
        )
        .named("GearGirls Bot 2")
        .withHardware(new RobotConfig.HardwareNames()
                // drive motors, imu, pinpoint and limelight use the default names
                .led("led_servo"))
        // Calibration defaults are the values that were shared by every robot before 2026-09-06.
        // Measure this chassis and set its own: rightRearPowerScale, strafeScale, turnCircumferenceIn.
        .withCalibration(new RobotConfig.Calibration());
    }
}
