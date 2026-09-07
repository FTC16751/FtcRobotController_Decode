package org.firstinspires.ftc.teamcode.teams.p3;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

/**
 * WHAT THE P3 BOT 3 CHASSIS IS (the current competition robot).
 *
 * Edit this file when the robot is rewired, a device is renamed in the Control Hub config, the
 * Pinpoint is moved, or the hub is remounted. For speeds, feed times, launcher presets and
 * waypoints, edit P3RobotConstants instead.
 *
 * Values here are exactly what RobotConfig.createP3Robot2Config() held before 2026-09-06.
 * That factory was used by both P3_Robot (Bot 2) and P3_Robot3 (Bot 3), so both use this file.
 * If Bot 2 still exists as a separate chassis, give it its own P3Bot2Config.
 */
public final class P3Bot3Config {

    private P3Bot3Config() {}

    public static RobotConfig create() {
        return new RobotConfig(
                new RobotConfig.DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE,   // left front
                        DcMotorEx.Direction.FORWARD,   // right front
                        DcMotorEx.Direction.REVERSE,   // left rear
                        DcMotorEx.Direction.FORWARD    // right rear
                ),
                new RobotConfig.OdometryConfig(
                        50, -152.0,                    // Pinpoint pod offsets, mm (X pod, Y pod)
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED
                ),
                new RobotConfig.ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ),
                new RobotConfig.PointToPointTuning()
                        .xyToleranceMm(18.0).yawToleranceRad(0.055)
                        .xyGains(0.00350, 0.000010, 0.00035).xyAccel(8.0)
                        .yawGains(2.5, 0.00005, 0.08).yawAccel(10.0),
                new RobotConfig.PedroPathingConfig(
                        4.5, -30.0, -60.0,
                        new PIDFCoefficients(0.02, 0, 0.004, 0.02),     // translational
                        new PIDFCoefficients(0.6, 0, 0.035, 0.01),      // heading
                        16.0, 1.05, 80.0, 55.0,
                        new PathConstraints(0.95, 90, 1, 1)
                )
        )
        .named("P3 Bot 3")
        .withHardware(new RobotConfig.HardwareNames()
                // drive motors, imu, pinpoint and limelight use the default names
                .led("light"))
        // Calibration defaults are the values that were shared by every robot before 2026-09-06.
        // Measure this chassis and set its own: rightRearPowerScale, strafeScale, turnCircumferenceIn.
        .withCalibration(new RobotConfig.Calibration());
    }
}
