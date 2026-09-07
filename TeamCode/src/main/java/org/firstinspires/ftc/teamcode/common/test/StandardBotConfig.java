package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

/**
 * Config for a plain goBILDA mecanum test chassis with the default device names, used only by the
 * Common test OpModes in this folder. Not a competition robot.
 *
 * Values are exactly what RobotConfig.createDefaultStandardBotConfig() held before 2026-09-06.
 */
public final class StandardBotConfig {

    private StandardBotConfig() {}

    public static RobotConfig create() {
        return new RobotConfig(
                new RobotConfig.DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE,   // left front
                        DcMotorEx.Direction.FORWARD,   // right front
                        DcMotorEx.Direction.REVERSE,   // left rear
                        DcMotorEx.Direction.FORWARD    // right rear
                ),
                new RobotConfig.OdometryConfig(
                        -0.0, -150.0,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                new RobotConfig.ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ),
                new RobotConfig.PointToPointTuning(), // defaults are the StandardBot values
                null // no Pedro Pathing
        )
        .named("StandardBot (test chassis)");
    }
}
