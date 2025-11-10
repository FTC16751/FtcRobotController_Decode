package org.firstinspires.ftc.teamcode.utilities.Common;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
/**
 * A data class to hold all configuration values that are specific to a
 * particular robot's physical build. This object is created in a robot-specific
 * class (like GGRobot) and passed to generic utilities (like DriveUtil2026)
 * to configure them correctly.
 */
public class RobotConfig {

    // === Drivetrain Motor Directions ===
    public final DcMotorEx.Direction leftFrontDirection;
    public final DcMotorEx.Direction rightFrontDirection;
    public final DcMotorEx.Direction leftRearDirection;
    public final DcMotorEx.Direction rightRearDirection;

    // === Pinpoint Odometry Configuration ===
    public final double pinpointOffsetX_mm;
    public final double pinpointOffsetY_mm;
    public final GoBildaPinpointDriver.EncoderDirection pinpointXPodDirection;
    public final GoBildaPinpointDriver.EncoderDirection pinpointYPodDirection;

    // === IMU Configuration ===
    public final RevHubOrientationOnRobot.LogoFacingDirection imuLogoDirection;
    public final RevHubOrientationOnRobot.UsbFacingDirection imuUsbDirection;


    /**
     * Constructor for the robot's physical configuration.
     */
    public RobotConfig(
            DcMotorEx.Direction leftFrontDirection,
            DcMotorEx.Direction rightFrontDirection,
            DcMotorEx.Direction leftRearDirection,
            DcMotorEx.Direction rightRearDirection,
            double pinpointOffsetX_mm,
            double pinpointOffsetY_mm,
            GoBildaPinpointDriver.EncoderDirection pinpointXPodDirection,
            GoBildaPinpointDriver.EncoderDirection pinpointYPodDirection,
            RevHubOrientationOnRobot.LogoFacingDirection imuLogoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection imuUsbDirection
    ) {
        this.leftFrontDirection = leftFrontDirection;
        this.rightFrontDirection = rightFrontDirection;
        this.leftRearDirection = leftRearDirection;
        this.rightRearDirection = rightRearDirection;
        this.pinpointOffsetX_mm = pinpointOffsetX_mm;
        this.pinpointOffsetY_mm = pinpointOffsetY_mm;
        this.pinpointXPodDirection = pinpointXPodDirection;
        this.pinpointYPodDirection = pinpointYPodDirection;
        this.imuLogoDirection = imuLogoDirection;
        this.imuUsbDirection = imuUsbDirection;
    }

    // =================================================================================
    //  STATIC FACTORY METHODS (The "Default" Templates)
    // =================================================================================

    /**
     * Creates the default, known-good configuration for the Gear Girls robot.
     * A new programmer can use this with one line of code.
     * @return A pre-populated RobotConfig object for the Gear Girls bot.
     */
    public static RobotConfig createDefaultGearGirlsConfig() {
        return new RobotConfig(
                /* Motor Directions */
                DcMotorEx.Direction.REVERSE,  // Left Front
                DcMotorEx.Direction.FORWARD,  // Right Front
                DcMotorEx.Direction.REVERSE,  // Left Rear
                DcMotorEx.Direction.FORWARD,   // Right Rear

                /* Pinpoint Config */
                -0.0,    // Offset X in mm
                -150.0,  // Offset Y in mm
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X Pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // Y Pod

                /* IMU Config */
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
    }

    /**
     * Creates the default, known-good configuration for the P3 robot.
     * @return A pre-populated RobotConfig object for the P3 bot.
     */
    public static RobotConfig createDefaultP3Config() {
        return new RobotConfig(
                /* Motor Directions */
                DcMotorEx.Direction.REVERSE,  // Left Front
                DcMotorEx.Direction.FORWARD,  // Right Front
                DcMotorEx.Direction.REVERSE,  // Left Rear
                DcMotorEx.Direction.FORWARD,   // Right Rear

                /* Pinpoint Config */
                -10.5,   // P3's X offset
                -145.0,  // P3's Y offset
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X Pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // Y Pod

                /* IMU Config */
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
    }
    public static RobotConfig createDefaultSkyLineConfig() {
        return new RobotConfig(
                /* Motor Directions */
                DcMotorEx.Direction.REVERSE,  // Left Front
                DcMotorEx.Direction.FORWARD,  // Right Front
                DcMotorEx.Direction.REVERSE,  // Left Rear
                DcMotorEx.Direction.FORWARD,   // Right Rear

                /* Pinpoint Config */
                -0.0,    // Offset X in mm
                -150.0,  // Offset Y in mm
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X Pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // Y Pod

                /* IMU Config */
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
    }
    /**
     * A generic "Standard Bot" config for new prototypes.
     * @return A pre-populated RobotConfig object for a standard build.
     */
    public static RobotConfig createDefaultStandardBotConfig() {
            return new RobotConfig(
                    /* Motor Directions */
                    DcMotorEx.Direction.REVERSE,  // Left Front
                    DcMotorEx.Direction.FORWARD,  // Right Front
                    DcMotorEx.Direction.REVERSE,  // Left Rear
                    DcMotorEx.Direction.FORWARD,   // Right Rear

                    /* Pinpoint Config */
                    -0.0,    // Offset X in mm
                    -150.0,  // Offset Y in mm
                    GoBildaPinpointDriver.EncoderDirection.FORWARD, // X Pod
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,   // Y Pod

                    /* IMU Config */
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            );

    }
}
