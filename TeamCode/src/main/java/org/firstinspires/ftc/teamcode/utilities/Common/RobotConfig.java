package org.firstinspires.ftc.teamcode.utilities.Common;
// Import all necessary classes from hardware and third-party libraries
import androidx.annotation.Nullable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * A data class to hold all configuration values that are specific to a
 * particular robot's physical build and tuning profile. An instance of this
 * class is created in a robot-specific factory method (e.g., createDefaultGearGirlsConfig)
 * and passed to generic utilities (like DriveUtil2026b) to configure them correctly.
 */
public class RobotConfig {

    // =================================================================================
    // NESTED CONFIGURATION DATA CLASSES
    // Each class holds a logical group of constants.
    // =================================================================================

    /**
     * Contains the physical motor direction settings for the drivetrain.
     */
    public static final class DrivetrainConfig {
        public final DcMotorEx.Direction leftFrontDirection;
        public final DcMotorEx.Direction rightFrontDirection;
        public final DcMotorEx.Direction leftRearDirection;
        public final DcMotorEx.Direction rightRearDirection;

        public DrivetrainConfig(DcMotorEx.Direction lF, DcMotorEx.Direction rF, DcMotorEx.Direction lR, DcMotorEx.Direction rR) {
            this.leftFrontDirection = lF;
            this.rightFrontDirection = rF;
            this.leftRearDirection = lR;
            this.rightRearDirection = rR;
        }
    }

    /**
     * Contains the physical setup for the GoBilda Pinpoint odometry module.
     */
    public static final class OdometryConfig {
        public final double pinpointOffsetX_mm;
        public final double pinpointOffsetY_mm;
        public final GoBildaPinpointDriver.EncoderDirection pinpointXPodDirection;
        public final GoBildaPinpointDriver.EncoderDirection pinpointYPodDirection;

        public OdometryConfig(double offsetX, double offsetY, GoBildaPinpointDriver.EncoderDirection xDir, GoBildaPinpointDriver.EncoderDirection yDir) {
            this.pinpointOffsetX_mm = offsetX;
            this.pinpointOffsetY_mm = offsetY;
            this.pinpointXPodDirection = xDir;
            this.pinpointYPodDirection = yDir;
        }
    }

    /**
     * Contains the physical orientation of the Control Hub's internal IMU.
     */
    public static final class ImuConfig {
        public final RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
        public final RevHubOrientationOnRobot.UsbFacingDirection usbDirection;

        public ImuConfig(RevHubOrientationOnRobot.LogoFacingDirection logo, RevHubOrientationOnRobot.UsbFacingDirection usb) {
            this.logoDirection = logo;
            this.usbDirection = usb;
        }
    }

    /**
     * Contains tuning values for the simple "driveToPoint" style of autonomous.
     */
    public static final class PointToPointTuning {
        public final double xyTolerance;
        public final double yawTolerance;
        public final double pGain;
        public final double dGain;
        public final double iGain;
        public final double accel;
        public final double yawPGain;
        public final double yawDGain;
        public final double yawAccel;

        public PointToPointTuning(double xy, double yaw, double p, double d, double i, double a, double yp, double yd, double ya) {
            this.xyTolerance = xy;
            this.yawTolerance = yaw;
            this.pGain = p;
            this.dGain = d;
            this.iGain = i;
            this.accel = a;
            this.yawPGain = yp;
            this.yawDGain = yd;
            this.yawAccel = ya;
        }
    }

    /**
     * Contains all tuning constants required by the Pedro Pathing library.
     */
    public static final class PedroPathingConfig {
        public final double followerMass;
        public final double forwardZeroPowerAccel;
        public final double lateralZeroPowerAccel;
        public final PIDFCoefficients translationalPIDF;
        public final PIDFCoefficients headingPIDF;
        public final double trackWidth;
        public final double lateralMultiplier;
        public final double driveMaxVelo;
        public final double strafeMaxVelo;
        public final PathConstraints pathConstraints;

        public PedroPathingConfig(double mass, double fwdAccel, double latAccel, PIDFCoefficients transPIDF, PIDFCoefficients headPIDF, double track, double latMulti, double driveVelo, double strafeVelo, PathConstraints constraints) {
            this.followerMass = mass;
            this.forwardZeroPowerAccel = fwdAccel;
            this.lateralZeroPowerAccel = latAccel;
            this.translationalPIDF = transPIDF;
            this.headingPIDF = headPIDF;
            this.trackWidth = track;
            this.lateralMultiplier = latMulti;
            this.driveMaxVelo = driveVelo;
            this.strafeMaxVelo = strafeVelo;
            this.pathConstraints = constraints;
        }
    }

    // =================================================================================
    // MAIN RobotConfig CLASS MEMBERS
    // =================================================================================

    public final DrivetrainConfig drivetrain;
    public final OdometryConfig odometry;
    public final ImuConfig imu;
    public final PointToPointTuning pointToPointTuning;
    public final PedroPathingConfig pedroPathing; // This can be null

    /**
     * The main constructor now takes these organized data objects.
     * @param pedroPathingConfig Can be null if a robot doesn't use Pedro Pathing.
     */
    public RobotConfig(DrivetrainConfig drivetrain, OdometryConfig odometry, ImuConfig imu, PointToPointTuning p2pTuning, @Nullable PedroPathingConfig pedroPathingConfig) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.imu = imu;
        this.pointToPointTuning = p2pTuning;
        this.pedroPathing = pedroPathingConfig;
    }

    // =================================================================================
    //  STATIC FACTORY METHODS (The "Default" Templates for each robot)
    // =================================================================================

    /**
     * Creates the complete configuration profile for the Gear Girls robot.
     */
    public static RobotConfig createDefaultGearGirlsConfig() {
        return new RobotConfig(
                new DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE,
                        DcMotorEx.Direction.FORWARD,
                        DcMotorEx.Direction.REVERSE,
                        DcMotorEx.Direction.FORWARD
                ),
                new OdometryConfig(
                        -0.0, -203.0,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                new ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                ),
                new PointToPointTuning(
                        17.0,
                        0.0349,
                        0.006,
                        0.00003,
                        0.000002,
                        10.0,
                        5.0,
                        0.0,
                        20.0
                ),
                // Gear Girls robot uses Pedro Pathing, so we provide its config.
                new PedroPathingConfig(
                        5.0, -34.46, -64.23,
                        new PIDFCoefficients(0.01905, 0, 0.0035, 0.02), // translational
                        new PIDFCoefficients(0.5, 0, 0.03, 0.01),      // heading
                        16.45, 1.0, 86.71, 60.75,
                        new PathConstraints(0.99, 100, 1, 1)
                )

        );
    }

    /**
     * Creates the complete configuration profile for the P3 robot.
     */
    public static RobotConfig createDefaultP3Config() {
        return new RobotConfig(
                new DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE, DcMotorEx.Direction.FORWARD,
                        DcMotorEx.Direction.REVERSE, DcMotorEx.Direction.FORWARD
                ),
                new OdometryConfig(
                        -38.0, 165.0,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                new ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP
                ),
                new PointToPointTuning(
                        17.0, 0.055, 0.002, 0.00003, 0.000002,
                        10.0, 5.0, 0.03, 10.0
                ),
                // P3 robot also uses Pedro Pathing, but with different tuning.
                new PedroPathingConfig(
                        4.5, -30.0, -60.0,
                        new PIDFCoefficients(0.02, 0, 0.004, 0.02),    // translational
                        new PIDFCoefficients(0.6, 0, 0.035, 0.01),      // heading
                        16.0, 1.05, 80.0, 55.0,
                        new PathConstraints(0.95, 90, 1, 1)
                )
        );
    }
    public static RobotConfig createP3Robot2Config() {
        return new RobotConfig(
                new DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE, DcMotorEx.Direction.FORWARD,
                        DcMotorEx.Direction.REVERSE, DcMotorEx.Direction.FORWARD
                ),
                new OdometryConfig(
                        -38.0, 165.0,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED
                ),
                new ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP
                ),
                new PointToPointTuning(
                        17.0, 0.055, 0.002, 0.00003, 0.000002,
                        10.0, 5.0, 0.03, 10.0
                ),
                // P3 robot also uses Pedro Pathing, but with different tuning.
                new PedroPathingConfig(
                        4.5, -30.0, -60.0,
                        new PIDFCoefficients(0.02, 0, 0.004, 0.02),    // translational
                        new PIDFCoefficients(0.6, 0, 0.035, 0.01),      // heading
                        16.0, 1.05, 80.0, 55.0,
                        new PathConstraints(0.95, 90, 1, 1)
                )
        );
    }
    /**
     * Creates the configuration for Skyline, assuming it does NOT use Pedro Pathing.
     */
    public static RobotConfig createDefaultSkyLineConfig() {
        return new RobotConfig(
                new DrivetrainConfig(
                        DcMotorEx.Direction.FORWARD, DcMotorEx.Direction.REVERSE,
                        DcMotorEx.Direction.FORWARD, DcMotorEx.Direction.REVERSE
                ),
                new OdometryConfig(
                        -0.0, -150.0,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                new ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ),
                new PointToPointTuning( // It has its own simple PID tuning
                        15.5, 0.0349066, 0.01905, 0.00111, 0.000002,
                        8.0, 5.0, 0.0, 20.0
                ),
                // This robot does NOT use Pedro Pathing, so we pass null.
                null
        );
    }

    public static RobotConfig createDefaultStandardBotConfig() {
        return new RobotConfig(
                new DrivetrainConfig(
                        DcMotorEx.Direction.REVERSE,  // Left Front
                        DcMotorEx.Direction.FORWARD,  // Right Front
                        DcMotorEx.Direction.REVERSE,  // Left Rear
                        DcMotorEx.Direction.FORWARD   // Right Rear
                ),
                new OdometryConfig(
                        -0.0, -150.0,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD
                ),
                new ImuConfig(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ),
                new PointToPointTuning( // It has its own simple PID tuning
                        15.5, 0.0349066, 0.01905, 0.00111, 0.000002,
                        8.0, 5.0, 0.0, 20.0
                ),
                // This robot does NOT use Pedro Pathing, so we pass null.
                null
        );
    }
}