package org.firstinspires.ftc.teamcode.common;

import androidx.annotation.Nullable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * WHAT THE ROBOT IS.
 *
 * Everything in here is a physical fact about one chassis: what its devices are named in the
 * Control Hub configuration, which way its motors spin, where its odometry pods sit, how its hub
 * is mounted, and the calibration numbers that make it drive straight. It changes when someone
 * rebuilds or rewires the robot. It does NOT change when the game changes.
 *
 * HOW THE ROBOT OPERATES (speeds, feed times, launcher presets, waypoints, driver choices) lives in
 * the team's Constants class instead. If a value would change when the game changes, it belongs
 * there, not here.
 *
 * One instance per physical chassis, created in that robot's own config file in its team folder
 * (for example teams/geargirls/GGBot2Config.java). Common code never names a specific robot.
 *
 * Rule of thumb for students: rewired a motor, moved the Pinpoint, renamed a device? Edit the
 * robot's config file. Want to shoot faster or drive to a different spot? Edit Constants.
 */
public class RobotConfig {

    // =================================================================================
    // NESTED CONFIGURATION DATA CLASSES
    // =================================================================================

    /**
     * Device names exactly as they appear in the Control Hub configuration.
     * Defaults are the names every robot used before 2026-09-06, so an unconfigured
     * RobotConfig behaves exactly as the old hardcoded code did.
     */
    public static final class HardwareNames {
        public String leftFront  = "Front_Left";
        public String rightFront = "Front_Right";
        public String leftRear   = "Rear_Left";
        public String rightRear  = "Rear_Right";
        public String imu        = "imu";
        /** goBILDA Pinpoint. Set to null if this robot has none; DriveUtil2026b then skips odometry. */
        public String pinpoint   = "odo";
        /** Limelight 3A. Set to null if this robot has none; VisionUtil then reports no targets. */
        public String limelight  = "limelight";
        /** Servo-driven status LED used by LedUtil. Null if this robot has none. */
        public String led        = null;

        public HardwareNames driveMotors(String leftFront, String rightFront, String leftRear, String rightRear) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftRear = leftRear;
            this.rightRear = rightRear;
            return this;
        }
        public HardwareNames imu(String name)       { this.imu = name; return this; }
        public HardwareNames pinpoint(String name)  { this.pinpoint = name; return this; }
        public HardwareNames limelight(String name) { this.limelight = name; return this; }
        public HardwareNames led(String name)       { this.led = name; return this; }
    }

    /**
     * Physical calibration numbers used by DriveUtil2026b's encoder moves and motor mixing.
     * Defaults are the values that were hardcoded in DriveUtil2026b before 2026-09-06. They were
     * tuned on one robot; each chassis should eventually measure its own.
     */
    public static final class Calibration {
        /** Extra power applied to the right-rear wheel in moveRobot(). 1.0 means no correction. */
        public double rightRearPowerScale = 1.15;
        /** Multiplier on strafe distance for encoder moves, because mecanum strafing slips. 1.0 = none. */
        public double strafeScale         = 1.1;
        /** Circumference (inches) of the circle the robot sweeps in a spin turn; used by drive_p3 turns. */
        public double turnCircumferenceIn = 27.5;
        /** Drive-motor encoder ticks per inch of robot travel. */
        public double encoderCountsPerInch = 45.33;
        /** Drive-motor encoder ticks per motor output revolution (537 for a goBILDA 312 rpm). */
        public double encoderTicksPerRev  = 537;
        /** External gear reduction between motor and wheel. 1.0 if direct drive. */
        public double gearReduction       = 1.0;
        /** Wheel diameter in centimeters. */
        public double wheelDiameterCm     = 9.6;
        /** Ticks per wheel revolution used by the simplified-odometry moves (384.5 for a goBILDA 435 rpm). */
        public double odometryTicksPerRev = 384.5;
        /** Robot turning-circle diameter in centimeters; used by rotateRobot(). */
        public double robotDiameterCm     = 60;

        public Calibration rightRearPowerScale(double v)  { this.rightRearPowerScale = v; return this; }
        public Calibration strafeScale(double v)          { this.strafeScale = v; return this; }
        public Calibration turnCircumferenceIn(double v)  { this.turnCircumferenceIn = v; return this; }
        public Calibration encoderCountsPerInch(double v) { this.encoderCountsPerInch = v; return this; }
        public Calibration encoderTicksPerRev(double v)   { this.encoderTicksPerRev = v; return this; }
        public Calibration gearReduction(double v)        { this.gearReduction = v; return this; }
        public Calibration wheelDiameterCm(double v)      { this.wheelDiameterCm = v; return this; }
        public Calibration odometryTicksPerRev(double v)  { this.odometryTicksPerRev = v; return this; }
        public Calibration robotDiameterCm(double v)      { this.robotDiameterCm = v; return this; }
    }

    /** Which way each drive motor spins so that positive power drives the robot forward. */
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

    /** Physical setup of the goBILDA Pinpoint odometry computer. Ignored if hardware.pinpoint is null. */
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

    /** How the Control Hub is mounted, so its internal IMU reports the right heading. */
    public static final class ImuConfig {
        public final RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
        public final RevHubOrientationOnRobot.UsbFacingDirection usbDirection;

        public ImuConfig(RevHubOrientationOnRobot.LogoFacingDirection logo, RevHubOrientationOnRobot.UsbFacingDirection usb) {
            this.logoDirection = logo;
            this.usbDirection = usb;
        }
    }

    /**
     * Tuning for DriveUtil2026b.driveTo(), the Pinpoint point-to-point PID.
     * Build it with the named setters so nobody has to remember the order of ten doubles:
     * <pre>
     *   new PointToPointTuning()
     *       .xyToleranceMm(18).yawToleranceRad(0.055)
     *       .xyGains(0.0035, 0.00001, 0.00035).xyAccel(8.0)
     *       .yawGains(2.5, 0.00005, 0.08).yawAccel(10.0)
     * </pre>
     * Defaults are the StandardBot values.
     */
    public static final class PointToPointTuning {
        public double xyTolerance  = 15.5;      // mm
        public double yawTolerance = 0.0349066; // rad (2 degrees)
        public double pGain        = 0.01905;
        public double iGain        = 0.000002;
        public double dGain        = 0.00111;
        public double accel        = 8.0;
        public double yawPGain     = 5.0;
        public double yawIGain     = 0.0;
        public double yawDGain     = 0.0;
        public double yawAccel     = 20.0;

        public PointToPointTuning() {}

        public PointToPointTuning xyToleranceMm(double v)  { this.xyTolerance = v; return this; }
        public PointToPointTuning yawToleranceRad(double v){ this.yawTolerance = v; return this; }
        public PointToPointTuning xyGains(double p, double i, double d) { this.pGain = p; this.iGain = i; this.dGain = d; return this; }
        public PointToPointTuning xyAccel(double v)        { this.accel = v; return this; }
        public PointToPointTuning yawGains(double p, double i, double d) { this.yawPGain = p; this.yawIGain = i; this.yawDGain = d; return this; }
        public PointToPointTuning yawAccel(double v)       { this.yawAccel = v; return this; }

        /** @deprecated Ten positional doubles are easy to transpose. Use the no-arg constructor and the named setters. */
        @Deprecated
        public PointToPointTuning(double xy, double yaw, double p, double d, double i, double a, double yp, double yd, double yi, double ya) {
            this.xyTolerance = xy;
            this.yawTolerance = yaw;
            this.pGain = p;
            this.dGain = d;
            this.iGain = i;
            this.accel = a;
            this.yawPGain = yp;
            this.yawDGain = yd;
            this.yawIGain = yi;
            this.yawAccel = ya;
        }
    }

    /** Tuning constants required by the Pedro Pathing library. Null if the robot does not use Pedro. */
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

    /** Shown in telemetry so the drive team can see which robot's config is running. */
    public String robotName = "unnamed";
    public HardwareNames hardware = new HardwareNames();
    public Calibration calibration = new Calibration();

    public final DrivetrainConfig drivetrain;
    public final OdometryConfig odometry;
    public final ImuConfig imu;
    public final PointToPointTuning pointToPointTuning;
    public final PedroPathingConfig pedroPathing; // null if the robot does not use Pedro Pathing

    /**
     * @param pedroPathingConfig Can be null if a robot doesn't use Pedro Pathing.
     */
    public RobotConfig(DrivetrainConfig drivetrain, OdometryConfig odometry, ImuConfig imu, PointToPointTuning p2pTuning, @Nullable PedroPathingConfig pedroPathingConfig) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.imu = imu;
        this.pointToPointTuning = p2pTuning;
        this.pedroPathing = pedroPathingConfig;
    }

    public RobotConfig named(String robotName)                { this.robotName = robotName; return this; }
    public RobotConfig withHardware(HardwareNames hardware)   { this.hardware = hardware; return this; }
    public RobotConfig withCalibration(Calibration calibration) { this.calibration = calibration; return this; }
}
