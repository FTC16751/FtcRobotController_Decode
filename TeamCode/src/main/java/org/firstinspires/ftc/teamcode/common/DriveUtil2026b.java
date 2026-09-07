package org.firstinspires.ftc.teamcode.common;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;


import java.util.Arrays;
import java.util.List;

/**
 * The shared drive utility. It exists so a new programmer can get a simple autonomous running
 * quickly through helper functions that hide the complexity of moving a robot.
 *
 * START HERE: the BEGINNER COMMANDS section just below the constructor. driveForward(24),
 * turnLeft(90), strafeRight(12), waitSeconds(0.5), stop(). Each one blocks until it is done, has a
 * time limit so a stuck wheel cannot hang the auto, and uses the team's default speed unless you
 * give one. Everything after that section is for later: encoder moves with three components,
 * Pinpoint waypoints (driveTo), and the AprilTag approach.
 *
 * Conventions everywhere in this file: forward is positive, strafe is positive to the RIGHT,
 * turns are positive CLOCKWISE. The beginner commands put the direction in the name so you never
 * have to remember that.
 */
public class DriveUtil2026b {
    // =================================================================================
    // SECTION 1: CLASS MEMBERS AND CONSTANTS
    // =================================================================================

    // --- Robot Physical Constants ---
    // All come from RobotConfig.calibration (see the robot's config file in its team folder).
    // Every encoder move converts through this one number; turns add calibration.turnCircumferenceIn.
    private final double ENCODER_COUNTS_PER_INCH;   // drive-motor ticks per inch of travel
    // Encoder-move time limits (driveRobotToPosition). 60 in/s is a little under what a goBILDA
    // 312 rpm motor on a 96 mm wheel free-runs, so the estimate errs long; the limit is a safety
    // net, not a stopwatch.
    private static final double DRIVE_MAX_INCHES_PER_SEC = 60.0;
    private static final double MOVE_MIN_TIMEOUT_SEC = 3.0;
    private static final long   MOVE_POLL_MS = 10;

    // --- Drivetrain Motor Members ---
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftRearMotor;
    public DcMotorEx rightRearMotor;
    private List<DcMotorEx> motors;
    // Device names come from RobotConfig.hardware.

    // --- IMU & Sensor Members ---
    public IMU imu;
    private double headingOffset = 0;

    // --- GoBilda Pinpoint Odometry Members ---
    public GoBildaPinpointDriver pinpoint;

    // --- AprilTag approach (driveToTagAsync) ---
    private TagApproach tagApproach;          // the math; built from config.tagApproach in the constructor
    private TagSighting tagSighting;          // usually the robot's VisionUtil, handed in per call


    private final ElapsedTime GBholdTimer = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();
    // Point-to-point tuning (xy/yaw tolerance, P/I/D gains, accel) comes from config.pointToPointTuning.

    // --- PID & Autonomous Control Members ---
    /* for gobildas pid control */
    private final PinpointPIDLoop xPID = new PinpointPIDLoop();
    private final PinpointPIDLoop yPID = new PinpointPIDLoop();
    private final PinpointPIDLoop hPID = new PinpointPIDLoop();

    // --- Defaults for the beginner and intermediate commands. Set once by the robot class from
    //     the team's Constants (how the robot operates), see setDefaultSpeeds() and
    //     setDefaultHoldTime(). Safe values if never set.
    private double defaultDriveSpeed  = 0.4;
    private double defaultTurnSpeed   = 0.3;
    private double defaultHoldTimeSec = 0.25;

    // --- Async waypoint drive (startDriveTo), stepped by update() under DRIVING_TO_POINT_PINPOINT ---
    private Pose2D asyncTarget;
    private double asyncPower;
    private double asyncHoldTimeSec;
    private double asyncTimeoutSec;
    private final ElapsedTime asyncTimer = new ElapsedTime();
    private boolean lastMoveSucceeded = false;      // result of the most recent async move (waypoint or tag)

    // --- Field-centric TeleOp: which way is "forward" for the driver (see resetFieldForward) ---
    private double fieldForwardOffsetRad = 0.0;

    // --- General Members ---
    private Telemetry telemetry;
    private OpMode myOpMode; // Even if unused, grouping it here is correct.
    private RobotConfig config; // The injected robot configuration object

    // --- Enums ---
    private enum Direction { x, y, h }
    private enum DriveState { IDLE, DRIVING_TO_POINT_PINPOINT, ALIGNING_TO_APRILTAG }
    private DriveState driveState = DriveState.IDLE;

    /**
     * Legacy field. Only the removed simplified-odometry loops ever wrote it, so it has always read 0.
     * Ten P3 autos still put it in telemetry as "imu heading"; they should call getHeading() instead.
     * Kept so they compile. Do not use in new code.
     */
    public double heading           = 0;
    // The Follower can be null if this robot doesn't use it.
    //public final Follower follower;

    // =================================================================================
    // SECTION 2: CONSTRUCTOR & INITIALIZATION
    // =================================================================================

    public DriveUtil2026b(HardwareMap hardwareMap, Telemetry telemetry, OpMode opMode, RobotConfig config) {
        this.telemetry = telemetry;
        this.config = config;
        this.tagApproach = new TagApproach(config.tagApproach);

        // Physical constants from the robot's config
        ENCODER_COUNTS_PER_INCH  = config.calibration.encoderCountsPerInch;

        // Initialize all hardware components
        initializeIMU(hardwareMap);
        initMotors(hardwareMap);
        initOdo(hardwareMap);

//        if (config.pedroPathing != null) {
//            // This robot wants smooth driving. Create the Follower.
//            this.follower = Constants.createFollower(hardwareMap, config);
//            this.follower.setStartingPose(new Pose(0, 0, 0));
//            this.follower.startTeleopDrive(true);
//            telemetry.log().add("DriveUtil: Initialized with Pedro Pathing Follower.");
//        } else {
//            // This robot does NOT use Pedro. The follower remains null.
//            this.follower = null;
//            telemetry.log().add("DriveUtil: Initialized in Simple PID Mode.");
//           ;
//        }
    }

    private void initMotors(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, config.hardware.leftFront);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, config.hardware.rightFront);
        leftRearMotor = hardwareMap.get(DcMotorEx.class, config.hardware.leftRear);
        rightRearMotor = hardwareMap.get(DcMotorEx.class, config.hardware.rightRear);
        motors = Arrays.asList(leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor);

        // Use the injected config for directions
        leftFrontMotor.setDirection(config.drivetrain.leftFrontDirection);
        rightFrontMotor.setDirection(config.drivetrain.rightFrontDirection);
        leftRearMotor.setDirection(config.drivetrain.leftRearDirection);
        rightRearMotor.setDirection(config.drivetrain.rightRearDirection);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, config.hardware.imu);
        // Use the injected config for IMU orientation
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(config.imu.logoDirection, config.imu.usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetHeading();
    }

    /**
     * The Pinpoint is optional. A robot without one sets hardware.pinpoint to null in its config;
     * the encoder-based moves (drive_p3, driveRobotDistance*) and TeleOp driving still work, and
     * the odometry-based methods (driveTo, getOdoPosition, getPinpointHeading) report zero.
     */
    private void initOdo(HardwareMap hardwareMap) {
        if (config.hardware.pinpoint == null) {
            pinpoint = null;
            telemetry.addData("DriveUtil", "No Pinpoint in this robot's config; odometry disabled");
            return;
        }
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.hardware.pinpoint);
        configurePinpoint();
    }

    /** True if this robot has a Pinpoint and it was initialized. */
    public boolean hasPinpoint() {
        return pinpoint != null;
    }

    private void configurePinpoint() {
        // Use the injected config for Pinpoint setup
        pinpoint.setOffsets(config.odometry.pinpointOffsetX_mm, config.odometry.pinpointOffsetY_mm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(config.odometry.pinpointXPodDirection, config.odometry.pinpointYPodDirection);
        pinpoint.resetPosAndIMU();
    }

    // =================================================================================
    // BEGINNER COMMANDS (START HERE)
    // =================================================================================
    //
    // Everything a first autonomous needs. The direction is in the name, distances are in inches,
    // turns in degrees. Every command blocks until the move is done (or its time limit passes, or
    // the Driver Station presses Stop) and returns true if the wheels reached their targets.
    // Speed is optional: leave it out and the team's default is used.
    //
    //   robot.drive.driveForward(24);
    //   robot.drive.turnLeft(90);
    //   robot.drive.strafeRight(12);
    //   robot.drive.waitSeconds(0.5);
    //   robot.drive.stop();
    //
    // These are the encoder moves underneath (drive_p3 and the driveRobotDistance* family), so
    // they use the calibration numbers measured with the Encoder Move Check OpMode. No Pinpoint
    // or camera is needed, except by driveToTag.

    /**
     * Set the speeds the beginner commands use when no speed is given. The robot class calls
     * this once from the team's Constants. Powers are 0 to 1.
     */
    public void setDefaultSpeeds(double driveSpeed, double turnSpeed) {
        defaultDriveSpeed = driveSpeed;
        defaultTurnSpeed  = turnSpeed;
    }
    public double getDefaultDriveSpeed() { return defaultDriveSpeed; }
    public double getDefaultTurnSpeed()  { return defaultTurnSpeed; }

    /** How long startDriveTo holds inside tolerance before it counts as arrived, when not given. */
    public void setDefaultHoldTime(double seconds) { defaultHoldTimeSec = seconds; }
    public double getDefaultHoldTime()             { return defaultHoldTimeSec; }

    /** Drive straight ahead this many inches at the default speed. */
    public boolean driveForward(double inches)                { return driveForward(inches, defaultDriveSpeed); }
    /** Drive straight ahead this many inches. Speed is motor power, 0 to 1. */
    public boolean driveForward(double inches, double speed)  { return driveRobotDistanceForwardInches(Math.abs(inches), speed); }

    /** Drive straight back this many inches at the default speed. */
    public boolean driveBackward(double inches)               { return driveBackward(inches, defaultDriveSpeed); }
    /** Drive straight back this many inches. Speed is motor power, 0 to 1. */
    public boolean driveBackward(double inches, double speed) { return driveRobotDistanceBackwardInches(Math.abs(inches), speed); }

    /** Slide left this many inches without turning, at the default speed. */
    public boolean strafeLeft(double inches)                  { return strafeLeft(inches, defaultDriveSpeed); }
    /** Slide left this many inches without turning. Speed is motor power, 0 to 1. */
    public boolean strafeLeft(double inches, double speed)    { return driveRobotDistanceStrafeLeftInches(Math.abs(inches), speed); }

    /** Slide right this many inches without turning, at the default speed. */
    public boolean strafeRight(double inches)                 { return strafeRight(inches, defaultDriveSpeed); }
    /** Slide right this many inches without turning. Speed is motor power, 0 to 1. */
    public boolean strafeRight(double inches, double speed)   { return driveRobotDistanceStrafeRightInches(Math.abs(inches), speed); }

    /** Turn left (counter-clockwise) in place this many degrees, at the default turn speed. */
    public boolean turnLeft(double degrees)                   { return turnLeft(degrees, defaultTurnSpeed); }
    /** Turn left (counter-clockwise) in place this many degrees. Speed is motor power, 0 to 1. */
    public boolean turnLeft(double degrees, double speed)     { return drive_p3(0, 0, -Math.abs(degrees), speed); }

    /** Turn right (clockwise) in place this many degrees, at the default turn speed. */
    public boolean turnRight(double degrees)                  { return turnRight(degrees, defaultTurnSpeed); }
    /** Turn right (clockwise) in place this many degrees. Speed is motor power, 0 to 1. */
    public boolean turnRight(double degrees, double speed)    { return drive_p3(0, 0, Math.abs(degrees), speed); }

    /**
     * Do nothing for this long. Ends early if the Driver Station presses Stop. Use it to let a
     * launcher spin up or a servo finish before the next move.
     */
    public void waitSeconds(double seconds) {
        sleep((long) (Math.max(0, seconds) * 1000));
    }

    /** Stop all four wheels. */
    public void stop() {
        stopRobot();
    }

    /**
     * Drive to a spot in front of an AprilTag and square up to it, then stop. Blocks until the
     * robot is there, the tag has been out of view too long, or the time limit passes. Needs the
     * robot's camera; the gains come from the robot's config (tagApproach).
     *
     * @param vision         the robot's VisionUtil
     * @param tagId          the AprilTag id to drive to
     * @param standoffInches how far in front of the tag to stop
     * @return true if the robot got there; false if it gave up (see getTagApproach().getState())
     */
    public boolean driveToTag(VisionUtil vision, int tagId, double standoffInches) {
        driveToTagAsync(vision, tagId, standoffInches, 0.25);
        while (isBusy()) {
            if (Thread.currentThread().isInterrupted()) {
                cancelDriveToTag();
                return false;
            }
            vision.update();
            update();
            sleep(MOVE_POLL_MS);
        }
        return lastTagApproachSucceeded();
    }

    // =================================================================================
    // INTERMEDIATE COMMANDS: the robot knows where it is
    // =================================================================================
    //
    // Two new ideas over the beginner commands. First, a position on the field: the Pinpoint
    // odometry computer tracks X (inches forward from where it was zeroed), Y (inches to the
    // LEFT), and heading (degrees, counter-clockwise positive). Second, doing two things at once:
    // the start* commands return immediately, the robot classes' update() moves the robot a little
    // each loop, and the auto polls isBusy() while a launcher or intake runs in the same loop.
    //
    //   case DRIVE:  robot.drive.startDriveTo(24, 12, 90);  state = WAIT;  break;
    //   case WAIT:   if (!robot.drive.isBusy()) { ... next step ... }     break;
    //
    // Power and hold time default from the team's Constants (setDefaultSpeeds, setDefaultHoldTime).
    // Every start* move has a time limit; lastMoveSucceeded() says whether it arrived or gave up.
    // A robot without a Pinpoint: the getters return 0 and start* moves finish at once, failed.

    /** X position in inches, forward from where the position was last zeroed. 0 without a Pinpoint. */
    public double getX() {
        return pinpoint == null ? 0.0 : pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    /** Y position in inches, to the LEFT of where the position was last zeroed. 0 without a Pinpoint. */
    public double getY() {
        return pinpoint == null ? 0.0 : pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    /** Heading in degrees, counter-clockwise positive, 0 where the position was last zeroed. */
    public double getHeadingDegrees() {
        return pinpoint == null ? 0.0 : pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /** The current position as one object, for waypoint math and telemetry. Zero without a Pinpoint. */
    public Pose2D getPose() {
        if (pinpoint == null) return pose(0, 0, 0);
        return pinpoint.getPosition();
    }

    /** Tell the odometry where the robot is, for example the start tile at the beginning of an auto. */
    public void setPosition(double xInches, double yInches, double headingDegrees) {
        if (pinpoint != null) pinpoint.setPosition(pose(xInches, yInches, headingDegrees));
    }

    /** Make here (0, 0) facing heading 0. Instant; the IMU is not recalibrated (see resetPosAndIMU). */
    public void resetPosition() {
        setPosition(0, 0, 0);
    }

    /** Build a field position: inches forward, inches left, degrees counter-clockwise. */
    public static Pose2D pose(double xInches, double yInches, double headingDegrees) {
        return new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees);
    }

    /**
     * One encoder move with all three components at once, blocking: forward inches (negative for
     * back), right inches (negative for left), and degrees clockwise (negative for left). The
     * readable name for drive_p3.
     */
    public boolean move(double forwardInches, double rightInches, double turnDegrees) {
        return move(forwardInches, rightInches, turnDegrees, defaultDriveSpeed);
    }
    public boolean move(double forwardInches, double rightInches, double turnDegrees, double speed) {
        return drive_p3(forwardInches, rightInches, turnDegrees, speed);
    }

    /**
     * Start driving to a field position and return at once. Call update() every loop (the robot
     * classes do) and poll isBusy(). Uses the Pinpoint point-to-point PID and the tuning in the
     * robot's config. Power and hold time are the defaults; the time limit is generous and scales
     * with the distance.
     */
    public void startDriveTo(double xInches, double yInches, double headingDegrees) {
        startDriveTo(pose(xInches, yInches, headingDegrees));
    }
    public void startDriveTo(double xInches, double yInches, double headingDegrees, double power) {
        startDriveTo(pose(xInches, yInches, headingDegrees), power);
    }
    public void startDriveTo(Pose2D target) {
        startDriveTo(target, defaultDriveSpeed);
    }
    public void startDriveTo(Pose2D target, double power) {
        startDriveTo(target, power, defaultHoldTimeSec, defaultDriveToTimeoutSec(target, power));
    }
    /** Full control: explicit power, hold time inside tolerance, and time limit. */
    public void startDriveTo(Pose2D target, double power, double holdTimeSec, double timeoutSec) {
        cancel();
        if (pinpoint == null) {
            lastMoveSucceeded = false;
            return;                       // nothing to navigate with; isBusy() stays false
        }
        asyncTarget      = target;
        asyncPower       = power;
        asyncHoldTimeSec = holdTimeSec;
        asyncTimeoutSec  = timeoutSec;
        asyncTimer.reset();
        xPID.pidReset(); yPID.pidReset(); hPID.pidReset();
        GBholdTimer.reset();
        driveState = DriveState.DRIVING_TO_POINT_PINPOINT;
    }

    /** Turn in place to face a field heading (degrees, counter-clockwise positive), non-blocking. */
    public void turnToHeading(double headingDegrees) {
        startDriveTo(getX(), getY(), headingDegrees, defaultTurnSpeed);
    }

    /** The non-blocking tag approach under its tier name. Same as driveToTagAsync with the default hold. */
    public void startDriveToTag(TagSighting sighting, int tagId, double standoffInches) {
        driveToTagAsync(sighting, tagId, standoffInches, defaultHoldTimeSec);
    }

    /** Abandon whatever async move is running (waypoint or tag) and stop the wheels. Safe when idle. */
    public void cancel() {
        if (driveState == DriveState.ALIGNING_TO_APRILTAG) tagApproach.stop();
        if (driveState != DriveState.IDLE) {
            stopRobot();
            lastMoveSucceeded = false;
            driveState = DriveState.IDLE;
        }
    }

    /** True if the most recent async move (startDriveTo, turnToHeading, startDriveToTag) arrived rather than gave up. */
    public boolean lastMoveSucceeded() {
        return lastMoveSucceeded;
    }

    /**
     * Field-centric TeleOp: make the direction the robot is facing right now "forward" for the
     * driver. Only fieldCentricDrive uses this; waypoints and autos keep the Pinpoint's own frame.
     */
    public void resetFieldForward() {
        fieldForwardOffsetRad = getPinpointHeading();
    }

    /** Time limit for a startDriveTo: three times the straight-line time at this power, plus 3 s, never under 3 s. */
    private double defaultDriveToTimeoutSec(Pose2D target, double power) {
        double p = Math.max(Math.abs(power), 0.05);
        double inches = distanceTo(getPose(), target, DistanceUnit.INCH);
        double idealSec = inches / (DRIVE_MAX_INCHES_PER_SEC * p);
        return Math.max(MOVE_MIN_TIMEOUT_SEC, idealSec * 3.0 + 3.0);
    }

    // =================================================================================
    // SECTION 3: SIMPLE HARDWARE HELPER METHODS
    // =================================================================================

    /* Setter(resetter) Methods*/
    public void setMotorMode(DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightRearMotor.setPower(rightRearPower);
        rightFrontMotor.setPower(rightFrontPower);
    }

    public void setMotorPowers(List<Double> powers) {
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public void stopRobot() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }
    /** Same as stopRobot(); kept for the GearGirls callers. */
    public void stopMotors() {
        stopRobot();
    }
    public void resetEncoders() {
        // Stop and reset encoders for all motors
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        // Set all motors to RUN_USING_ENCODER mode
        setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void resetHeading() {
        headingOffset = getHeading();
    }

    public void resetPosAndIMU() {
        if (pinpoint != null) pinpoint.resetPosAndIMU();
    }

     public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        } else {
            // Log an error to the Driver Station or Logcat for debugging
            telemetry.addData("RobotHardware", "IMU is not initialized. Cannot reset yaw.");
        }
    }
    /* Getter Methods */

    public double getmotorPower(DcMotorEx motor) {
        /* tested */
        return motor.getPower();
    }

    public double getmotorPosition(DcMotorEx motor) {
        /* tested */
        return motor.getCurrentPosition();
    }

    private boolean areMotorsBusy() {
        for (DcMotorEx motor : motors) {
            if (motor != null && motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (orientation.getYaw(AngleUnit.DEGREES) - headingOffset);
    }

    public double getPinpointHeading() {
        if (pinpoint == null) return 0.0;
        return pinpoint.getHeading(RADIANS);
    }
    /**
     * The robot's position from the Pinpoint. Same as getPose(), which is the current name; this
     * one stays for the P3 autos. Until 2026-09-07 it also wrote five telemetry lines on every
     * call, so a getter used a dozen times per loop flooded the Driver Station. The position lines
     * now come from addTelemetry(), once per loop.
     */
    public Pose2D getOdoPosition() {
        return getPose();
    }

    public boolean isBusy() {
        return driveState != DriveState.IDLE;
    }

    public void addTelemetry() {
        telemetry.addLine("--- drive telemetry ---");
        telemetry.addData("robot config", config.robotName);
        if (pinpoint == null) {
            telemetry.addData("odometry", "none (no Pinpoint in config)");
            return;
        }
        telemetry.addData("current X coordinate", pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("current Y coordinate", pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("current Heading angle", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

    }
    /**** CONVERSION METHODS *********/

    /**
     * Converts a distance from encoder ticks to inches.
     *
     * @param ticks The distance measured in encoder ticks.
     * @return The equivalent distance in inches.
     */
    public double fromEncoderTicksToInches(double ticks) {
        return ticks / ENCODER_COUNTS_PER_INCH;
    }

    /**
     * Converts a distance from inches to encoder ticks.
     *
     * @param distanceInches The distance to convert, specified in inches.
     * @return The equivalent number of encoder ticks as a double.
     */
    public double inchesToEncoderTicks(double distanceInches) {
        return distanceInches * ENCODER_COUNTS_PER_INCH;
    }

    // =================================================================================
    // SECTION 4: MID-LEVEL DRIVE METHODS (TELEOP)
    // =================================================================================

    /**
     * The one place wheel powers are set from a drive command. drive +forward, strafe +right,
     * yaw +clockwise. The mixing and normalization are MecanumMixer.mix (pure, unit-tested).
     */
    public void moveRobot(double drive, double strafe, double yaw) {
        MecanumMixer.Powers p = MecanumMixer.mix(drive, strafe, yaw, config.calibration.rightRearPowerScale);
        setMotorPowers(p.leftFront, p.leftRear, p.rightRear, p.rightFront);
    }

    public void arcadeDrive(double strafe, double drive, double turn, double rightStickY, double speed) {
       // if (follower != null) {
            // If the follower exists, use it for smooth, stateful control.
           // follower.setTeleOpDrive(drive, strafe, turn, true);
        //} else {
            // This is where you would apply smoothing/deadband if desired,
            // or just pass the raw values to moveRobot.
            moveRobot(drive * speed, strafe * speed, turn * speed);
       // }
    }

    public void fieldCentricDrive(double strafe, double drive, double turn, double speed) {
//        if (follower != null) {
//            // If the follower exists, use it for smooth, stateful control.
//            follower.setTeleOpDrive(drive, strafe, turn, false);
//        } else {
            // Heading from the Pinpoint (radians, counter-clockwise positive). No Pinpoint: 0,
            // and this silently becomes robot-centric driving.
            double botHeading = getPinpointHeading() - fieldForwardOffsetRad;

            // Rotate the stick command from the field frame into the robot frame. The stick's
            // strafe is right-positive; fieldToRobot takes field-left, hence the minus.
            MecanumMixer.Command c = MecanumMixer.fieldToRobot(drive, -strafe, botHeading);
            moveRobot(c.drive * speed, c.strafeRight * speed, turn * speed);
        //}
    }

    public void simpleTankDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
        // Negate stick values because joysticks typically return negative for forward.
        double leftPower = -left_stick_y * DRIVE_SPEED;
        double rightPower = -right_stick_y * DRIVE_SPEED;

        // The Range.clip method is not needed here if driveSpeed <= 1.0 and joystick
        // inputs are within [-1.0, 1.0]. The results will already be in range.

        // Send calculated power to wheels
        setMotorPowers(leftPower, leftPower, rightPower, rightPower);
    }

    // =================================================================================
    // SECTION 4B: AUTONOMOUS DRIVE ACTIONS BUT NOT PID
    // =================================================================================
    /**
     * Drives the robot to a specified target position for each motor using encoder counts.
     * <p>
     * This method is a core component for encoder-based autonomous movements. It configures each
     * of the four drive motors to run to a specific encoder tick count. The method will block
     * (wait) until all motors have reached their target positions before returning.
     * <p>
     * The process involves:
     * <ol>
     *     <li>Stopping the robot and resetting the motor encoders.</li>
     *     <li>Setting the target encoder position for each motor.</li>
     *     <li>Switching the motors to {@link DcMotor.RunMode#RUN_TO_POSITION} mode.</li>
     *     <li>Applying power to the motors to begin the movement.</li>
     *     <li>Monitoring the motors' {@code isBusy()} status and waiting for completion.</li>
     *     <li>Stopping the robot and returning the motors to {@link DcMotor.RunMode#RUN_USING_ENCODER} mode.</li>
     * </ol>
     *
     * @param targetPositions An array of four integers representing the target encoder ticks for each motor.
     *                        The order should be: [Front Left, Front Right, Rear Left, Rear Right].
     * @param targetSpeed     The desired motor power (from 0.0 to 1.0) to be applied during the movement.
     *                        This value is applied to all motors equally.
     * @return true if every motor reached its target; false if the move timed out or the OpMode
     *         was stopped. Callers that do not care can ignore the result.
     */
    public boolean driveRobotToPosition(int[] targetPositions, double targetSpeed) {
        return driveRobotToPosition(targetPositions, targetSpeed, defaultMoveTimeoutSec(targetPositions, targetSpeed));
    }

    /**
     * Same as {@link #driveRobotToPosition(int[], double)} but with an explicit time limit.
     * The move ends early, and the motors stop, if the limit passes or the OpMode is stopped.
     * Without this, a stalled wheel would keep the loop running after the Driver Station's Stop,
     * and the SDK's stuck-OpMode watchdog would restart the Robot Controller app.
     *
     * @param timeoutSec Maximum seconds to wait for the motors to reach their targets.
     */
    public boolean driveRobotToPosition(int[] targetPositions, double targetSpeed, double timeoutSec) {

        for (int i = 0; i < motors.size(); i++) {
            DcMotorEx motor = motors.get(i);
            if (motor != null) {
                motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(targetPositions[i]);
                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                motor.setPower(targetSpeed);
            }
        }

        // Wait for all motors to finish, or for the time limit, or for the OpMode to be stopped
        // (the SDK interrupts the OpMode thread on Stop).
        ElapsedTime moveTimer = new ElapsedTime();
        boolean reachedTarget = true;
        while (areMotorsBusy()) {
            if (Thread.currentThread().isInterrupted() || moveTimer.seconds() > timeoutSec) {
                reachedTarget = false;
                break;
            }
            sleep(MOVE_POLL_MS);   // give the hub's I2C and the DS a turn; no need to spin
        }

        // Stop the robot and reset run mode
        stopRobot();
        setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        return reachedTarget;
    }

    /**
     * A generous time limit for an encoder move: three times the ideal travel time at the requested
     * power, plus a margin, never less than MOVE_MIN_TIMEOUT_SEC. Meant to end a stalled step, not to
     * be tight. Pass an explicit timeout to the three-argument overload for something tighter.
     */
    private double defaultMoveTimeoutSec(int[] targetPositions, double targetSpeed) {
        int maxTicks = 0;
        for (int t : targetPositions) maxTicks = Math.max(maxTicks, Math.abs(t));
        double power = Math.max(Math.abs(targetSpeed), 0.05);
        double maxInches = maxTicks / ENCODER_COUNTS_PER_INCH;
        double idealSec = maxInches / (DRIVE_MAX_INCHES_PER_SEC * power);
        return Math.max(MOVE_MIN_TIMEOUT_SEC, idealSec * 3.0 + 2.0);
    }


    /**
     * Sets the power for all drive motors, ensuring the value is within the valid [-1.0, 1.0] range.
     * A positive power drives the robot forward, and a negative power drives it backward.
     *
     * @param power The desired power for the motors. It will be clamped to the range [-1.0, 1.0].
     */
    public void driveRobotForward(double power) {
        // Clamp the input power to the valid range of -1.0 to 1.0
        double clampedPower = Range.clip(power, -1.0, 1.0);
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setPower(clampedPower);
            }
        }
    }

    /**
     * Drives the robot sideways (strafes) at a given speed.
     *
     * This method controls a mecanum or X-drive drivetrain for strafing.
     *
     * @param speed The speed and direction for the robot to strafe.
     *              A positive value (e.g., 0.8) makes the robot strafe to the right.
     *              A negative value (e.g., -0.8) makes the robot strafe to the left.
     *              The expected range is [-1.0, 1.0].
     */
    public void driveRobotSideStrafe(double speed) {
        // Set the motor powers for a right strafe.
        // If 'speed' is negative, the powers will automatically be inverted,
        // resulting in a left strafe.
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(-speed);
        leftRearMotor.setPower(-speed);
        rightRearMotor.setPower(speed);
    }




    /**
     * Drives the robot a specified distance forward, sideways, and rotationally.
     * @param forwardInches  Distance to move forward (negative for backward).
     * @param strafeInches   Distance to move right (negative for left).
     * @param turnDegrees    Angle to rotate clockwise (negative for counter-clockwise).
     * @param speed          The motor speed (0.0 to 1.0).
     * @return true if the motors reached their targets; false if the move timed out or the OpMode
     *         was stopped (see driveRobotToPosition). The other encoder moves return the same flag.
     */
    public boolean drive_p3(double forwardInches, double strafeInches, double turnDegrees, double speed) {
        // The tick arithmetic is EncoderMoveMath.ticksFor (pure, unit-tested).
        int[] targetPositions = EncoderMoveMath.ticksFor(forwardInches, strafeInches, turnDegrees,
                ENCODER_COUNTS_PER_INCH, config.calibration.strafeScale, config.calibration.turnCircumferenceIn);
        return driveRobotToPosition(targetPositions, speed);
    }
    /**
     * The driveRobotDistance* family: the simplest commands, one direction each, distance in cm
     * or inches. All of them convert through calibration.encoderCountsPerInch, the same number
     * drive_p3 uses, so "12 inches" means the same thing everywhere.
     */
    public boolean driveRobotDistanceForward(double distanceInCM, double targetSpeed) {
        // (An earlier version passed a tick count, divided by 25.4, into drive_p3 as if it were
        // inches, which drove about 1.8x too far.)
        int targetCount = ticksForCm(distanceInCM, 1.0);
        int[] targetPositions = {targetCount, targetCount, targetCount, targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    /** Ticks for a distance in cm, times a scale (1.0, or calibration.strafeScale for strafes). */
    private int ticksForCm(double distanceInCM, double scale) {
        return (int) Math.round((distanceInCM / 2.54) * ENCODER_COUNTS_PER_INCH * scale);
    }

    public boolean driveRobotDistanceForwardInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceForward(distanceInCM, targetSpeed);
    }

    public boolean driveRobotDistanceBackward(double distanceInCM, double targetSpeed) {
        int targetCount = ticksForCm(distanceInCM, 1.0);
        int[] targetPositions = {-targetCount, -targetCount, -targetCount, -targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    public boolean driveRobotDistanceBackwardInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceBackward(distanceInCM, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeRight(double distanceInCM, double targetSpeed) {
        int targetCount = ticksForCm(distanceInCM, config.calibration.strafeScale);
        int[] targetPositions = {targetCount, -targetCount, -targetCount, targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeRightInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceStrafeRight(distanceInCM, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeLeft(double distanceInCM, double targetSpeed) {
        int targetCount = ticksForCm(distanceInCM, config.calibration.strafeScale);
        int[] targetPositions = {-targetCount, targetCount, targetCount, -targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeLeftInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceStrafeLeft(distanceInCM, targetSpeed);
    }

    /**
     * Spin in place. Positive is clockwise, the same as drive_p3's turn argument, and it uses the
     * same calibration number (turnCircumferenceIn), so "turn 90" means the same thing here as in
     * drive_p3(0, 0, 90, speed). (Until 2026-09-07 this used a separate turning-circle diameter
     * that disagreed with drive_p3's by a factor of 2.7.)
     */
    public boolean rotateRobot(double angleInDegrees, double targetSpeed) {
        return drive_p3(0, 0, angleInDegrees, targetSpeed);
    }



    // =================================================================================
    // SECTION 5: COMPLEX AUTONOMOUS DRIVE METHODS (USING PID)
    // =================================================================================

    public void update() {
        if (pinpoint != null) pinpoint.update();
        switch (driveState) {
            case DRIVING_TO_POINT_PINPOINT:
                // Step the waypoint drive (startDriveTo). driveTo does one loop of PID and moves.
                if (asyncTimer.seconds() > asyncTimeoutSec) {
                    stopRobot();
                    lastMoveSucceeded = false;
                    driveState = DriveState.IDLE;
                } else if (driveTo(pinpoint.getPosition(), asyncTarget, asyncPower, asyncHoldTimeSec)) {
                    stopRobot();
                    lastMoveSucceeded = true;
                    driveState = DriveState.IDLE;
                }
                break;
            case ALIGNING_TO_APRILTAG:
                // Step the tag approach; it reads the sighting and hands back three powers.
                if (tagApproach.update(tagSighting)) {
                    stopRobot();
                    lastMoveSucceeded = tagApproach.succeeded();
                    driveState = DriveState.IDLE;   // finished: DONE, LOST or TIMED_OUT (see lastTagApproachSucceeded)
                } else {
                    moveRobot(tagApproach.getDrivePower(), tagApproach.getStrafePower(), tagApproach.getYawPower());
                }
                break;
            case IDLE:
            default:
                // Do nothing
                break;
        }
//        if (follower != null) {
//            follower.update();
//        }
    }

public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
    boolean atTarget;

    // Check if we're at target FIRST
    if (inBounds(currentPosition, targetPosition)) {
        // We're at target - STOP ALL MOTORS
        moveRobot(0, 0, 0);
        atTarget = true;

        // Check if we've held position long enough
        if(GBholdTimer.time() > holdTime){
            return true;  // Done!
        }
        return false;  // Still holding

    } else {
        // Not at target yet - calculate PID and move
        GBholdTimer.reset();
        atTarget = false;

        double xPWR = calculatePID(currentPosition, targetPosition, Direction.x);
        double yPWR = calculatePID(currentPosition, targetPosition, Direction.y);
        double hOutput = calculatePID(currentPosition, targetPosition, Direction.h);

        // The PID outputs are in the field frame (x forward, y left); rotate them into the
        // robot's frame. Heading PID output is counter-clockwise positive; moveRobot's yaw is
        // clockwise positive, hence the minus.
        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        MecanumMixer.Command c = MecanumMixer.fieldToRobot(xPWR, yPWR, heading);
        moveRobot(c.drive * power, c.strafeRight * power, -(hOutput * power));

        return false;  // Still driving
    }
}


    private double calculatePID(Pose2D currentPosition, Pose2D targetPosition, Direction direction){
        if(direction ==Direction.x){
            double xError = targetPosition.getX(MM) - currentPosition.getX(MM);
            // ADD THIS: Early return if within tolerance (same as yaw has)
            if (Math.abs(xError) < config.pointToPointTuning.xyTolerance) {
                xPID.pidReset();
                return 0.0;
            }
            return xPID.calculateAxisPID(xError,
                    config.pointToPointTuning.pGain,
                    config.pointToPointTuning.iGain,  // ADDED THIS
                    config.pointToPointTuning.dGain,
                    config.pointToPointTuning.accel,
                    PIDTimer.seconds(),
                    config.pointToPointTuning.xyTolerance);
        }
        if(direction == Direction.y){
            double yError = targetPosition.getY(MM) - currentPosition.getY(MM);
            // ADD THIS: Early return if within tolerance (same as yaw has)
            if (Math.abs(yError) < config.pointToPointTuning.xyTolerance) {
                yPID.pidReset();
                return 0.0;
            }
            return yPID.calculateAxisPID(yError,
                    config.pointToPointTuning.pGain,
                    config.pointToPointTuning.iGain,  // ADDED THIS
                    config.pointToPointTuning.dGain,
                    config.pointToPointTuning.accel,
                    PIDTimer.seconds(),
                    config.pointToPointTuning.xyTolerance);
        }
        if(direction == Direction.h){
            double targetH = targetPosition.getHeading(AngleUnit.RADIANS);
            double currentH = currentPosition.getHeading(AngleUnit.RADIANS);
            double hError = Angle.normDelta(targetH - currentH);
            //double hError = targetPosition.getHeading(AngleUnit.RADIANS) - currentPosition.getHeading(AngleUnit.RADIANS);
            if (Math.abs(hError) < config.pointToPointTuning.yawTolerance) {
                hPID.pidReset();
                return 0.0;
            }
            return hPID.calculateAxisPID(hError,
                    config.pointToPointTuning.yawPGain,
                    config.pointToPointTuning.yawIGain,  // ADD THIS (you'll need to add this to your tuning config)
                    config.pointToPointTuning.yawDGain,
                    config.pointToPointTuning.yawAccel,
                    PIDTimer.seconds(),
                    config.pointToPointTuning.yawTolerance);
        }
        return 0;
    }

    private boolean inBounds(Pose2D currPose, Pose2D trgtPose){
        boolean xInBounds = currPose.getX(MM) > (trgtPose.getX(MM) - config.pointToPointTuning.xyTolerance) && currPose.getX(MM) < (trgtPose.getX(MM) + config.pointToPointTuning.xyTolerance);
        boolean yInBounds = currPose.getY(MM) > (trgtPose.getY(MM) - config.pointToPointTuning.xyTolerance) && currPose.getY(MM) < (trgtPose.getY(MM) + config.pointToPointTuning.xyTolerance);
        double targetH = trgtPose.getHeading(RADIANS);
        double currentH = currPose.getHeading(RADIANS);
        double hError = Angle.normDelta(targetH - currentH);
        boolean hInBounds = Math.abs(hError) < config.pointToPointTuning.yawTolerance;
        //boolean hInBounds = currPose.getHeading(RADIANS) > (trgtPose.getHeading(RADIANS) - config.yawTolerance) &&
        //        currPose.getHeading(RADIANS) < (trgtPose.getHeading(RADIANS) + config.yawTolerance);

        return xInBounds && yInBounds && hInBounds;
    }

    public double calculateTargetHeading(Pose2D currPose, Pose2D trgtPose){
        double xDelta = trgtPose.getX(MM) - currPose.getX(MM);
        double yDelta = trgtPose.getY(MM) - currPose.getY(MM);

        if(Math.abs(xDelta) > config.pointToPointTuning.xyTolerance || Math.abs(yDelta) > config.pointToPointTuning.xyTolerance){
            return Math.atan2(yDelta, xDelta);
        } else {
            return currPose.getHeading(RADIANS);
        }

    }

    public double distanceTo(Pose2D currPose, Pose2D trgtPose,DistanceUnit distanceUnit) {
        double dx = trgtPose.getX(distanceUnit) - currPose.getX(distanceUnit);
        double dy = trgtPose.getY(distanceUnit) - currPose.getY(distanceUnit);
        return Math.hypot(dx, dy);
    }
    public double bearingTo(Pose2D currPose, Pose2D trgtPose) {
        double dx =trgtPose.getX(MM) - currPose.getX(MM);
        double dy = trgtPose.getY(MM) - currPose.getY(MM);
        return Math.atan2(dy, dx); // radians, world-frame
    }

    public double headingError(Pose2D currPose, Pose2D tgt) {
        return Angle.normDelta(tgt.getHeading(RADIANS) - currPose.getHeading(RADIANS));
    }

    public Vec2 robotFrameError(Pose2D currPose, Pose2D tgt) {
        double dx = tgt.getX(MM) - currPose.getX(MM);
        double dy = tgt.getY(MM) - currPose.getY(MM);
        double th = currPose.getHeading(RADIANS);
        double ex =  Math.cos(-th)*dx - Math.sin(-th)*dy;
        double ey =  Math.sin(-th)*dx + Math.cos(-th)*dy;
        return new Vec2(ex, ey);
    }
    public static  class Angle {
        /**
         * Wraps an angle difference into the range -PI to +PI.
         * Example: normDelta(3.5π) → -0.5π
         */
        public static double normDelta(double radians) {
            while (radians > Math.PI)  radians -= 2 * Math.PI;
            while (radians < -Math.PI) radians += 2 * Math.PI;
            return radians;
        }
    }
    public static class Vec2 {
        public final double x;
        public final double y;

        public Vec2(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double magnitude() { return Math.hypot(x, y); }
        public double angle()     { return Math.atan2(y, x); }
        public String toString()  { return String.format("(%.2f, %.2f)", x, y); }
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted (which usually
     * indicates that the OpMode has been stopped).
     * <p>This is simple shorthand for {@link Thread#sleep(long) sleep()}, but it does not throw {@link InterruptedException}.</p>
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /************ GRAND EXPERIMENTS ******
     * experimental code to use at your own risk :-)
     * good luck
     *
     */

    private boolean relActive = false;
    private Pose2D relTarget = null;
    public boolean driveRelative(Pose2D currentPosition,double driveInches, double strafeInches, double turnDegrees, double power, double holdTime) {
        // 1) create target once:
        if (!relActive) {

            double startX = currentPosition.getX(DistanceUnit.INCH);
            double startY = currentPosition.getY(DistanceUnit.INCH);
            double startHeadingRad = currentPosition.getHeading(AngleUnit.RADIANS);

            // --- 2. Calculate the Target Position ---
            // We need to rotate the relative drive/strafe commands by the robot's current heading
            // to find the change in world coordinates (deltaX, deltaY).
            double deltaX = driveInches * Math.cos(startHeadingRad) - strafeInches * Math.sin(startHeadingRad);
            double deltaY = driveInches * Math.sin(startHeadingRad) + strafeInches * Math.cos(startHeadingRad);

            // The new absolute target coordinates are the starting position plus the calculated deltas.
            double targetX = startX + deltaX;
            double targetY = startY + deltaY;
            double targetHeadingDeg = AngleUnit.normalizeDegrees(currentPosition.getHeading(AngleUnit.DEGREES) + turnDegrees);

            // --- 4. Create the Target Pose2D ---
            // Create the final absolute target pose that our driveTo() method can understand.
            // The Pose2D constructor uses base units: inches and radians.
            relTarget = new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.DEGREES, targetHeadingDeg);

            relActive = true;

            telemetry.addData("DriveRelative", "Start: (%.1f, %.1f) H: %.1f", startX, startY, currentPosition.getHeading(AngleUnit.DEGREES));
            telemetry.addData("DriveRelative", "Target: (%.1f, %.1f) H: %.1f", targetX, targetY, targetHeadingDeg);
            telemetry.update();
        }

        // --- 5. Execute the Movement by Calling the Existing driveTo() Method ---
        // This reuses all your existing, tested PID logic!
        boolean atTarget =  driveTo(currentPosition, relTarget, power, holdTime);

        if(atTarget) {
            relActive = false;
            relTarget = null;
        }
        return atTarget;
    }
    /**
     * Start driving to a spot in front of an AprilTag without blocking. Call once; then call
     * update() every loop (the robot classes already do) and poll isBusy(). When isBusy() goes
     * false, lastTagApproachSucceeded() says whether the robot got there or gave up (tag lost,
     * or the time limit). Other subsystems keep running in the same loop.
     *
     * The math is common/TagApproach; the gains are config.tagApproach. The sighting is normally
     * the robot's VisionUtil, which implements TagSighting.
     *
     * @param sighting       where the tag is, in the robot's frame (VisionUtil)
     * @param tagId          AprilTag id to approach
     * @param standoffInches how far in front of the tag's face to stop
     * @param holdTimeSec    how long to sit inside tolerance before reporting done
     */
    public void driveToTagAsync(TagSighting sighting, int tagId, double standoffInches, double holdTimeSec) {
        cancel();
        this.tagSighting = sighting;
        tagApproach.start(tagId, standoffInches, holdTimeSec);
        this.driveState = DriveState.ALIGNING_TO_APRILTAG;
    }

    /** Abandon a driveToTagAsync in progress and stop the wheels. Same as cancel(). */
    public void cancelDriveToTag() {
        cancel();
    }

    /** True if the most recent driveToTagAsync ended in DONE rather than LOST or TIMED_OUT. */
    public boolean lastTagApproachSucceeded() {
        return tagApproach.succeeded();
    }

    /** The approach controller, for telemetry (state, errors, powers). */
    public TagApproach getTagApproach() {
        return tagApproach;
    }
    /**
     * Calculates the turn power required to automatically aim the robot at a target.
     * This uses a simple Proportional (P) controller based on heading error.
     *
     * @param headingErrorDegrees The heading error from the vision system in degrees.
     * @return The calculated turn power, from -1.0 to 1.0.
     */
    public double calculateAutoAimTurn(double headingErrorDegrees) {
        // These constants could be moved to your RobotConfig if they differ between robots
        final double kP_TURN = 0.03;  // Proportional gain for turning
        final double MIN_TURN_POWER = 0.05; // Minimum power to overcome friction
        final double HEADING_TOLERANCE_DEG = 1.0; // Deadband to prevent "buzzing"

        // Deadband: If we are close enough, don't apply any power.
        if (Math.abs(headingErrorDegrees) < HEADING_TOLERANCE_DEG) {
            return 0.0;
        }

        // Calculate the raw turn command using the P-controller.
        // The negative sign ensures a positive error (tag is to the right) causes a positive (clockwise) turn.
        double turnCmd = -kP_TURN * headingErrorDegrees;

        // Apply a minimum power if the robot needs to move but the command is too small.
        if (Math.abs(turnCmd) < MIN_TURN_POWER) {
            turnCmd = Math.copySign(MIN_TURN_POWER, turnCmd);
        }

        // Clamp the output to the valid motor power range [-1.0, 1.0].
        return Range.clip(turnCmd, -1.0, 1.0);
    }


//****************************************************************************************************
//****************************************************************************************************

    // =================================================================================
    // SECTION 6: INNER CLASSES
    // =================================================================================

    // PinpointPIDLoop (the per-axis PID behind driveTo) moved to its own file, common/PinpointPIDLoop,
    // on 2026-09-07 so it could be unit-tested. Nothing about it changed.
}
