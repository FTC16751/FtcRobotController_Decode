
package org.firstinspires.ftc.teamcode.utilities.Common;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static java.lang.Math.toDegrees;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil;

import java.util.Arrays;
import java.util.List;


public class DriveUtilDepricated {
    // Robot Constants
    private static double ROBOT_SIZE_DIAMETER = 60; //in cm
    private static final double ENCODER_COUNTS_PER_INCH = 45.33;
    private static final double ENCODER_RESOLUTION = 537;
    private static final double WHEEL_DIAMETER_CM = 9.6;
    private static final double WHEEL_RADIUS = WHEEL_DIAMETER_CM / 2;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_CM * Math.PI;
    private static final double GEAR_REDUCTION = 1.0;
    private static final double TRACK_WIDTH = 17.5;
    private static final double COUNTS_PER_GEAR_REV = ENCODER_RESOLUTION * GEAR_REDUCTION;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV / 360;

    // Declare motor variables
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    private List<DcMotor> motors;

    public static final String FRONT_LEFT_MOTOR_NAME = "Front_Left";
    public static final String FRONT_RIGHT_MOTOR_NAME = "Front_Right";
    public static final String REAR_LEFT_MOTOR_NAME = "Rear_Left";
    public static final String REAR_RIGHT_MOTOR_NAME = "Rear_Right";
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private Telemetry telemetry;
    public GoBildaPinpointDriver pinpoint; // Declare OpMode member for the Odometry Computer
    private DistanceSensor sensorDistance;

//    public enum Direction {
//        FORWARD,
//        BACKWARD,
//        LEFT,
//        RIGHT
//    }

    private static final double SPEED_GAIN = 0.15;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    private static final double STRAFE_GAIN = 0.03;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    private static final double TURN_GAIN = 0.03;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    private static final double MAX_AUTO_SPEED = 0.80;   //  Clip the approach speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_STRAFE = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_TURN = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)

    // Adjust these numbers to suit your robot.
    private static final double COUNTS_PER_REV            = 384.5;   //  GoBilda 435 RPM motor (13.7:1 gear)
    private static final double  WHEEL_DIAMETER_IN         = 3.75;    //  GoBilda 96mm Mecanum Wheel

    private final double  AXIAL_INCHES_PER_COUNT    = (Math.PI * WHEEL_DIAMETER_IN) / COUNTS_PER_REV;
    private final double  LATERAL_INCHES_PER_COUNT  = AXIAL_INCHES_PER_COUNT * 0.866;

    // === NEW CONSTANTS FOR ENCODER BASED AUTONOMOUS USING PID ===
    private static final double DRIVE_GAIN          = 0.085;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

   // private static final double STRAFE_GAIN         = 0.03;    // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.018;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public DriveUtilProportionalControldepricated driveController     = new DriveUtilProportionalControldepricated(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public DriveUtilProportionalControldepricated strafeController    = new DriveUtilProportionalControldepricated(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public DriveUtilProportionalControldepricated yawController       = new DriveUtilProportionalControldepricated(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // Hardware interface Objects
    private int encoderLF;              // Encoder value for front left wheel
    private int encoderRF;              // Encoder value for front right wheel
    private int encoderLB;              // Encoder value for back left wheel
    private int encoderRB;              // Encoder value for back right wheel
    private int startLeftFront = 0;
    private int startRightFront = 0;
    private int startLeftBack = 0;
    private int startRightBack = 0;
    private int deltaLeftFront = 0;
    private int deltaRightFront = 0;
    private int deltaLeftBack = 0;
    private int deltaRightBack = 0;

    private double rawHeading       = 0; // Unmodified heading (degrees)
    private double headingOffset    = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU (deg / sec)
    private boolean showTelemetry     = true;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.
    private OpMode myOpMode;
    // === END OF NEW CONSTANTS FOR ENCODER BASED AUTONOMOUS USING PID ===


    // === NEW CONSTANTS FOR APRILTAG ALIGNMENT ===
    // These gains control how aggressively the robot corrects its position.
    // Start with small values like these and tune them for your robot's weight and speed.
    private static final double TAG_RANGE_GAIN      = 0.02;  // Forward/backward power per inch of error
    private static final double TAG_STRAFE_GAIN     = 0.04;  // Strafe power per inch of sideways error
    private static final double TAG_TURN_GAIN       = 0.03;  // Turn power per degree of heading error

    // Define the tolerance for when the robot is considered "aligned".
    private static final double TAG_RANGE_TOLERANCE_IN   = 0.5;  // Inches
    private static final double TAG_STRAFE_TOLERANCE_IN  = 0.5;  // Inches
    private static final double TAG_TURN_TOLERANCE_DEG   = 2.0;  // Degrees
    // Timer to track how long the target has been lost
    private final ElapsedTime targetLostTimer = new ElapsedTime();

    // Timeout in seconds. If the tag is lost for longer than this, the action fails.
    private static final double TARGET_LOST_TIMEOUT_SEC = 1.5;

    // Variables to store the last known good motor powers when the tag was visible
    private double lastGoodDrivePower = 0;
    private double lastGoodStrafePower = 0;
    private double lastGoodTurnPower = 0;

    // === UPDATE DriveState ENUM ===
    private enum DriveState {
        IDLE,
        DRIVING_TO_ENCODER_TARGET,  // Your existing state for encoder driving
        ALIGNING_TO_APRILTAG        // New state for auto-alignment
    }
    private DriveState driveState = DriveState.IDLE;

    // Add new member variables to store the alignment target
    private VisionUtil vision;
    private int targetTagId;
    private double desiredTagDistanceInches;
    private double holdTime;

    /************************
     * GBPINPOINT STUFF
     *
     */
    public enum DriveType {
        MECANUM,
        TANK
    }

    public enum DriveMotor{
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    }

    private enum Direction {
        x,
        y,
        h
    }

    private enum InBounds {
        NOT_IN_BOUNDS,
        IN_X_Y,
        IN_HEADING,
        IN_BOUNDS
    }

    private static double xyTolerance = 8;
    private static double yawTolerance = 0.0349066;

    private static double pGain = 0.01705;
    private static double dGain = 0.00111;
    private static double accel = 8.0;

    private static double yawPGain = 5.0;
    private static double yawDGain = 0.0;
    private static double yawAccel = 20.0;

    private double leftFrontMotorOutput  = 0;
    private double rightFrontMotorOutput = 0;
    private double leftBackMotorOutput   = 0;
    private double rightBackMotorOutput  = 0;

    private final ElapsedTime GBholdTimer = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();

    private LinearOpMode myLinearOpMode; //todo: consider if this is required

    private final PIDLoopDepricated xPID = new PIDLoopDepricated();
    private final PIDLoopDepricated yPID = new PIDLoopDepricated();
    private final PIDLoopDepricated hPID = new PIDLoopDepricated();

    private final PIDLoopDepricated xTankPID = new PIDLoopDepricated();

    private DriveType selectedDriveType = DriveType.MECANUM;


    // P3-specific members (will only be initialized if it's the P3 bot)
    private Follower pedroFollower;
    // limelight camera stuff
    private Limelight3A limelight; //any camera here

    private final KalmanFilterParameters kfParams = new KalmanFilterParameters(6,1); // todo: tune?

    private final KalmanFilter xFilter =  new KalmanFilter(kfParams);
    private final KalmanFilter yFilter = new KalmanFilter(kfParams);
    private final KalmanFilter thetaFilter = new KalmanFilter(kfParams);
    public DriveUtilDepricated(HardwareMap hardwareMap, Telemetry telemetry, OpMode opMode) {
        myOpMode = opMode;

        this.telemetry = telemetry;
        initializeIMU(hardwareMap);
        initOdo(hardwareMap);
        initMotors(hardwareMap);
        runtime = new ElapsedTime();

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private void initMotors(HardwareMap hardwareMap) {
        // Initialize each motor using the constants
        leftFrontMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
        rightFrontMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
        leftRearMotor = hardwareMap.get(DcMotor.class, REAR_LEFT_MOTOR_NAME);
        rightRearMotor = hardwareMap.get(DcMotor.class, REAR_RIGHT_MOTOR_NAME);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Gear Girls Robot
//        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // Group motors into a list for easy bulk operations
        motors = Arrays.asList(leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor);

        // Set the zero power behavior for all motors at once
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // zero out all the odometry readings.
        startMotion();
        //imu.resetYaw();
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
    }

    private void initOdo(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();
    }
    private void configurePinpoint()
    {
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-0.0, -150.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        //pinpoint.resetPosAndIMU();
    }

    /**
     * Gets the current power level of a specific motor.
     * The power level is a value between -1.0 (full power reverse) and 1.0 (full power forward),
     * with 0.0 representing a stopped motor.
     *
     * @param motor The motor object (e.g., leftFrontMotor) for which to retrieve the power.
     * @return The current power of the motor, a value in the range [-1.0, 1.0].
     */
    public double getmotorPower(DcMotor motor) {
        /* tested */
        return motor.getPower();
    }


    /**
     * Retrieves the current encoder position of a specified motor.
     * The position is returned in encoder ticks.
     *
     * @param motor The {@link DcMotor} for which to get the current position.
     * @return The current position of the motor in encoder ticks.
     */
    public double getmotorPosition(DcMotor motor) {
        /* tested */
        return motor.getCurrentPosition();
    }

    // Helper method for logging
    private void logMethodCall() {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3]; // Adjust this number if needed
        RobotLog.dd("GAMLOG", "current method: " + name + ": called from: " + e);
    }

    /**
     * Sets the same power level for all drive motors.
     * This is a convenience method to apply a uniform power setting across the entire drivetrain.
     * Note: This method does not clamp the power value; it is the caller's responsibility
     * to ensure the provided power is within the valid range of [-1.0, 1.0].
     *
     * @param power The power level to apply to all motors. Positive values typically
     *              drive the robot forward, while negative values drive it backward.
     */
    public void setMotorPower(double power) {
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setPower(power);
            }
        }
    }

    /**
     * Sets the same power level for all four drive motors.
     * This is a convenience method that calls {@link #setMotorPowers(double, double, double, double)}
     * with the same power value for each motor.
     *
     * @param p The desired power level for all motors, from -1.0 (full reverse) to 1.0 (full forward).
     */ //Set power to all motors
    public void setAllPower(double p) {
        setMotorPowers(p, p, p, p);
    }

    /**
     * Immediately stops all drive motors by setting their power to zero.
     * This method iterates through the list of motors and sets each one's power to 0.0,
     * effectively halting any robot movement. It's a fundamental safety and control method.
     */ // Stop all motors
    public void stopMotors() {
        // Stop all motors by setting their power to 0.0
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setPower(0.0);
            }
        }
    }
    /**
     * Sets motor powers from a list or array of doubles.
     * The order is assumed to be: Left Front, Left Rear, Right Rear, Right Front.
     *
     * @param powers A list or array containing the four motor powers.
     * @throws IllegalArgumentException if the input list does not contain exactly 4 values.
     */
    public void setMotorPowers(List<Double> powers) {
         setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }


    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightRearMotor.setPower(rightRearPower);
        rightFrontMotor.setPower(rightFrontPower);
    }


    // Set the motor mode for all motors
    public void setMotorMode(DcMotor.RunMode mode) {
        // Set the specified mode for all motors
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setMode(mode);
            }
        }
    }

    public void resetEncoders() {
        // Stop and reset encoders for all motors
        for (DcMotor motor : motors) {
            if (motor != null) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // Set all motors to RUN_USING_ENCODER mode
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Sets the {@link DcMotor.RunMode} for all motors in the drivetrain.
     * <p>
     * This is a convenience method to apply a specific run mode, such as
     * {@code RUN_USING_ENCODER} or {@code RUN_TO_POSITION}, to all four drive motors
     * simultaneously. It iterates through the list of motors and sets the mode for each one.
     *
     * @param runMode The {@code DcMotor.RunMode} to set for all drive motors.
     */ // Helper method to set run mode for all motors
    private void setMotorRunMode(DcMotor.RunMode runMode) {
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setMode(runMode);
            }
        }
    }

    /**
     * Checks if any of the drive motors are currently busy.
     * <p>
     * This method iterates through the list of motors and returns {@code true}
     * as soon as it finds a motor that is not null and its {@link DcMotor#isBusy()}
     * method returns true. This is typically used to wait for the completion of
     * an encoder-based movement (e.g., {@code RUN_TO_POSITION}).
     *
     * @return {@code true} if at least one motor is busy, {@code false} otherwise.
     */
    private boolean areMotorsBusy() {
        for (DcMotor motor : motors) {
            if (motor != null && motor.isBusy()) {
                return true;
            }
        }
        return false;
    }


    /**
     * Resets the robot's yaw (heading) angle to zero.
     * <p>
     * This method is a wrapper around the IMU's {@code resetYaw()} function. It tells the IMU
     * that the robot's current orientation should be considered the new "zero" heading.
     * This is useful at the beginning of an autonomous period or a match to establish a
     * consistent frame of reference for all subsequent turns and field-centric movements.
     */
    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        } else {
            // Log an error to the Driver Station or Logcat for debugging
           telemetry.addData("RobotHardware", "IMU is not initialized. Cannot reset yaw.");
        }
    }

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

     /**
     * Calculates the number of encoder counts per degree of chassis rotation.
     *
     * @param wheelDiameter The diameter of the wheels, in inches.
     * @param encoderTicksPerRevolution The number of ticks the encoder registers for one full motor revolution.
     * @param gearRatio The ratio of motor revolutions to wheel revolutions (e.g., a 2:1 ratio means this value is 2.0).
     * @param trackWidth The distance between the center of the left and right wheels, in inches.
     * @return The number of encoder counts required to rotate the chassis by one degree.
     */
    private double calculateEncoderCountsPerDegreeOfChassisRotation(
            double wheelDiameter,
            int encoderTicksPerRevolution,
            double gearRatio,
            double trackWidth) {

        // 1. Calculate the circumference of the wheel.
        final double wheelCircumference = Math.PI * wheelDiameter;

        // 2. Calculate the number of encoder ticks for one full wheel revolution.
        final double ticksPerWheelRevolution = encoderTicksPerRevolution * gearRatio;

        // 3. Calculate the distance the wheel travels per single encoder tick.
        final double distancePerTick = wheelCircumference / ticksPerWheelRevolution;

        // 4. Calculate the circumference of the circle the robot travels during a 360-degree turn.
        // This assumes the robot pivots around its center point.
        final double chassisTurnCircumference = Math.PI * trackWidth;

        // 5. Calculate the total number of encoder ticks needed for a full 360-degree chassis turn.
        final double totalTicksFor360Turn = chassisTurnCircumference / distancePerTick;

        // 6. Calculate the number of ticks per degree of chassis rotation.
        return totalTicksFor360Turn / 360.0;
    }

    /**
     * Normalizes an angle to be within the range of -180 to +180 degrees.
     * This is useful for processing heading or bearing values to ensure consistency
     * and prevent issues with angle wrapping (e.g., 359 degrees vs -1 degree).
     *
     * @param angle The angle in degrees to normalize.
     * @return The normalized angle, which will be between -180 (exclusive) and +180 (inclusive).
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    ///      /**********************************************************
    ///      ********** ESSENTIAL MECHANUM ROBOT METHODS ****************
    ///      *******************************************************************/
    ///
    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {
        // Read motor encoders for each wheel
        encoderLF =         leftFrontMotor.getCurrentPosition();
        encoderRF =         rightFrontMotor.getCurrentPosition();
        encoderLB =         leftRearMotor.getCurrentPosition();
        encoderRB =         rightRearMotor.getCurrentPosition();

        updateMotion();  // determine how robot has moved from most recent startMotion() call;

        // read the IMU data.
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        heading     = rawHeading - headingOffset;
        turnRate    = angularVelocity.zRotationRate;

        if (showTelemetry) {
            telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    // Initialize all the encoder starting values for the next motion
    public void startMotion() {
        readSensors();  // get the latest data
        startLeftBack = encoderLB;  // Save the current values as the start values.
        startLeftFront = encoderLF;
        startRightBack = encoderRB;
        startRightFront = encoderRF;
        updateMotion();  // Update the derived motion data.
    }

    public void updateMotion() {
        deltaLeftFront = encoderLF - startLeftFront;
        deltaRightFront = encoderRF - startRightFront;
        deltaLeftBack = encoderLB - startLeftBack;
        deltaRightBack = encoderRB - startRightBack;

        driveDistance  = ((deltaLeftFront + deltaRightFront + deltaLeftBack + deltaRightBack ) / 4) * AXIAL_INCHES_PER_COUNT;
        strafeDistance = ((-deltaLeftFront + deltaRightFront + deltaLeftBack - deltaRightBack) / 4) * LATERAL_INCHES_PER_COUNT;
    }

    ///      /**********************************************************
    ///      ********** ROBOT DRIVING METHODS ****************
    ///      *******************************************************************/
    ///

    public void stopRobot_p3() {
        stopMotors();
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
        for (DcMotor motor : motors) {
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
     */
    public void driveRobotToPosition(int[] targetPositions, double targetSpeed) {

        for (int i = 0; i < motors.size(); i++) {
            DcMotor motor = motors.get(i);
            if (motor != null) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(targetPositions[i]);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(targetSpeed);
            }
        }

        // Wait for all motors to finish
        while (areMotorsBusy()) {
            // Optional: Add telemetry here to monitor motor positions.
            // Loop remains active while motors are running to their targets.
        }

        // Stop the robot and reset run mode
        stopRobot_p3();
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Drives the robot a specified distance forward, sideways, and rotationally.
     * @param forwardInches  Distance to move forward (negative for backward).
     * @param strafeInches   Distance to move right (negative for left).
     * @param turnDegrees    Angle to rotate clockwise (negative for counter-clockwise).
     * @param speed          The motor speed (0.0 to 1.0).
     */
    public void drive_p3(double forwardInches, double strafeInches, double turnDegrees, double speed) {
        int forwardTicks = (int) (forwardInches * ENCODER_COUNTS_PER_INCH);
        int strafeTicks = (int) (strafeInches * ENCODER_COUNTS_PER_INCH * 1.1); // Strafe fudge factor

        // Calculate turn ticks based on robot's track width
        // 1. Calculate the circumference of the circle the ROBOT ITSELF makes during a full turn.
        //    The diameter of this circle is the robot's track width.
        //    **Crucially, TRACK_WIDTH must be in INCHES to match ENCODER_COUNTS_PER_INCH
        double turnCircumference = Math.PI * 27.5; //ROBOT_SIZE_DIAMETER;
        double turnDistanceInches = (turnDegrees / 360.0) * turnCircumference;
        int turnTicks = (int) (turnDistanceInches * ENCODER_COUNTS_PER_INCH);

        int fl_ticks = forwardTicks + strafeTicks + turnTicks;
        int fr_ticks = forwardTicks - strafeTicks - turnTicks;
        int rl_ticks = forwardTicks - strafeTicks + turnTicks;
        int rr_ticks = forwardTicks + strafeTicks - turnTicks;

        int[] targetPositions = {fl_ticks, fr_ticks, rl_ticks, rr_ticks};

        driveRobotToPosition(targetPositions, speed);
    }
    public void driveRobotDistanceForward(double distanceInCM, double targetSpeed) {
        // ... (logging and unit conversion)
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);
        //int[] targetPositions = {targetCount, targetCount, targetCount, targetCount};
        //driveRobotToPosition(targetPositions, targetSpeed);
        drive_p3(targetCount/25.4, 0, 0, targetSpeed);
    }

    public void driveRobotDistanceForwardInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        driveRobotDistanceForward(distanceInCM, targetSpeed);
    }

    public void driveRobotDistanceBackward(double distanceInCM, double targetSpeed) {
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {-targetCount, -targetCount, -targetCount, -targetCount};
        driveRobotToPosition(targetPositions, targetSpeed);
    }

    public void driveRobotDistanceBackwardInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        driveRobotDistanceBackward(distanceInCM, targetSpeed);
    }

    public void driveRobotDistanceStrafeRight(double distanceInCM, double targetSpeed) {
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV * 1.1 / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {targetCount, -targetCount, -targetCount, targetCount};
        driveRobotToPosition(targetPositions, targetSpeed);
    }

    public void driveRobotDistanceStrafeRightInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        driveRobotDistanceStrafeRight(distanceInCM, targetSpeed);
    }

    public void driveRobotDistanceStrafeLeft(double distanceInCM, double targetSpeed) {
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV * 1.1 / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {-targetCount, targetCount, targetCount, -targetCount};
        driveRobotToPosition(targetPositions, targetSpeed);
    }

    public void driveRobotDistanceStrafeLeftInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        driveRobotDistanceStrafeLeft(distanceInCM, targetSpeed);
    }

    public void rotateRobot(double angleInDegrees, double targetSpeed) {
        //rotate(90, 0.5);
        // Calculate the target count based on the angle and robot diameter
        double circumference = Math.PI * ROBOT_SIZE_DIAMETER;
        double distanceToTravel = (Math.abs(angleInDegrees) / 360.0) * circumference;
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceToTravel);

        // Determine the direction of rotation (clockwise or counterclockwise)
        int direction = angleInDegrees > 0 ? 1 : -1;

        // Set target positions for each motor
        int[] targetPositions = {direction * targetCount, -direction * targetCount, direction * targetCount, -direction * targetCount};

        // Call the helper method to execute the turn
        driveRobotToPosition(targetPositions, targetSpeed);
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

    public void tankDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
        // Use a separate turning speed multiplier for finer control
        final double TURN_SENSITIVITY = 0.8;

        // Negate moveValue as joysticks are often inverted.
        double drive = -left_stick_x * DRIVE_SPEED;
        double turn = right_stick_y * DRIVE_SPEED * TURN_SENSITIVITY;

        // Combine drive and turn for blended motion.
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize the values to ensure they do not exceed +/- 1.0
        double maxMagnitude = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxMagnitude > 1.0) {
            leftPower /= maxMagnitude;
            rightPower /= maxMagnitude;
        }

        // Send calculated power to wheels
        setMotorPowers(leftPower, leftPower, rightPower, rightPower);
    }

    public void arcadeDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
        double drive = -left_stick_y * DRIVE_SPEED; // Remember, this is reversed!
        double strafe = left_stick_x * 1.1 * DRIVE_SPEED; // Counteract imperfect strafing
        double yaw = right_stick_x * DRIVE_SPEED;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(yaw), 1);
        double frontLeftPower = (drive + strafe + yaw) / denominator;
        double backLeftPower = (drive - strafe + yaw) / denominator;
        double frontRightPower = (drive - strafe - yaw) / denominator;
        double backRightPower = (drive + strafe - yaw) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }



    /**
     * Move robot to a designated X,Y position and heading
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */
    public boolean pathComplete = false;

    public boolean pathComplete() {
        return pathComplete;
    }


    /**
     * Move robot according to desired axes motions assuming robot centric point of view
     * Positive X is forward
     * Positive Y is strafe right
     * Positive Yaw is clockwise: note this is not how the IMU reports yaw(heading)
     */
    void moveRobot(double drive, double strafe, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower = drive + strafe + yaw;
        double rightFrontPower =  drive - strafe - yaw;
        double leftBackPower = drive - strafe + yaw;
        double rightBackPower = drive + strafe - yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);


        if (showTelemetry) {
            telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            telemetry.addData("Left Front Power",  leftFrontPower);
            telemetry.addData("Right Front Power",  rightFrontPower);
            telemetry.addData("Left Rear Power",  leftBackPower);
            telemetry.addData("Right Rear Power", "%5.2f", rightBackPower);
            telemetry.addData("left front position" , leftFrontMotor.getCurrentPosition());
            telemetry.addData("right front position" , rightFrontMotor.getCurrentPosition());
            telemetry.addData("left rear position" , leftRearMotor.getCurrentPosition());
            telemetry.addData("right rear position" , rightRearMotor.getCurrentPosition());

            //telemetry.update(); //  Assume this is the last thing done in the loop.
        }
    }



    /**
     * Drives the robot a specified distance and angle RELATIVE to its current position.
     * This is a BLOCKING method that wraps the core driveTo() logic. It is ideal
     * for building paths from a sequence of simple movements.
     *
     * @param driveInches  The distance to drive forward (positive) or backward (negative).
     * @param strafeInches The distance to strafe right (positive) or left (negative).
     * @param turnDegrees  The angle to turn clockwise (positive) or counter-clockwise (negative).
     * @param power        The maximum power for the movement.
     * @param holdTime     The time to hold the final position.
     * @return             `true` if the movement succeeded, `false` if it timed out or was interrupted.
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

    public void pinpointDrive(double targetX, double targetY, double targetHeading, int maxTime) {

        pathComplete = false;
        double drive, strafe, turn;
        double xError, yError, distanceToTarget, angleToTarget, yawError, relativeBearing;
        pinpoint.update();
        Pose2D currentPos  = pinpoint.getPosition();

        // Calculate initial errors
        xError = targetX - currentPos.getX(DistanceUnit.INCH);
        yError = targetY - currentPos.getY(DistanceUnit.INCH);
        distanceToTarget = Math.hypot(xError, yError);
        angleToTarget = Math.atan2(yError, xError); // Angle to target in global frame
        yawError = targetHeading - currentPos.getHeading(AngleUnit.DEGREES);
        //yawError = AngleUnit.normalizeDegrees(yawError);

        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(distanceToTarget) > 1) || (Math.abs(yawError) > 2))) {
            pinpoint.update();
            // Recalculate current position
            currentPos = pinpoint.getPosition();

            // Recalculate errors
            xError = targetX - currentPos.getX(DistanceUnit.INCH);
            yError = targetY - currentPos.getY(DistanceUnit.INCH);
            distanceToTarget = Math.hypot(xError, yError);
            angleToTarget = Math.atan2(yError, xError); // Angle to target relative to the X-axis

            // **Relative bearing** (difference between where we want to go and where we're pointing)
            relativeBearing = angleToTarget - Math.toRadians(currentPos.getHeading(AngleUnit.DEGREES));
            relativeBearing = normalizeAngle(Math.toDegrees(relativeBearing)); // Normalize relative bearing
            relativeBearing = Math.toRadians(relativeBearing);

            // Correct yaw error relative to global frame
            telemetry.addData("target angle)", targetHeading);
            telemetry.addData("Yaw ", currentPos.getHeading(AngleUnit.DEGREES));
            yawError = targetHeading - currentPos.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Yaw Error (Before Norm)", yawError);
            //yawError = AngleUnit.normalizeDegrees(yawError);
            telemetry.addData("Yaw Error (after Norm)", yawError);
            //telemetry.update();
            // **Core Movement Calculation**
            // Decompose relativeBearing into drive (forward/backward) and strafe (left/right)
            double movementAngle = relativeBearing; // Relative to the robot's frame
            drive = Range.clip(Math.cos(movementAngle) * distanceToTarget * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(Math.sin(movementAngle) * distanceToTarget * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            // Prioritize rotation if yawError is large
            if (Math.abs(yawError) > 10) {
                drive *= 0.5; // Reduce forward motion while turning
                strafe *= 0.5;
            }

            // **Telemetry for Debugging**
            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("Current X", currentPos.getX(DistanceUnit.INCH));
            telemetry.addData("Current Y", currentPos.getY(DistanceUnit.INCH));
            telemetry.addData("Current Heading", currentPos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Relative Bearing", Math.toDegrees(relativeBearing));
            telemetry.addData("Yaw Error", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain
            moveRobot(drive, strafe, turn);

            // Check for completion
            if ((Math.abs(distanceToTarget) <= 1.5) && (Math.abs(yawError) <= 2)) {
                pathComplete = true;
                break; // Exit the loop early if path is complete
            }
        }

        // Stop the robot
        moveRobot(0, 0, 0);

        // Final telemetry
        pinpoint.update();
        currentPos = pinpoint.getPosition();
        telemetry.addData("Final X", currentPos.getX(DistanceUnit.INCH));
        telemetry.addData("Final Y", currentPos.getY(DistanceUnit.INCH));
        telemetry.addData("Final Heading", currentPos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }


    public void pinPointDrive2(double targetX, double targetY, double targetHeading, int maxTime) {
        pinPointMoveXY(targetX, targetY, maxTime);
        pinPointTurnToHeading(targetHeading, maxTime);
    }

    public void pinPointMoveXY(double targetX, double targetY, int maxTime) {
        double drive, strafe;
        double xError, yError;

        Pose2D currentPos = myPinPointPosition();
        xError = targetX - currentPos.getX(DistanceUnit.INCH);
        yError = targetY - currentPos.getY(DistanceUnit.INCH);

        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(xError) > 1) || (Math.abs(yError) > 1))) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f", drive, strafe);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.getX(DistanceUnit.INCH));
            telemetry.addData("current Y coordinate", currentPos.getY(DistanceUnit.INCH));
            telemetry.addData("current Heading angle", currentPos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, 0);

            // then recalc error
            currentPos = myPinPointPosition();
            xError = targetX - currentPos.getX(DistanceUnit.INCH);
            yError = targetY - currentPos.getY(DistanceUnit.INCH);
            if ((Math.abs(xError) <= 1) && (Math.abs(yError) <= 1)) {
                pathComplete = true;
            }
        }
        moveRobot(0, 0, 0);
        currentPos = myPinPointPosition();

    }

    private Pose2D myPinPointPosition() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    public void pinPointTurnToHeading(double targetHeading, int maxTime) {
        double turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        Pose2D currentPos = myPinPointPosition();
        yawError = targetHeading - currentPos.getHeading(AngleUnit.DEGREES);
        // Normalize yawError to -180 to +180 range
        yawError = AngleUnit.normalizeDegrees(yawError);
        //yawError = normalizeAngle(yawError);
        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(yawError) > 2))) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.

            turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            // Apply desired axes motions to the drivetrain.
            moveRobot(0, 0, turn);

            // then recalc error
            currentPos = myPinPointPosition();
            yawError = targetHeading - currentPos.getHeading(AngleUnit.DEGREES);
           // yawError = ((yawError + 180) % 360) - 180;
            // Normalize yawError to -180 to +180 range
            yawError = normalizeAngle(yawError);
//            if ((Math.abs(yawError) <= 2)) {
//                pathComplete = true;
//            }
        }
        moveRobot(0, 0, 0);
        currentPos = myPinPointPosition();
        telemetry.addData("current X coordinate", currentPos.getX(DistanceUnit.INCH));
        telemetry.addData("current Y coordinate", currentPos.getY(DistanceUnit.INCH));
        telemetry.addData("current Heading angle", currentPos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        startMotion();

        driveController.reset(distanceInches, power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();

        while (readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            sleep(10);

        }
        stopRobot();
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
    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        startMotion();

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(distanceInches, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {

        yawController.reset(headingDeg, power);
        while (readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            sleep(10);
        }
        stopRobot();
    }

    // Add these methods to DriveUtilDepricated.java

    /**
     * Starts a non-blocking movement to align the robot to a specific AprilTag.
     * Call this once, then call update() in a loop until isBusy() is false.
     *
     * @param visionUtil An initialized VisionUtil object to get data from.
     * @param tagId           The ID of the AprilTag to align to.
     * @param distanceInches  The desired final distance from the tag.
     * @param holdTimeSec     The time in seconds to hold the position once reached.
     */
    public void driveToTagAsync(VisionUtil visionUtil, int tagId, double distanceInches, double holdTimeSec) {
        if (driveState == DriveState.IDLE) {
            this.vision = visionUtil;
            this.targetTagId = tagId;
            this.desiredTagDistanceInches = distanceInches;
            this.holdTime = holdTimeSec;
            this.driveState = DriveState.ALIGNING_TO_APRILTAG;
            holdTimer.reset(); // Reset hold timer for stability check
            // --- ADD THIS INITIALIZATION LOGIC ---
            // Reset timers and last known powers for a clean start on every new call.
            holdTimer.reset();
            targetLostTimer.reset();
            lastGoodDrivePower = 0;
            lastGoodStrafePower = 0;
            lastGoodTurnPower = 0;
        }
    }



    /**
     * Private helper method to run the control loop for AprilTag alignment.
     * This is called by the main update() method and uses Limelight SDK data
     * to emulate the logic from the official FTC `RobotAutoDriveToAprilTagOmni` sample.
     */
    private void updateAlignToTag() {
        // Attempt to find the desired AprilTag in the latest vision data
        LLResultTypes.FiducialResult targetTag = vision.getFiducialById(targetTagId);

        // Get the robot's calculated 3D position from the main vision result.
        Pose3D botPose = vision.getBotPose();

        // If we cannot see the tag OR if the botpose is invalid, handle it gracefully.
        if (targetTag == null || botPose == null) {
            // If the target has been lost for too long, give up and stop.
            if (targetLostTimer.seconds() > TARGET_LOST_TIMEOUT_SEC) {
                telemetry.addData("Align Status", "Target lost for too long! Stopping.");
                stopRobot();
                driveState = DriveState.IDLE;
                return;
            }

            // The target is temporarily lost. Continue moving with the last known good powers.
            telemetry.addData("Align Status", "Target Lost! Using last known velocity.");
            moveRobot(lastGoodDrivePower, lastGoodStrafePower, lastGoodTurnPower);
            return; // Exit this loop cycle
        }

        // If we get here, the target is visible and we have a valid botpose.
        targetLostTimer.reset(); // We found the tag, so reset the "lost" timer.

        // --- CORRECTED: Calculate Errors using Limelight data mapped to FTC concepts ---

        // 1. Range Error (for driving forward/backward)
        // We get this from the Limelight's botpose X-axis.
        // This assumes your Limelight coordinate system has X as the forward/backward axis.
        double rangeError = botPose.getPosition().x - desiredTagDistanceInches;

        // 2. Heading Error (for turning the robot to face the tag)
        // This comes directly from the Limelight's 'tx' value.
        double headingError = targetTag.getTargetXDegrees();

        // 3. Yaw Error (for strafing the robot to be centered on the tag)
        // We get this from the Limelight's botpose Z-axis.
        // This assumes your Limelight coordinate system has Z as the left/right axis.
        double yawError = botPose.getPosition().z;

        // --- Check for Completion ---
        boolean isAligned = (Math.abs(rangeError) < TAG_RANGE_TOLERANCE_IN) &&
                (Math.abs(yawError) < TAG_STRAFE_TOLERANCE_IN) &&
                (Math.abs(headingError) < TAG_TURN_TOLERANCE_DEG);

        if (isAligned) {
            // We are at the target. If we've held the position long enough, we're done.
            if (holdTimer.time() > this.holdTime) {
                stopRobot();
                driveState = DriveState.IDLE;
            } else {
                // We're in position but haven't held it yet. Command a stop to hold position.
                stopRobot();
            }
        } else {
            // --- Proportional Control Calculation (This logic now matches the FTC sample) ---
            double drivePower  = Range.clip(rangeError * TAG_RANGE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turnPower   = Range.clip(-headingError * TAG_TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafePower = Range.clip(-yawError * TAG_STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // Store these as the "last known good" powers for when the tag is lost.
            lastGoodDrivePower = drivePower;
            lastGoodStrafePower = strafePower;
            lastGoodTurnPower = turnPower;

            // Command the robot to move.
            moveRobot(drivePower, strafePower, turnPower);
            holdTimer.reset(); // If we are moving, we must reset the hold timer.
        }

        // Add Telemetry for debugging
        telemetry.addData("Align Status", isAligned ? "Holding Position" : "Aligning...");
        telemetry.addData("Range Err (Drive)", "%.2f in", rangeError);
        telemetry.addData("Yaw Err (Strafe)", "%.2f in", yawError);
        telemetry.addData("Bearing Err (Turn)", "%.2f deg", headingError);
    }





    /**
     * This method MUST be called in every loop of the OpMode.
     * It routes to the correct update logic based on the current drive state.
     */
    public void update() {
        switch (driveState) {
            case IDLE:
                // Do nothing if the robot is idle.
                break;
            case DRIVING_TO_ENCODER_TARGET:
                // updateDriveToPosition(); // Your logic for encoder driving would go here
                break;
            case ALIGNING_TO_APRILTAG:
                updateAlignToTag(); // Our new alignment logic
                break;
        }
    }

    /**
     * Checks if the drivetrain is currently executing a motion.
     * @return true if the robot is driving, false if it is idle.
     */
    public boolean isBusy() {
        return areMotorsBusy();
        //return driveState != DriveState.IDLE;
    }
    //  ########################  Low level control functions.  ###############################



    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }

    //GB poinpoint methods

    public void setDriveType(DriveType driveType){
        selectedDriveType = driveType;
    }


    public void setXYCoefficients(double p, double d, double acceleration, DistanceUnit unit, double tolerance){
        pGain = p;
        dGain = d;
        accel = acceleration;
        xyTolerance = unit.toMm(tolerance);
    }

    public void setYawCoefficients(double p, double d, double acceleration, AngleUnit unit, double tolerance){
        yawPGain = p;
        yawDGain = d;
        yawAccel = acceleration;
        yawTolerance = unit.toRadians(tolerance);
    }

    public double getMotorPower(DriveMotor driveMotor){
        if(driveMotor == DriveMotor.LEFT_FRONT){
            return leftFrontMotorOutput;
        } else if (driveMotor == DriveMotor.RIGHT_FRONT){
            return rightFrontMotorOutput;
        } else if (driveMotor == DriveMotor.LEFT_BACK){
            return leftBackMotorOutput;
        } else {
            return rightBackMotorOutput;
        }
    }

    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
        boolean atTarget;

        if (selectedDriveType == DriveType.TANK){
            double xPWR;
            double hPWR;
            double headingTowardsTarget = calculateTargetHeading(currentPosition,targetPosition);
            double lengthToTarget = Math.hypot((targetPosition.getX(MM) - currentPosition.getX(MM)),(targetPosition.getY(MM) - currentPosition.getY(MM)));
            Pose2D temp = new Pose2D(MM,targetPosition.getX(MM),targetPosition.getY(MM),RADIANS,headingTowardsTarget);

            if (headingTowardsTarget > (Math.PI/2) || headingTowardsTarget < -(Math.PI/2)){
                //headingTowardsTarget -= Math.PI;
                headingTowardsTarget = targetPosition.getHeading(RADIANS);
                lengthToTarget = -lengthToTarget;
            }

            xPWR = xTankPID.calculateAxisPID(lengthToTarget,pGain,dGain,accel, PIDTimer.seconds());
            hPWR = calculatePID(currentPosition, temp, Direction.h);
            calculateTankOutput(xPWR * power, hPWR * power);


            //Mecanum Drive Code:
        } else {
            double xPWR = calculatePID(currentPosition, targetPosition, Direction.x);
            double yPWR = calculatePID(currentPosition, targetPosition, Direction.y);
            double hOutput = calculatePID(currentPosition, targetPosition, Direction.h);

            double heading = currentPosition.getHeading(AngleUnit.RADIANS);
            double cosine = Math.cos(heading);
            double sine = Math.sin(heading);

            double xOutput = (xPWR * cosine) + (yPWR * sine);
            double yOutput = (xPWR * sine) - (yPWR * cosine);

            //calculateMecanumOutput(xOutput * power, yOutput * power, hOutput * power);
            moveRobot(xOutput * power, yOutput * power, -(hOutput * power));
        }

        if(inBounds(currentPosition,targetPosition) == InBounds.IN_BOUNDS){
            atTarget = true;
        }
        else {
            GBholdTimer.reset();
            atTarget = false;
        }

        if(atTarget && GBholdTimer.time() > holdTime){
            return true;
        }
        return false;
    }

    private void calculateMecanumOutput(double forward, double strafe, double yaw) {
        double leftFront = forward - -strafe - yaw;
        double rightFront = forward + -strafe + yaw;
        double leftBack = forward + -strafe - yaw;
        double rightBack = forward - -strafe + yaw;

        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        setMotorPowers(leftFront,leftBack,rightBack,rightFront);
    }

    private void calculateTankOutput(double forward, double yaw){
        double left = forward - yaw;
        double right = forward + yaw;

        double max = Math.max(Math.abs(left),Math.abs(right));

        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        setMotorPowers(left,left,right,right);
    }


    private double calculatePID(Pose2D currentPosition, Pose2D targetPosition, Direction direction){
        if(direction == Direction.x){
            double xError = targetPosition.getX(MM) - currentPosition.getX(MM);
            return xPID.calculateAxisPID(xError, pGain, dGain, accel,PIDTimer.seconds());
        }
        if(direction == Direction.y){
            double yError = targetPosition.getY(MM) - currentPosition.getY(MM);
            return yPID.calculateAxisPID(yError, pGain, dGain, accel, PIDTimer.seconds());
        }
        if(direction == Direction.h){
            double hError = targetPosition.getHeading(AngleUnit.RADIANS) - currentPosition.getHeading(AngleUnit.RADIANS);
            return hPID.calculateAxisPID(hError, yawPGain, yawDGain, yawAccel, PIDTimer.seconds());
        }
        return 0;
    }

    private InBounds inBounds (Pose2D currPose, Pose2D trgtPose){
        boolean xInBounds = currPose.getX(MM) > (trgtPose.getX(MM) - xyTolerance) && currPose.getX(MM) < (trgtPose.getX(MM) + xyTolerance);
        boolean yInBounds = currPose.getY(MM) > (trgtPose.getY(MM) - xyTolerance) && currPose.getY(MM) < (trgtPose.getY(MM) + xyTolerance);
        boolean hInBounds = currPose.getHeading(RADIANS) > (trgtPose.getHeading(RADIANS) - yawTolerance) &&
                currPose.getHeading(RADIANS) < (trgtPose.getHeading(RADIANS) + yawTolerance);

        if (xInBounds && yInBounds && hInBounds){
            return InBounds.IN_BOUNDS;
        } else if (xInBounds && yInBounds){
            return InBounds.IN_X_Y;
        } else if (hInBounds){
            return InBounds.IN_HEADING;
        } else
            return InBounds.NOT_IN_BOUNDS;
    }

    public double calculateTargetHeading(Pose2D currPose, Pose2D trgtPose){
        double xDelta = trgtPose.getX(MM) - currPose.getX(MM);
        double yDelta = trgtPose.getY(MM) - currPose.getY(MM);

        if(Math.abs(xDelta) > xyTolerance || Math.abs(yDelta) > xyTolerance){
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

        /** Degrees version if you ever need it. */
        public double normDeltaDeg(double degrees) {
            while (degrees > 180)  degrees -= 360;
            while (degrees < -180) degrees += 360;
            return degrees;
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

    /************ GRAND EXPERIMENTS ******
    * experimental code to use at your own risk :-)
    * good luck
     *
     */

    private void updateRobotPoseOffsetFromLimeLight() {
        //get the camera
        limelight.updateRobotOrientation(toDegrees(follower.getPoseTracker().getIMUHeadingEstimate()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // have a camera pose
            Pose3D botPose_mt2 = result.getBotpose_MT2();

            if (botPose_mt2 != null) {
                PoseTracker poseTracker = follower.getPoseTracker();
                // good bot pose
                double x = botPose_mt2.getPosition().x;
                double y = botPose_mt2.getPosition().y;
                double theta = botPose_mt2.getOrientation().getYaw();
                Pose botPose2d = new Pose(x, y, theta);

                // get difference between vision bot pose and odo bot pose
                Pose diff = poseTracker.getRawPose().minus(botPose2d);

                //kalman filter the difference between the vision and odometry
                xFilter.update(diff.getX(), 0);
                yFilter.update(diff.getX(), 0);
                thetaFilter.update(diff.getHeading(), 0);

                //update pose tracker offsets
                poseTracker.setXOffset(xFilter.getState());
                poseTracker.setYOffset(yFilter.getState());
                poseTracker.setHeadingOffset(thetaFilter.getState());
            }
        }
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

}

//****************************************************************************************************
//****************************************************************************************************

/***  sub-class used by simplified odometry movements (a step beyond deadreckoning with encoders, but not
 * advanced path planning or the pinpoint computer
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class DriveUtilProportionalControldepricated {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    /**
     *
     * @param gain
     * @param accelLimit
     * @param outputLimit   Clip output to +/- this value
     * @param tolerance     Absolute error less than this value is considered "inPosition"
     * @param deadband      Absolute error less than this value causes zero output
     * @param circular      set True if working with circular heading that wraps at 0/360
     */
    public DriveUtilProportionalControldepricated(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }

    public void snapPedroPathing() {



    }
}

/* sub classs using the pinpoint pid movments*/
class PIDLoopDepricated {
    private double previousError;
    private double previousTime;
    private double previousOutput;

    private double errorR;

    public double calculateAxisPID(double error, double pGain, double dGain, double accel, double currentTime){
        double p = error * pGain;
        double cycleTime = currentTime - previousTime;
        double d = dGain * (previousError - error) / (cycleTime);
        double output = p - d;
        double dV = cycleTime * accel;
        double dVup = cycleTime * accel;
        double dVdown = cycleTime * accel*2;

        double max = Math.abs(output);
        if(max > 1.0){
            output /= max;
        }

//        if((output - previousOutput) > dV){
//            output = previousOutput + dV;
//        } else if ((output - previousOutput) < -dV){
//            output = previousOutput - dV;
//        }
        if((output - previousOutput) > dVup){
            output = previousOutput + dVup;
        } else if ((output - previousOutput) < -dVdown){
            output = previousOutput - dVdown;
        }
        previousOutput = output;
        previousError  = error;
        previousTime   = currentTime;

        errorR = error;

        return output;
    }


}

