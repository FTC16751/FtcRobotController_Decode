package org.firstinspires.ftc.teamcode.utilities.Common;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil;


import java.util.Arrays;
import java.util.List;

public class DriveUtil2026b {
    // =================================================================================
    // SECTION 1: CLASS MEMBERS AND CONSTANTS
    // =================================================================================

    // --- Robot Physical & Tuning Constants ---
    private static final double ROBOT_SIZE_DIAMETER = 60; //in cm
    private static final double ENCODER_COUNTS_PER_INCH = 45.33;
    private static final double ENCODER_RESOLUTION = 537;
    private static final double WHEEL_DIAMETER_CM = 9.6;
    private static final double WHEEL_DIAMETER_IN = WHEEL_DIAMETER_CM/2.54;//3.75;
    private static final double WHEEL_RADIUS = WHEEL_DIAMETER_CM / 2;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_CM * Math.PI;
    private static final double GEAR_REDUCTION = 1.0;
    private static final double TRACK_WIDTH = 17.5;
    private static final double COUNTS_PER_GEAR_REV = ENCODER_RESOLUTION * GEAR_REDUCTION;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV / 360;
    private static final double COUNTS_PER_REV = 384.5;
    private static final double DRIVE_SPEED = 1.0; // Default drive speed multiplier
    private final double  AXIAL_INCHES_PER_COUNT    = (Math.PI * WHEEL_DIAMETER_IN) / COUNTS_PER_REV;
    private final double  LATERAL_INCHES_PER_COUNT  = AXIAL_INCHES_PER_COUNT * 0.866;

    // --- Drivetrain Motor Members ---
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftRearMotor;
    public DcMotorEx rightRearMotor;
    private List<DcMotorEx> motors;
    public static final String FRONT_LEFT_MOTOR_NAME = "Front_Left";
    public static final String FRONT_RIGHT_MOTOR_NAME = "Front_Right";
    public static final String REAR_LEFT_MOTOR_NAME = "Rear_Left";
    public static final String REAR_RIGHT_MOTOR_NAME = "Rear_Right";

    // --- IMU & Sensor Members ---
    private IMU imu;
    private DistanceSensor sensorDistance;
    private double rawHeading = 0;
    private double headingOffset = 0;

    // --- GoBilda Pinpoint Odometry Members ---
    public GoBildaPinpointDriver pinpoint;

    // --- AprilTag Alignment Members ---
    private VisionUtil vision;
    private int targetTagId;
    private double desiredTagDistanceInches;
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private static final double TARGET_LOST_TIMEOUT_SEC = 1.5;
    private double lastGoodDrivePower = 0;
    private double lastGoodStrafePower = 0;
    private double lastGoodTurnPower = 0;


    private final ElapsedTime GBholdTimer = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();
    private static double xyTolerance = 15.5;
    private static double yawTolerance = 0.0349066;
    private static double pGain = 0.01905;
    private static double dGain = 0.00111;
    private static double accel = 8.0;
    private static double yawPGain = 5.0;
    private static double yawDGain = 0.0;
    private static double yawAccel = 20.0;
    // === NEW CONSTANTS FOR ENCODER BASED AUTONOMOUS USING PID ===
    private static final double DRIVE_GAIN          = 0.085;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.03;    // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.018;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // --- PID & Autonomous Control Members ---
    /* for gobildas pid control */
    private final PinpointPIDLoop xPID = new PinpointPIDLoop();
    private final PinpointPIDLoop yPID = new PinpointPIDLoop();
    private final PinpointPIDLoop hPID = new PinpointPIDLoop();

    /* for dr phils simplified odometry pid control */
    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public DriveUtilProportionalControldepricated driveController     = new DriveUtilProportionalControldepricated(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public DriveUtilProportionalControldepricated strafeController    = new DriveUtilProportionalControldepricated(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public DriveUtilProportionalControldepricated yawController       = new DriveUtilProportionalControldepricated(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // --- General Members ---
    private Telemetry telemetry;
    private OpMode myOpMode; // Even if unused, grouping it here is correct.
    private RobotConfig config; // The injected robot configuration object

    // --- Enums ---
    public enum DriveType { MECANUM, TANK }
    public enum DriveMotor { LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK }
    private enum Direction { x, y, h }
    private enum InBounds { NOT_IN_BOUNDS, IN_X_Y, IN_HEADING, IN_BOUNDS }
    private enum DriveState { IDLE, DRIVING_TO_POINT_PINPOINT, ALIGNING_TO_APRILTAG }
    private DriveState driveState = DriveState.IDLE;

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

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU (deg / sec)
    private boolean showTelemetry     = true;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.
    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU
    private double holdTime;
    // =================================================================================
    // SECTION 2: CONSTRUCTOR & INITIALIZATION
    // =================================================================================

    public DriveUtil2026b(HardwareMap hardwareMap, Telemetry telemetry, OpMode opMode, RobotConfig config) {
        this.telemetry = telemetry;
        this.config = config;

        // Initialize all hardware components
        initializeIMU(hardwareMap);
        initMotors(hardwareMap);
        initOdo(hardwareMap);
    }

    private void initMotors(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        leftRearMotor = hardwareMap.get(DcMotorEx.class, REAR_LEFT_MOTOR_NAME);
        rightRearMotor = hardwareMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR_NAME);
        motors = Arrays.asList(leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor);

        // Use the injected config for directions
        leftFrontMotor.setDirection(config.leftFrontDirection);
        rightFrontMotor.setDirection(config.rightFrontDirection);
        leftRearMotor.setDirection(config.leftRearDirection);
        rightRearMotor.setDirection(config.rightRearDirection);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        // Use the injected config for IMU orientation
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(config.imuLogoDirection, config.imuUsbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetHeading();
    }

    private void initOdo(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();
    }

    private void configurePinpoint() {
        // Use the injected config for Pinpoint setup
        pinpoint.setOffsets(config.pinpointOffsetX_mm, config.pinpointOffsetY_mm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(config.pinpointXPodDirection, config.pinpointYPodDirection);
        pinpoint.resetPosAndIMU();
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
    public void stopMotors() {
        // Stop all motors by setting their power to 0.0
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setPower(0.0);
            }
        }
    }
    private void setMotorRunMode(DcMotorEx.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setMode(runMode);
            }
        }
    }
    public void resetEncoders() {
        // Stop and reset encoders for all motors
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        // Set all motors to RUN_USING_ENCODER mode
        setMotorRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void resetHeading() {
        headingOffset = getHeading();
    }

    public void resetPosAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    public void resetActionTimer() {
        GBholdTimer.reset();
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
        return pinpoint.getHeading(RADIANS);
    }
    public Pose2D getOdoPosition() {
        Pose2D currentPos  = pinpoint.getPosition();
        telemetry.addData("current X coordinate", currentPos.getX(DistanceUnit.INCH));
        telemetry.addData("current Y coordinate", currentPos.getY(DistanceUnit.INCH));
        telemetry.addData("current Heading angle", currentPos.getHeading(AngleUnit.DEGREES));
        return currentPos;
    }

    public boolean isBusy() {
        return driveState != DriveState.IDLE;
    }
    public boolean pathComplete = false;
    public boolean pathComplete() {
        return pathComplete;
    }
    public void setPathComplete(boolean complete) {
        pathComplete = complete;
    }

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





    // =================================================================================
    // SECTION 4: MID-LEVEL DRIVE METHODS (TELEOP)
    // =================================================================================

    public void moveRobot(double drive, double strafe, double yaw) {
        double leftFrontPower = drive + strafe + yaw;
        double rightFrontPower = drive - strafe - yaw;
        double leftBackPower = drive - strafe + yaw;
        double rightBackPower = drive + strafe - yaw;

        // Normalize the motor powers
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
    }

    public void arcadeDrive(double strafe, double drive, double turn, double rightStickY, double speed) {
        // This is where you would apply smoothing/deadband if desired,
        // or just pass the raw values to moveRobot.
        moveRobot(drive * speed, strafe * speed, turn * speed);
    }

    public void fieldCentricDrive(double strafe, double drive, double turn, double speed) {
        // 1. Get the robot's current heading from the IMU.
        // We get it in radians because the Math functions work with radians.
        double botHeading = getPinpointHeading();

        // 2. The Field-Centric Transformation
        // Rotate the joystick inputs by the negative of the robot's heading.
        // This cancels out the robot's rotation from the control scheme.
        double rotatedX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotatedY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        // 3. Call the existing moveRobot method with the new, "rotated" inputs.
        // The turn input remains the same.
        moveRobot(rotatedY * speed, rotatedX * speed, turn * speed);
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
     */
    public void driveRobotToPosition(int[] targetPositions, double targetSpeed) {

        for (int i = 0; i < motors.size(); i++) {
            DcMotorEx motor = motors.get(i);
            if (motor != null) {
                motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(targetPositions[i]);
                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                motor.setPower(targetSpeed);
            }
        }

        // Wait for all motors to finish
        while (areMotorsBusy()) {
            // Optional: Add telemetry here to monitor motor positions.
            // Loop remains active while motors are running to their targets.
        }

        // Stop the robot and reset run mode
        stopRobot();
        setMotorRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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


    // =================================================================================
    // SECTION 5: COMPLEX AUTONOMOUS DRIVE METHODS (USING PID)
    // =================================================================================

    public void update() {
        pinpoint.update();
        switch (driveState) {
            case DRIVING_TO_POINT_PINPOINT:
                // updateDriveToPoint(); // Logic for non-blocking drive would go here
                break;
            case ALIGNING_TO_APRILTAG:
                // updateAlignToTag(); // Logic for non-blocking alignment would go here
                break;
            case IDLE:
            default:
                // Do nothing
                break;
        }
    }

    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
        boolean atTarget;
        double xPWR = calculatePID(currentPosition, targetPosition, Direction.x);
        double yPWR = calculatePID(currentPosition, targetPosition, Direction.y);
        double hOutput = calculatePID(currentPosition, targetPosition, Direction.h);

        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);

        double xOutput = (xPWR * cosine) + (yPWR * sine);
        double yOutput = (xPWR * sine) - (yPWR * cosine);

        moveRobot(xOutput * power, yOutput * power, -(hOutput * power));

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
        if(direction ==Direction.x){
            double xError = targetPosition.getX(MM) - currentPosition.getX(MM);
            return xPID.calculateAxisPID(xError, config.pGain, config.dGain, config.accel,PIDTimer.seconds(), xyTolerance);
        }
        if(direction == Direction.y){
            double yError = targetPosition.getY(MM) - currentPosition.getY(MM);
            return yPID.calculateAxisPID(yError, config.pGain, config.dGain, config.accel, PIDTimer.seconds(), xyTolerance);
        }
        if(direction == Direction.h){
            double targetH = targetPosition.getHeading(AngleUnit.RADIANS);
            double currentH = currentPosition.getHeading(AngleUnit.RADIANS);
            double hError = Angle.normDelta(targetH - currentH);
            //double hError = targetPosition.getHeading(AngleUnit.RADIANS) - currentPosition.getHeading(AngleUnit.RADIANS);
            if (Math.abs(hError) < config.yawTolerance) {
                hPID.pidReset();
                return 0.0;
            }
            return hPID.calculateAxisPID(hError, config.yawPGain, config.yawDGain, config.yawAccel, PIDTimer.seconds(), yawTolerance);
        }
        return 0;
    }

    private InBounds inBounds (Pose2D currPose, Pose2D trgtPose){
        boolean xInBounds = currPose.getX(MM) > (trgtPose.getX(MM) - config.xyTolerance) && currPose.getX(MM) < (trgtPose.getX(MM) + config.xyTolerance);
        boolean yInBounds = currPose.getY(MM) > (trgtPose.getY(MM) - config.xyTolerance) && currPose.getY(MM) < (trgtPose.getY(MM) + config.xyTolerance);
        double targetH = trgtPose.getHeading(RADIANS);
        double currentH = currPose.getHeading(RADIANS);
        double hError = Angle.normDelta(targetH - currentH);
        boolean hInBounds = Math.abs(hError) < config.yawTolerance;
        //boolean hInBounds = currPose.getHeading(RADIANS) > (trgtPose.getHeading(RADIANS) - config.yawTolerance) &&
        //        currPose.getHeading(RADIANS) < (trgtPose.getHeading(RADIANS) + config.yawTolerance);

        if (xInBounds && yInBounds && hInBounds){
            return InBounds.IN_BOUNDS;
        } else if (xInBounds && yInBounds){
            return InBounds.IN_X_Y;
        } else if (hInBounds){
            return InBounds.IN_HEADING;
        } else
            return InBounds.NOT_IN_BOUNDS;
    }

    public void pidReset() {

    }
    public double calculateTargetHeading(Pose2D currPose, Pose2D trgtPose){
        double xDelta = trgtPose.getX(MM) - currPose.getX(MM);
        double yDelta = trgtPose.getY(MM) - currPose.getY(MM);

        if(Math.abs(xDelta) > config.xyTolerance || Math.abs(yDelta) > config.xyTolerance){
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

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void simplifiedOdometryDrive(double distanceInches, double power, double holdTime) {
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

    public class PinpointPIDLoop {
        private double previousError;
        private double previousTime;
        private double previousOutput;

        private double errorR;
        private double integralError = 0.0;
        private static final double I_LIMIT = 0.3; // anti-windup clamp, tune as needed
        private final double iGain = 0.000002; // start tiny


        public double calculateAxisPID(double error, double pGain, double dGain, double accel, double currentTime, double tolerance) {
            double p = error * pGain;
            double cycleTime = currentTime - previousTime;
            double d = dGain * (previousError - error) / (cycleTime);
            double output = p - d;
            double dV = cycleTime * accel;
            double dVup = cycleTime * accel;
            double dVdown = cycleTime * accel *1.5;

            // --- INTEGRAL UPDATE ---
            // Only integrate when we're outside the "close enough" window
            if (Math.abs(error) > tolerance) {
                integralError += error * cycleTime;      // accumulate error over time
                // Anti-windup clamp
                if (integralError > I_LIMIT)  integralError = I_LIMIT;
                if (integralError < -I_LIMIT) integralError = -I_LIMIT;
            } else {
                // When we're very close, reset so it doesn't keep pushing
                integralError = 0.0;
            }

            double i = integralError * iGain;
            output = p + i - d;

            // --- NORMALIZE OUTPUT ---

            double max = Math.abs(output);
            if (max > 1.0) {
                output /= max;
            }

            if ((output - previousOutput) > dVup) {
                output = previousOutput + dVup;
            } else if ((output - previousOutput) < -dVdown) {
                output = previousOutput - dVdown;
            }
            previousOutput = output;
            previousError = error;
            previousTime = currentTime;

            errorR = error;

            return output;
        }
        public void pidReset() {
            previousOutput = 0.0;
            previousError = 0.0;
            previousTime = 0.0;
        }
    }
    public class SimplifiedOdoDriveUtilProportionalControl {
        double lastOutput;
        double gain;
        double accelLimit;
        double defaultOutputLimit;
        double liveOutputLimit;
        double setPoint;
        double tolerance;
        double deadband;
        boolean circular;
        boolean inPosition;
        ElapsedTime cycleTime = new ElapsedTime();

        /**
         * @param gain
         * @param accelLimit
         * @param outputLimit Clip output to +/- this value
         * @param tolerance   Absolute error less than this value is considered "inPosition"
         * @param deadband    Absolute error less than this value causes zero output
         * @param circular    set True if working with circular heading that wraps at 0/360
         */
        public SimplifiedOdoDriveUtilProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
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
         *
         * @param input Current live control input value (from sensors)
         * @return desired output power.
         */
        public double getOutput(double input) {
            double error = setPoint - input;
            double dV = cycleTime.seconds() * accelLimit;
            double output;

            // normalize to +/- 180 if we are controlling heading
            if (circular) {
                while (error > 180) error -= 360;
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

        public boolean inPosition() {
            return inPosition;
        }

        /**
         * Saves a new setpoint and resets the output power history.
         * This call allows a temporary power limit to be set to override the default.
         *
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
         *
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

    }
}
