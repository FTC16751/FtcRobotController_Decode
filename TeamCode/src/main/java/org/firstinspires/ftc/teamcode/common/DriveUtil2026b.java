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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

public class DriveUtil2026b {
    // =================================================================================
    // SECTION 1: CLASS MEMBERS AND CONSTANTS
    // =================================================================================

    // --- Robot Physical Constants ---
    // All come from RobotConfig.calibration (see the robot's config file in its team folder).
    // Assigned once in the constructor.
    private final double ROBOT_SIZE_DIAMETER;       // cm, turning-circle diameter for rotateRobot()
    private final double ENCODER_COUNTS_PER_INCH;   // drive-motor ticks per inch of travel
    private final double COUNTS_PER_GEAR_REV;       // drive-motor ticks per wheel revolution
    private final double WHEEL_CIRCUMFERENCE;       // cm
    private static final double DRIVE_SPEED = 1.0;  // Default drive speed multiplier
    // Encoder-move time limits (driveRobotToPosition). 5 rev/s is a little under a goBILDA 312 rpm
    // motor's free speed, so the estimate errs long; the limit is a safety net, not a stopwatch.
    private static final double DRIVE_MOTOR_MAX_REV_PER_SEC = 5.0;
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
    private DistanceSensor sensorDistance;
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

        // Physical constants from the robot's config (defaults match the pre-R5 hardcoded values)
        RobotConfig.Calibration cal = config.calibration;
        ROBOT_SIZE_DIAMETER      = cal.robotDiameterCm;
        ENCODER_COUNTS_PER_INCH  = cal.encoderCountsPerInch;
        COUNTS_PER_GEAR_REV      = cal.encoderTicksPerRev * cal.gearReduction;
        WHEEL_CIRCUMFERENCE      = cal.wheelDiameterCm * Math.PI;

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
        if (pinpoint != null) pinpoint.resetPosAndIMU();
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
        if (pinpoint == null) return 0.0;
        return pinpoint.getHeading(RADIANS);
    }
    public Pose2D getOdoPosition() {
        if (pinpoint == null) return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D currentPos  = pinpoint.getPosition();
        telemetry.addData("current X coordinate", currentPos.getX(DistanceUnit.INCH));
        telemetry.addData("current Y coordinate", currentPos.getY(DistanceUnit.INCH));
        telemetry.addData("current Heading angle", currentPos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("pinpoin x direction: ", config.odometry.pinpointXPodDirection);
        telemetry.addData("pinpoin y direction: ", config.odometry.pinpointYPodDirection);
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
        double rightBackPower = (drive + strafe - yaw) * config.calibration.rightRearPowerScale;

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
            // Get the robot's current heading from the IMU.
            double botHeading = getPinpointHeading();

            // Field-Centric Transformation
            // Rotate the joystick inputs by the negative of the robot's heading.
            // This cancels out the robot's rotation
            double rotatedX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotatedY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            // Call the existing moveRobot method with the new, "rotated" inputs.
            // The turn input remains the same.
            moveRobot(rotatedY * speed, rotatedX * speed, turn * speed);
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
        setMotorRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        double idealSec = maxTicks / (COUNTS_PER_GEAR_REV * DRIVE_MOTOR_MAX_REV_PER_SEC * power);
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
        int forwardTicks = (int) (forwardInches * ENCODER_COUNTS_PER_INCH);
        int strafeTicks = (int) (strafeInches * ENCODER_COUNTS_PER_INCH * config.calibration.strafeScale); // mecanum strafe slips

        // Turn ticks: the circumference of the circle the robot sweeps in a spin turn (inches, per robot config)
        double turnCircumference = config.calibration.turnCircumferenceIn;
        double turnDistanceInches = (turnDegrees / 360.0) * turnCircumference;
        int turnTicks = (int) (turnDistanceInches * ENCODER_COUNTS_PER_INCH);

        int fl_ticks = forwardTicks + strafeTicks + turnTicks;
        int fr_ticks = forwardTicks - strafeTicks - turnTicks;
        int rl_ticks = forwardTicks - strafeTicks + turnTicks;
        int rr_ticks = forwardTicks + strafeTicks - turnTicks;

        int[] targetPositions = {fl_ticks, fr_ticks, rl_ticks, rr_ticks};

        return driveRobotToPosition(targetPositions, speed);
    }
    public boolean driveRobotDistanceForward(double distanceInCM, double targetSpeed) {
        // Same tick math as driveRobotDistanceBackward. (An earlier version passed the tick count,
        // divided by 25.4, into drive_p3 as if it were inches, which drove about 1.8x too far.)
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {targetCount, targetCount, targetCount, targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    public boolean driveRobotDistanceForwardInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceForward(distanceInCM, targetSpeed);
    }

    public boolean driveRobotDistanceBackward(double distanceInCM, double targetSpeed) {
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {-targetCount, -targetCount, -targetCount, -targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    public boolean driveRobotDistanceBackwardInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceBackward(distanceInCM, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeRight(double distanceInCM, double targetSpeed) {
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV * config.calibration.strafeScale / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {targetCount, -targetCount, -targetCount, targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeRightInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceStrafeRight(distanceInCM, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeLeft(double distanceInCM, double targetSpeed) {
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV * config.calibration.strafeScale / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {-targetCount, targetCount, targetCount, -targetCount};
        return driveRobotToPosition(targetPositions, targetSpeed);
    }

    public boolean driveRobotDistanceStrafeLeftInches(double distanceInInches, double targetSpeed) {
        double distanceInCM = distanceInInches * 2.54;
        return driveRobotDistanceStrafeLeft(distanceInCM, targetSpeed);
    }

    public boolean rotateRobot(double angleInDegrees, double targetSpeed) {
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
        return driveRobotToPosition(targetPositions, targetSpeed);
    }


    // =================================================================================
    // SECTION 5: COMPLEX AUTONOMOUS DRIVE METHODS (USING PID)
    // =================================================================================

    public void update() {
        if (pinpoint != null) pinpoint.update();
        switch (driveState) {
            case DRIVING_TO_POINT_PINPOINT:
                // updateDriveToPoint(); // Logic for non-blocking drive would go here
                break;
            case ALIGNING_TO_APRILTAG:
                // Step the tag approach; it reads the sighting and hands back three powers.
                if (tagApproach.update(tagSighting)) {
                    stopRobot();
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

//    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
//        boolean atTarget;
//        double xPWR = calculatePID(currentPosition, targetPosition, Direction.x);
//        double yPWR = calculatePID(currentPosition, targetPosition, Direction.y);
//        double hOutput = calculatePID(currentPosition, targetPosition, Direction.h);
//
//        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
//        double cosine = Math.cos(heading);
//        double sine = Math.sin(heading);
//
//        double xOutput = (xPWR * cosine) + (yPWR * sine);
//        double yOutput = (xPWR * sine) - (yPWR * cosine);
//
//        moveRobot(xOutput * power, yOutput * power, -(hOutput * power));
//        //moveRobot(xOutput * power, yOutput * power, 0);
//
//        if(inBounds(currentPosition,targetPosition) == InBounds.IN_BOUNDS){
//            atTarget = true;
//        }
//        else {
//            GBholdTimer.reset();
//            atTarget = false;
//        }
//
//        if(atTarget && GBholdTimer.time() > holdTime){
//            return true;
//        }
//        return false;
//    }
public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
    boolean atTarget;

    // Check if we're at target FIRST
    InBounds boundsStatus = inBounds(currentPosition, targetPosition);

    if(boundsStatus == InBounds.IN_BOUNDS){
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

        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);

        double xOutput = (xPWR * cosine) + (yPWR * sine);
        double yOutput = (xPWR * sine) - (yPWR * cosine);

        moveRobot(xOutput * power, yOutput * power, -(hOutput * power));

        return false;  // Still driving
    }
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

    private InBounds inBounds (Pose2D currPose, Pose2D trgtPose){
        boolean xInBounds = currPose.getX(MM) > (trgtPose.getX(MM) - config.pointToPointTuning.xyTolerance) && currPose.getX(MM) < (trgtPose.getX(MM) + config.pointToPointTuning.xyTolerance);
        boolean yInBounds = currPose.getY(MM) > (trgtPose.getY(MM) - config.pointToPointTuning.xyTolerance) && currPose.getY(MM) < (trgtPose.getY(MM) + config.pointToPointTuning.xyTolerance);
        double targetH = trgtPose.getHeading(RADIANS);
        double currentH = currPose.getHeading(RADIANS);
        double hError = Angle.normDelta(targetH - currentH);
        boolean hInBounds = Math.abs(hError) < config.pointToPointTuning.yawTolerance;
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
        if (driveState == DriveState.IDLE) {
            this.tagSighting = sighting;
            tagApproach.start(tagId, standoffInches, holdTimeSec);
            this.driveState = DriveState.ALIGNING_TO_APRILTAG;
        }
    }

    /** Abandon a driveToTagAsync in progress and stop the wheels. Safe to call when idle. */
    public void cancelDriveToTag() {
        if (driveState == DriveState.ALIGNING_TO_APRILTAG) {
            tagApproach.stop();
            stopRobot();
            driveState = DriveState.IDLE;
        }
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

    public class PinpointPIDLoop {
        private double previousError;
        private double previousTime;
        private double previousOutput;
        private double integralSum;
        private double filteredD;
        private double errorR;

        public double calculateAxisPID(double error, double pGain, double iGain, double dGain, double accel, double currentTime, double tolerance)
        {

            // First call initialization
            if (previousTime == 0.0) {
                previousTime = currentTime;
                previousError = error;
                return 0;
            }

            double cycleTime = currentTime - previousTime;
            if (cycleTime <= 1e-3) cycleTime = 1e-3;

            // Check if we're settled - partial reset to avoid pause
            if (Math.abs(error) <= tolerance && Math.abs(previousOutput) < 0.05) {
                previousOutput = 0;
                integralSum = 0;
                filteredD = 0;
                previousError = error;
                previousTime = currentTime;
                return 0;
            }

            // P term
            double p = error * pGain;

            // I term with anti-windup
            integralSum += error * cycleTime;
            if (iGain > 1e-9) {
                double iMaxOutput = 0.2;  // Max contribution from integral
                double maxIntegral = iMaxOutput / iGain;
                integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
            } else {
                integralSum = 0;
            }
            double i = iGain * integralSum;

            // D term - FIX: CORRECTED SIGN (for real this time!)
            double rawD = (error - previousError) / cycleTime;  // Negative error rate
            filteredD = 0.85 * filteredD + 0.15 * rawD;         // Slower filter for odometry
            double d = dGain * filteredD;

            double output = p + i + d;


            // Asymmetric acceleration limiting
            double dV = cycleTime * accel;
            double outputChange = output - previousOutput;

            boolean isBraking = Math.abs(output) < Math.abs(previousOutput);
            boolean isReversing = (output * previousOutput) < 0;

            if (!isBraking || isReversing) {
                // Limit acceleration and direction changes
                if (outputChange > dV) {
                    output = previousOutput + dV;
                } else if (outputChange < -dV) {
                    output = previousOutput - dV;
                }
            }
            // Allow unlimited deceleration when braking (not reversing)


            // Final clamp to maxPower
            output = Math.max(-1.0, Math.min(1.0, output));


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
            integralSum = 0.0;
            filteredD = 0.0;
        }
    }
}
