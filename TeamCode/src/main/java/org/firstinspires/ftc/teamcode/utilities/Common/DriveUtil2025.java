
package org.firstinspires.ftc.teamcode.utilities.Common;

import static android.os.SystemClock.sleep;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static java.lang.Math.toDegrees;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class DriveUtil2025 {
    // Robot Constants
    private static double ROBOT_SIZE_DIAMETER = 60; //in cm
    private static final double ENCODER_COUNTS_PER_INCH = 45.33;
    private static final double ENCODER_RESOLUTION = 537; //for 312 rpm gobilda motor
    //private static final double ENCODER_RESOLUTION = 384.5; //for 435 gobilda motor
    private static final double GEAR_RATIO = 1;
    private static final double MAX_MOTOR_SPEED = 0.8;
    private static final double MOTOR_ACCELERATION = 1;
    private static final double WHEEL_DIAMETER_INCHES = 3.77953;
    private static final double WHEEL_DIAMETER_CM = 9.6;
    private static final double WHEEL_RADIUS = WHEEL_DIAMETER_CM / 2;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_CM * Math.PI;
    private static final double GEAR_REDUCTION = 1.0;
    private static final double TRACK_WIDTH = 16.45;
    private static final double COUNTS_PER_GEAR_REV = ENCODER_RESOLUTION * GEAR_REDUCTION;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV / 360;

    // PID Constants
    private static final double Kp = 0.005;
    private static final double Ki = 0.00;
    private static final double Kd = 0.00;

    // Ramp Parameters
    private static final double RAMP_UP_TIME = 0.2;
    private static final double RAMP_DOWN_TIME = 0.2;
    private static final double K_ARC = 0.1;

    // Turn Constants
    private static final double P_TURN_GAIN = 0.02;
    private static final double P_DRIVE_GAIN = 0.03;
    private static final double HEADING_THRESHOLD = 1.0;

    // Error Threshold
    private static final double ERROR_THRESHOLD = 10;

    // Class Members
    private OpMode myOpMode = null;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    private ElapsedTime runtime = new ElapsedTime();
    ;
    private IMU imu;
    private Telemetry telemetry;
    private double ENCODER_COUNTS_PER_DEGREE; // Set in constructor

    // Enums
    public enum motors {
        frontLeftMotor,
        frontRightMotor,
        rearLeftMotor,
        rearRightMotor
    }

    public enum UnitOfMeasure {
        INCHES,
        CM
    }

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    final double SPEED_GAIN = 0.15;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.03;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public double MAX_AUTO_SPEED = 0.80;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_TURN = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)
    private SparkFunOTOS myOtos;        // Optical tracking odometry sensor
    SparkFunOTOS.Pose2D pos;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DistanceSensor sensorDistance;

    // limelight camera stuff
    private Limelight3A limelight; //any camera here

    private final KalmanFilterParameters kfParams = new KalmanFilterParameters(6,1); // todo: tune?

    private final KalmanFilter xFilter =  new KalmanFilter(kfParams);
    private final KalmanFilter yFilter = new KalmanFilter(kfParams);
    private final KalmanFilter thetaFilter = new KalmanFilter(kfParams);

    /* local OpMode members. */
    HardwareMap hardwareMap = null;

    public DriveUtil2025(OpMode opmode) {
        myOpMode = opmode;

    }

    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        initMotors(ahwMap);
        initializeIMU(ahwMap);
        //initOtos(ahwMap);
        initOdo(ahwMap);
        runtime = new ElapsedTime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();


        // you can use this as a regular DistanceSensor.
      //  sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
       // Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

    }

    public void update() {
        odo.update();
    }
    private void initMotors(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Front_Left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Front_Right");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Rear_Left");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Rear_Right");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[]{leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void initializeIMU(HardwareMap hardwareMap) {

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void initOtos(HardwareMap hardwareMap) {
        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFun");
        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();
        sleep(1000);
    }

    private void initOdo(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configOdo();
    }
    private void configOdo()
    {
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */

        // x: -2.05
        // y: 0.5
        // at offset x: -1

        // x: -0.017
        // y: -0.58
        // at offset x: -2
        // ROTATE 360 DEGREES
        odo.setOffsets(38, -165,DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #2
        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        //odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

    }
    Orientation myRobotOrientation;

    public double getmotorPower(DcMotor motor) {
        /* tested */
        return motor.getPower();
    }
    public double getmotorPosition(DcMotor motor) {
        /* tested */
        return motor.getCurrentPosition();
    }
    private void setMotorPowersWithRamp(DcMotor[] motors, double[] powers, double maxSpeed, double acceleration) {

        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
        RobotLog.dd("GAMLOG", "current method: " + name + ": called from: " + e);

        double[] desiredPowers = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            desiredPowers[i] = Math.min(Math.max(powers[i], -1.0), 1.0);
            setMotorPowerWithRamp(motors[i], desiredPowers[i], maxSpeed, acceleration);
        }
    }

    private void setMotorPowerWithRamp(DcMotor motor, double targetPower, double maxSpeed, double acceleration) {

        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
        RobotLog.dd("GAMLOG", "current method: " + name + ": called from: " + e);

        double currentPower = motor.getPower();
        double desiredPower = Math.min(Math.max(targetPower, -1.0), 1.0);
        double deltaPower = desiredPower - currentPower;

        //double rampRate = Math.abs(acceleration * runtime.milliseconds() / 1000.0);
        double powerChange = deltaPower; // Calculate change in power

        double newPower = currentPower + powerChange;

        // Ensure new power is within the bounds of -maxSpeed and maxSpeed
        newPower = Math.min(Math.max(newPower, -maxSpeed), maxSpeed);
        RobotLog.dd("GAMLOG", "current method: " + name + " start of telemetry");
        RobotLog.dd("GAMLOG", "current power: " + currentPower);
        RobotLog.dd("GAMLOG", "desired power: " + desiredPower);
        RobotLog.dd("GAMLOG", "delta power: " + deltaPower);
        RobotLog.dd("GAMLOG", "new Power: " + newPower);
        RobotLog.dd("GAMLOG", "position of motor: " + motor.getCurrentPosition());
        RobotLog.dd("GAMLOG", "current power of motor power to set: " + motor.getPower());
        RobotLog.dd("GAMLOG", "current method: " + name + "end of telemetry");
        motor.setPower(newPower);
    }

    // Helper method for logging
    private void logMethodCall() {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3]; // Adjust this number if needed
        RobotLog.dd("GAMLOG", "current method: " + name + ": called from: " + e);
    }

    // Helper method for calculating and applying motor powers
    private void calculateAndSetMotorPowers(double y, double x, double rx, boolean isFieldOriented) {
        DcMotor[] motors = {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};
        if (isFieldOriented) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double tempX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double tempY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            x = tempX * 1.1; // Adjust for strafing
            y = tempY;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double[] powers = {
                (y + x + rx) / denominator,
                (y - x - rx) / denominator,
                (y - x + rx) / denominator,
                (y + x - rx) / denominator
        };
        setMotorPowersWithRamp(motors, powers, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
    }

    // Updated driveMecanum
    public void driveMecanum(double forwardPower, double strafePower, double turnPower, double rightStickY, double driveSpeed) {
        logMethodCall();
        double y = forwardPower * driveSpeed;
        double x = strafePower * 1.1 * driveSpeed;
        double rx = turnPower * driveSpeed;
        calculateAndSetMotorPowers(y, x, rx, false);
    }

    // Updated driveFieldOriented
    public void driveFieldOriented(double forwardPower, double strafePower, double turnPower, double rightStickY, double driveSpeed) {
        logMethodCall();
        double y = forwardPower * driveSpeed;
        double x = strafePower * driveSpeed;
        double rx = turnPower * driveSpeed;
        calculateAndSetMotorPowers(y, x, rx, true);
    }


    // ... Add methods for stopping motors, setting motor mode, etc. ...
    public double getEncoderDistance(DcMotor motor) {
        // Return the current position of the specified motor, or 0 if not recognized
        if (motor == null) {
            return 0; // Handle null cases gracefully
        }
        return motor.getCurrentPosition();
    }

    // Stop all motors
    public void stopMotors() {
        // Stop all motors by setting their power to 0.0
        DcMotor[] motors = {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }

    // Set the motor mode for all motors
    public void setMotorMode(DcMotor.RunMode mode) {
        // Set the specified mode for all motors
        DcMotor[] motors = {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public double encoderTicksToInches(double encoderTicks) {
        return encoderTicks / ENCODER_COUNTS_PER_INCH;
    }

    public double inchesToEncoderTicks(double distance) {
        return distance * ENCODER_COUNTS_PER_INCH;
    }

    public void resetEncoders() {
        DcMotor[] motors = {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};

        // Stop and reset encoders for all motors
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Set all motors to RUN_USING_ENCODER mode
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getAverageEncoderPosition() {
        DcMotor[] motors = {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};
        double totalPosition = 0;

        for (DcMotor motor : motors) {
            totalPosition += motor.getCurrentPosition();
        }

        return totalPosition / motors.length;
    }
    private double getAverageEncoderPosition(DcMotor[] motors) {
        int total = 0;
        for (DcMotor motor : motors) {
            total += motor.getCurrentPosition();
        }
        return total / motors.length;
    }
    private double getAverageEncoderDistance() {
        // Get individual encoder distances for each motor
        double leftDistance = getEncoderDistance(leftFrontMotor);
        double rightDistance = getEncoderDistance(rightFrontMotor);
        double rearLeftDistance = getEncoderDistance(leftRearMotor);
        double rearRightDistance = getEncoderDistance(rightRearMotor);

        // Calculate average encoder distance
        double averageDistance = (leftDistance + rightDistance + rearLeftDistance + rearRightDistance) / 4.0;

        return averageDistance;
    }


    public double getAverageEncoderAngle() {
        // Get average encoder position
        double averagePosition = getAverageEncoderPosition();

        // Convert average position to degrees
        double angle = averagePosition / ENCODER_COUNTS_PER_DEGREE;

        return angle;
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    /////////////// from driveutil 2023
    public void stopRobot() {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        //RobotLog.dd("GAMLOG", "current method: "+name);
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
        String methodName = e.getMethodName();
        RobotLog.dd("GAMLOG", "current method: " + name + ": called from: " + e);
        // Send calculated power to wheels
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }//StopRobot

    /***
     * Routine that drives forward at a given speed
     * @param targetSpeed
     */
    public void driveRobotForward(double targetSpeed) {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        RobotLog.dd("GAMLOG", "current method: " + name);
        // Send calculated power to wheels
        leftFrontMotor.setPower(targetSpeed);
        rightFrontMotor.setPower(targetSpeed);
        leftRearMotor.setPower(targetSpeed);
        rightRearMotor.setPower(targetSpeed);
    }

    /***
     * Routine that drives Backward at a given speed
     * @param targetSpeed
     */
    public void driveRobotBackward(double targetSpeed) {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        RobotLog.dd("GAMLOG", "current method: " + name);
        // Send calculated power to wheels
        leftFrontMotor.setPower(-targetSpeed);
        rightFrontMotor.setPower(-targetSpeed);
        leftRearMotor.setPower(-targetSpeed);
        rightRearMotor.setPower(-targetSpeed);
    }

    /***
     * Routine that strafes right at a given speed
     * @param targetSpeed
     */
    public void driveRobotStrafeRight(double targetSpeed) {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        RobotLog.dd("GAMLOG", "current method: " + name);
        // Send calculated power to wheels
        leftFrontMotor.setPower(targetSpeed);
        rightFrontMotor.setPower(-targetSpeed);
        leftRearMotor.setPower(-targetSpeed);
        rightRearMotor.setPower(targetSpeed);
    }

    /***
     * Routine that strafes left at a given speed
     * @param targetSpeed
     */
    public void driveRobotStrafeLeft(double targetSpeed) {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        RobotLog.dd("GAMLOG", "current method: " + name);
        // Send calculated power to wheels
        leftFrontMotor.setPower(-targetSpeed);
        rightFrontMotor.setPower(targetSpeed);
        leftRearMotor.setPower(targetSpeed);
        rightRearMotor.setPower(-targetSpeed);
    }

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param //distanceInCM
     * @param targetSpeed
     */
    public void driveRobotToPosition(int[] targetPositions, double targetSpeed) {
        // Reset encoders and set run mode
        stopRobot();
        sleep(10);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions and run mode
        leftFrontMotor.setTargetPosition(targetPositions[0]);
        rightFrontMotor.setTargetPosition(targetPositions[1]);
        leftRearMotor.setTargetPosition(targetPositions[2]);
        rightRearMotor.setTargetPosition(targetPositions[3]);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Drive the robot
        while (leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftRearMotor.isBusy()
                || rightRearMotor.isBusy()) {
            leftFrontMotor.setPower(targetSpeed);
            rightFrontMotor.setPower(targetSpeed);
            leftRearMotor.setPower(targetSpeed);
            rightRearMotor.setPower(targetSpeed);
        }

        // Stop the robot and reset run mode
        stopRobot();
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveRobotDistanceForward(double distanceInCM, double targetSpeed) {
        // ... (logging and unit conversion)
        int targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);
        int[] targetPositions = {targetCount, targetCount, targetCount, targetCount};
        driveRobotToPosition(targetPositions, targetSpeed);
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

    public void rotateLeft90Degrees() {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        RobotLog.dd("GAMLOG", "current method: " + name);
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 54;   //diameter in cms measured between left front and right rear or RF and LR
        double angleInDegrees = -90;
        double circumference = Math.PI * ROBOT_SIZE_DIAMETER;
        double distanceToTravel = (angleInDegrees / 360.0) * circumference;
        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        //targetCount = (int) Math.round(COUNTS_PER_GEAR_REV  / WHEEL_CIRCUMFERENCE * ROBOT_SIZE_DIAMETER * Math.PI / 4);
        //targetCount = (int) Math.round(COUNTS_PER_GEAR_REV  / WHEEL_CIRCUMFERENCE * ROBOT_SIZE_DIAMETER * Math.PI / 4);

        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceToTravel);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        leftFrontMotor.setTargetPosition(-targetCount);
        rightFrontMotor.setTargetPosition(targetCount);
        leftRearMotor.setTargetPosition(-targetCount);
        rightRearMotor.setTargetPosition(targetCount);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftRearMotor.isBusy()
                || rightRearMotor.isBusy()) {
            // Send calculated power to wheels
            leftFrontMotor.setPower(targetSpeed);
            rightFrontMotor.setPower(targetSpeed);
            leftRearMotor.setPower(targetSpeed);
            rightRearMotor.setPower(targetSpeed);
        }//end while

        stopRobot();
    }
    public void rotateByXDegrees(boolean clockwise, double targetAngle, double speed, long timeoutMillis) {
        String name = new Object(){}.getClass().getEnclosingMethod().getName();
        RobotLog.dd("GAMLOG", "current method: "+name);

        DcMotor[] motors = {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};
        int targetCount;
        double targetSpeed = speed;
        double diameter = 64;//80;//56.8;   //diameter in cms measured between left front and right rear or RF and LR
        long startTime = System.currentTimeMillis();
        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * diameter * Math.PI / (360/targetAngle));
        double initialPosition = getAverageEncoderPosition(motors);
        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        resetEncoders();


        if (clockwise) {
            //set target stop and mode for running to a position
            leftFrontMotor.setTargetPosition(targetCount);
            rightFrontMotor.setTargetPosition(-targetCount);
            leftRearMotor.setTargetPosition(targetCount);
            rightRearMotor.setTargetPosition(-targetCount);
        } else {
            leftFrontMotor.setTargetPosition(-targetCount);
            rightFrontMotor.setTargetPosition(targetCount);
            leftRearMotor.setTargetPosition(-targetCount);
            rightRearMotor.setTargetPosition(targetCount);
        }
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //continue driving until all of the motors hits the distance
        while ( (System.currentTimeMillis() - startTime) < timeoutMillis &&
                (leftFrontMotor.isBusy()
                        || rightFrontMotor.isBusy()
                        //|| left_rear_motor.isBusy()
                        //|| right_rear_motor.isBusy()
                )
        ) {
            // Send calculated power to wheels
            leftFrontMotor.setPower(targetSpeed);
            rightFrontMotor.setPower(targetSpeed);
            leftRearMotor.setPower(targetSpeed);
            rightRearMotor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        // reverseMotor(right_front_motor);
        // reverseMotor(right_rear_motor);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    DcMotor reverseMotor(DcMotor thisMotor) {
        /****************
         * Use this to reverse the motor direction so encoding will work
         * @param thisMotor
         * @return reversed motor
         */
        if (thisMotor.getDirection() == DcMotor.Direction.FORWARD)
            thisMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        else thisMotor.setDirection(DcMotor.Direction.FORWARD);
        return thisMotor;
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        String name = new Object() {
        }.getClass().getEnclosingMethod().getName();
        RobotLog.dd("GAMLOG", "current method: " + name);
        leftFrontMotor.setPower(lf);
        leftRearMotor.setPower(lr);
        rightRearMotor.setPower(rr);
        rightFrontMotor.setPower(rf);
        //telemetry.addData("setmotorpowers", v);
    }

    //Set power to all motors
    public void setAllPower(double p) {
        setMotorPowers(p, p, p, p);
    }

    public void simpleTankDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = -left_stick_y * DRIVE_SPEED;
        turn = right_stick_x * (DRIVE_SPEED * .8);

        // Combine drive and turn for blended motion.
        left = (drive + turn);
        right = (drive - turn);

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Send calculated power to wheels
        setMotorPowers(left, left, right, right);
    }

    public void tankDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
      /* This enables a tank drive-like arrangement where the left_stick controls the left
            wheels and the right_stick controls the right wheels uses it x/y values and hypotenuse
            to assign magnitude to the stick_y and stick_x values.  Avoids divide by 0 by checking
            hypotenuse
        */
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;
        double L_HYPOTENUSE;
        double R_HYPOTENUSE;

        L_HYPOTENUSE = Math.sqrt(left_stick_y * left_stick_y +
                left_stick_x * left_stick_x);
        if (L_HYPOTENUSE == 0) {
            leftFrontPower = 0;
            leftBackPower = 0;
        } else {
            leftFrontPower = -left_stick_y *
                    Math.abs(left_stick_y / L_HYPOTENUSE);
            leftFrontPower += left_stick_x *
                    Math.abs(left_stick_x / L_HYPOTENUSE);
            leftBackPower = -left_stick_y *
                    Math.abs(left_stick_y / L_HYPOTENUSE);
            leftBackPower -= left_stick_x *
                    Math.abs(left_stick_x / L_HYPOTENUSE);
        }
        R_HYPOTENUSE = Math.sqrt(right_stick_y * right_stick_y +
                right_stick_x * right_stick_x);
        if (R_HYPOTENUSE == 0) {
            rightFrontPower = 0;
            rightBackPower = 0;
        } else {
            rightFrontPower = -right_stick_y *
                    Math.abs(right_stick_y / R_HYPOTENUSE);
            rightFrontPower += right_stick_x *
                    Math.abs(right_stick_x / R_HYPOTENUSE);
            rightBackPower = -right_stick_y *
                    Math.abs(right_stick_y / R_HYPOTENUSE);
            rightBackPower -= right_stick_x *
                    Math.abs(right_stick_x / R_HYPOTENUSE);
        }

        //Ensure Power is between -1 and 1, then factor down by DRIVE_SPEED
        leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0) * DRIVE_SPEED;
        leftBackPower = Range.clip(leftBackPower, -1.0, 1.0) * DRIVE_SPEED;
        rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0) * DRIVE_SPEED;
        rightBackPower = Range.clip(rightBackPower, -1.0, 1.0) * DRIVE_SPEED;

        // Send calculated power to wheels
        setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
    }

    public void arcadeDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
        double y = -left_stick_y * DRIVE_SPEED; // Remember, this is reversed!
        double x = left_stick_x * 1.1 * DRIVE_SPEED; // Counteract imperfect strafing
        double rx = right_stick_x * DRIVE_SPEED;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    private double calculateEncoderCountsPerDegree(double wheelDiameter, int encoderResolution, double gearRatio) {
        // Calculate circumference of the wheel
        double circumference = Math.PI * wheelDiameter;

        // Calculate distance traveled per encoder tick
        double distancePerTick = circumference / encoderResolution * gearRatio;

        // Calculate encoder counts per degree
        return 360 / distancePerTick;
    }

    public SparkFunOTOS getMyOtos() {
        myOtos.getStatus();
        return myOtos;
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(7, 4, 90); // should be -3.75 & -7.5 and 90
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(.996);
        myOtos.setAngularScalar(0.999793126138);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        //myOtos.calibrateImu(510, true);
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    public void otosDriveReset() {
        myOtos.resetTracking();
    }

    /**
     * Move robot to a designated X,Y position and heading
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */
    public boolean pathComplete = false;

    public boolean pathComplete() {
        return pathComplete;
    }

    public void otosDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX - currentPos.x;
        yError = targetY - currentPos.y;
        yawError = targetHeading - currentPos.h;

        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(xError) > 1) || (Math.abs(yError) > 1) || (Math.abs(yawError) > 2))) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

            // then recalc error
            currentPos = myPosition();
            xError = targetX - currentPos.x;
            yError = targetY - currentPos.y;
            yawError = targetHeading - currentPos.h;
            yawError = ((yawError + 180) % 360) - 180;
            if ((Math.abs(xError) <= 1) && (Math.abs(yError) <= 1) && (Math.abs(yawError) <= 2)) {
                pathComplete = true;
            }
        }
        moveRobot(0, 0, 0);
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    public void otosDrive2(double targetX, double targetY, double targetHeading, int maxTime) {
        moveXY(targetX, targetY, maxTime);
        turnToHeading(targetHeading, maxTime);
    }

    public void moveXY(double targetX, double targetY, int maxTime) {
        double drive, strafe;
        double xError, yError;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX - currentPos.x;
        yError = targetY - currentPos.y;

        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(xError) > 1) || (Math.abs(yError) > 1))) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f", drive, strafe);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, 0);

            // then recalc error
            currentPos = myPosition();
            xError = targetX - currentPos.x;
            yError = targetY - currentPos.y;
            if ((Math.abs(xError) <= 1) && (Math.abs(yError) <= 1)) {
                pathComplete = true;
            }
        }
        moveRobot(0, 0, 0);
        currentPos = myPosition();

    }

    public void turnToHeading(double targetHeading, int maxTime) {
        double turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        yawError = targetHeading - currentPos.h;

        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(yawError) > 2))) {
            // Use the speed and turn "gains" to calculate how we want the robot to move.

            turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(0, 0, turn);

            // then recalc error
            currentPos = myPosition();
            yawError = targetHeading - currentPos.h;
            yawError = ((yawError + 180) % 360) - 180;
            if ((Math.abs(yawError) <= 2)) {
                pathComplete = true;
            }
        }
        moveRobot(0, 0, 0);
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */
    SparkFunOTOS.Pose2D myPosition() {
        pos = myOtos.getPosition();
        //SparkFunOTOS.Pose2D myPos = new SparkFunOTOS.Pose2D(pos.y, pos.x, -pos.h);
        SparkFunOTOS.Pose2D myPos = new SparkFunOTOS.Pose2D(pos.x, -pos.y, -pos.h);
        return (myPos);
    }



    /**
     * Move robot according to desired axes motions assuming robot centric point of view
     * Positive X is forward
     * Positive Y is strafe right
     * Positive Yaw is clockwise: note this is not how the IMU reports yaw(heading)
     */
    void moveRobot(double x, double y, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower = x + y + yaw;
        double rightFrontPower = x - y - yaw;
        double leftBackPower = x - y + yaw;
        double rightBackPower = x + y - yaw;

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
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftRearMotor.setPower(leftBackPower);
        rightRearMotor.setPower(rightBackPower);
        //sleep(10);
    }

    public boolean robotIsMoving() {
        if (leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftRearMotor.isBusy() || rightRearMotor.isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    // Closing braces for the class
    public void otosDriveexperiment(double targetX, double targetY, double targetHeading, int maxTime) {
        pathComplete = false;
        double drive, strafe, turn;
        double xError, yError, distanceToTarget, angleToTarget, yawError, relativeBearing;
        SparkFunOTOS.Pose2D currentPos = myPosition();

        // Calculate initial errors
        xError = targetX - currentPos.x;
        yError = targetY - currentPos.y;
        distanceToTarget = Math.hypot(xError, yError);
        angleToTarget = Math.atan2(yError, xError); // Angle to target in global frame
        yawError = targetHeading - currentPos.h;
        yawError = ((yawError + 180) % 360) - 180; // Normalize to -180 to +180

        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(distanceToTarget) > 1) || (Math.abs(yawError) > 2))) {

            // Recalculate current position
            currentPos = myPosition();

            // Recalculate errors
            xError = targetX - currentPos.x;
            yError = targetY - currentPos.y;
            distanceToTarget = Math.hypot(xError, yError);
            angleToTarget = Math.atan2(yError, xError); // Angle to target relative to the X-axis

            // **Relative bearing** (difference between where we want to go and where we're pointing)
            relativeBearing = angleToTarget - Math.toRadians(currentPos.h);
            relativeBearing = ((relativeBearing + Math.PI) % (2 * Math.PI)) - Math.PI; // Normalize to -PI to +PI

            // Correct yaw error relative to global frame
            yawError = targetHeading - currentPos.h;
            yawError = ((yawError + 180) % 360) - 180; // Normalize to -180 to +180

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
            telemetry.addData("Current X", currentPos.x);
            telemetry.addData("Current Y", currentPos.y);
            telemetry.addData("Current Heading", currentPos.h);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Relative Bearing", toDegrees(relativeBearing));
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
        currentPos = myPosition();
        telemetry.addData("Final X", currentPos.x);
        telemetry.addData("Final Y", currentPos.y);
        telemetry.addData("Final Heading", currentPos.h);
        telemetry.update();
    }
    public void otosDriveexperiment(double targetX, double targetY, double targetHeading, int maxTime, double MAX_AUTO_SPEED, double MAX_AUTO_STRAFE, double MAX_AUTO_TURN) {
        pathComplete = false;
        double drive, strafe, turn;
        double xError, yError, distanceToTarget, angleToTarget, yawError, relativeBearing;
        SparkFunOTOS.Pose2D currentPos = myPosition();

        // Calculate initial errors
        xError = targetX - currentPos.x;
        yError = targetY - currentPos.y;
        distanceToTarget = Math.hypot(xError, yError);
        angleToTarget = Math.atan2(yError, xError); // Angle to target in global frame
        yawError = targetHeading - currentPos.h;
        yawError = ((yawError + 180) % 360) - 180; // Normalize to -180 to +180

        runtime.reset();

        while ((runtime.milliseconds() < maxTime * 1000) &&
                ((Math.abs(distanceToTarget) > 1) || (Math.abs(yawError) > 2))) {

            // Recalculate current position
            currentPos = myPosition();

            // Recalculate errors
            xError = targetX - currentPos.x;
            yError = targetY - currentPos.y;
            distanceToTarget = Math.hypot(xError, yError);
            angleToTarget = Math.atan2(yError, xError); // Angle to target relative to the X-axis

            // **Relative bearing** (difference between where we want to go and where we're pointing)
            relativeBearing = angleToTarget - Math.toRadians(currentPos.h);
            relativeBearing = ((relativeBearing + Math.PI) % (2 * Math.PI)) - Math.PI; // Normalize to -PI to +PI

            // Correct yaw error relative to global frame
            yawError = targetHeading - currentPos.h;
            yawError = ((yawError + 180) % 360) - 180; // Normalize to -180 to +180

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
            telemetry.addData("Current X", currentPos.x);
            telemetry.addData("Current Y", currentPos.y);
            telemetry.addData("Current Heading", currentPos.h);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Relative Bearing", toDegrees(relativeBearing));
            telemetry.addData("Yaw Error", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain
            moveRobot(drive, strafe, turn);

            // Check for completion
            if (((Math.abs(distanceToTarget) <= 1.5) && (Math.abs(yawError) <= 2)))
            {
                pathComplete = true;
                break; // Exit the loop early if path is complete
            }
        }

        // Stop the robot
        moveRobot(0, 0, 0);

        // Final telemetry
        currentPos = myPosition();
        telemetry.addData("Final X", currentPos.x);
        telemetry.addData("Final Y", currentPos.y);
        telemetry.addData("Final Heading", currentPos.h);
        telemetry.update();
    }
    public void pinpointDrive(double targetX, double targetY, double targetHeading, int maxTime) {

        pathComplete = false;
        double drive, strafe, turn;
        double xError, yError, distanceToTarget, angleToTarget, yawError, relativeBearing;
        odo.update();
        Pose2D currentPos  = odo.getPosition();

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
            odo.update();
            // Recalculate current position
            currentPos = odo.getPosition();

            // Recalculate errors
            xError = targetX - currentPos.getX(DistanceUnit.INCH);
            yError = targetY - currentPos.getY(DistanceUnit.INCH);
            distanceToTarget = Math.hypot(xError, yError);
            angleToTarget = Math.atan2(yError, xError); // Angle to target relative to the X-axis

            // **Relative bearing** (difference between where we want to go and where we're pointing)
            relativeBearing = angleToTarget - Math.toRadians(currentPos.getHeading(AngleUnit.DEGREES));
            relativeBearing = normalizeAngle(toDegrees(relativeBearing)); // Normalize relative bearing
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
            telemetry.addData("Relative Bearing", toDegrees(relativeBearing));
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
        odo.update();
        currentPos = odo.getPosition();
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
        odo.update();
        return odo.getPosition();
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

    private double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public void snapPedroPathing() {



    }

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
    public Pose2D getOdoPosition() {
        Pose2D currentPos  = odo.getPosition();
        telemetry.addData("current X coordinate", currentPos.getX(DistanceUnit.INCH));
        telemetry.addData("current Y coordinate", currentPos.getY(DistanceUnit.INCH));
        telemetry.addData("current Heading angle", currentPos.getHeading(AngleUnit.DEGREES));
        return currentPos;
    }
}