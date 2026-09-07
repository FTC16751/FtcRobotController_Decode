package org.firstinspires.ftc.teamcode.utilities.Common;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * LEGACY drive utility, kept only for the older OpModes that still use it:
 * TeleOp/Other/Coachbot, BasicDriveTeleop, BasicOpMode_Linear1, and utilities/PriorSeason/Robot.
 *
 * New code should use DriveUtil2026b, which takes a RobotConfig instead of hardcoding motor
 * directions, IMU orientation, and Pinpoint offsets.
 *
 * This class was trimmed on 2026-09-06 to the two methods those OpModes call, init() and
 * arcadeDrive(), plus the hardware setup they depend on. Behavior of those two methods is
 * unchanged from the full version, including:
 *   - hardcoded motor directions (LF/LR forward, RF/RR reverse) and BRAKE zero-power behavior
 *   - IMU orientation logo UP, USB LEFT
 *   - Pinpoint "odo" offsets (38, -165) mm, 4-bar pods, X forward / Y reversed, reset on init
 *   - a Limelight named "limelight" is REQUIRED and started during init
 *   - arcadeDrive negates left_stick_y internally and scales strafe by 1.1
 *
 * Everything else (encoder moves, Pinpoint/OTOS drive-to-position, Kalman correction, ramping)
 * was removed; its successor lives in DriveUtil2026b.
 */
public class DriveUtil2025 {

    // Hardware
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    private IMU imu;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;

    // Local members
    private OpMode myOpMode = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    public DriveUtil2025(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        hardwareMap = ahwMap;
        initMotors(ahwMap);
        initializeIMU(ahwMap);
        initOdo(ahwMap);
        runtime = new ElapsedTime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
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
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void initOdo(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configOdo();
    }

    private void configOdo() {
        // Pod offsets relative to the tracking point. X pod: left of center is positive.
        // Y pod: forward of center is positive.
        odo.setOffsets(38, -165, DistanceUnit.MM); // tuned for 3110-0002-0001 Product Insight #2
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // X (forward) pod should increase moving forward; Y (strafe) pod should increase moving left.
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Reset position to 0,0,0 and recalibrate the Pinpoint IMU. Robot must be stationary.
        odo.resetPosAndIMU();
    }

    /**
     * Robot-centric arcade drive. Note the sign conventions this legacy API expects:
     * left_stick_y is negated internally (pass the raw gamepad value), and strafe is scaled by 1.1.
     */
    public void arcadeDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
        double y = -left_stick_y * DRIVE_SPEED; // Remember, this is reversed!
        double x = left_stick_x * 1.1 * DRIVE_SPEED; // Counteract imperfect strafing
        double rx = right_stick_x * DRIVE_SPEED;

        // Denominator is the largest motor power (absolute value) or 1.
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1].
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        RobotLog.dd("GAMLOG", "current method: setMotorPowers");
        leftFrontMotor.setPower(lf);
        leftRearMotor.setPower(lr);
        rightRearMotor.setPower(rr);
        rightFrontMotor.setPower(rf);
    }
}
