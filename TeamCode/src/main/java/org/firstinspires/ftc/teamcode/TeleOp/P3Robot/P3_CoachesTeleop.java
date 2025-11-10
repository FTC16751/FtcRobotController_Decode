package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// LIMELIGHT imports

import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot;

@TeleOp(name="P3 Teleop (Coaches opmode)", group=" _P3opmodes")
public class P3_CoachesTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private P3_Robot robot;
    private double lastTagDistanceMeters = -1.0;
    private int lastTagId = -1;

    // --- OPMODE STATE VARIABLES ---
    private enum AllianceColor { RED, BLUE }
    private AllianceColor alliance = AllianceColor.RED;
    private enum IntakeState { ON, OFF, REVERSE }
    private IntakeState intakeState = IntakeState.OFF;
    private boolean isShooterOn = false;
    private static final double DRIVE_SPEED = 1.0;
    private static final double JOYSTICK_DEADBAND = 0.05;
    private static final double INTAKE_POWER = 1.0;
    private double targetVelocityForDistance;
    private double lastKnownGoodVelocity = 0.0;

    @Override
    public void init() {
        robot = new P3_Robot(hardwareMap,telemetry);
        telemetry.addData("Status", "Initialized P3 Robot");
    }

    @Override
    public void init_loop() {
        // You can add alliance selection logic here
        if (gamepad1.x) { alliance = AllianceColor.BLUE; }
        if (gamepad1.b) { alliance = AllianceColor.RED; }
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // 1. Always update the robot's state first
        robot.update();

        // 2. Delegate all control logic to helper methods
        doDriveControls();
        handleIntakeControls();
        handleLauncherControls();
        calcShooterVelocity();
        // 3. Display telemetry
        robot.addTelemetry();
        doTelemetry();
        telemetry.update();
    }

    private void doTelemetry() {
        telemetry.addData("targetVelocityForDistance", targetVelocityForDistance);
        telemetry.addData("distanceToTagMeters", robot.vision.getDistanceToTagMeters());

    }

    @Override
    public void stop() {
        robot.stopAll();
        requestOpModeStop();
    }

    private void doDriveControls() {
        double driveInput = gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;
        //robot.drive.arcadeDrive(strafeInput, driveInput, -turnInput, gamepad1.right_stick_y, DRIVE_SPEED);

        // --- 2. Apply Deadband ---
        // If the raw input is less than the deadband, treat it as zero.
        double deadbandedDrive = Math.abs(driveInput) > JOYSTICK_DEADBAND ? driveInput : 0.0;
        double deadbandedStrafe = Math.abs(strafeInput) > JOYSTICK_DEADBAND ? strafeInput : 0.0;
        double deadbandedTurn = Math.abs(turnInput) > JOYSTICK_DEADBAND ? turnInput : 0.0;

        // --- 3. Apply Scaling Curve (Cubic) for Smoothing ---
        // Cubing the input provides finer control at low speeds.
        double smoothedDrive = Math.pow(deadbandedDrive, 3);
        double smoothedStrafe = Math.pow(deadbandedStrafe, 3);
        double smoothedTurn = Math.pow(deadbandedTurn, 3);

        // --- 4. Call arcadeDrive with Smoothed Inputs ---
        // Note: The 'gamepad1.right_stick_y' parameter is still present but its purpose is unclear.
        // It's often used for a speed modifier or to toggle field-centric control.
        robot.drive.arcadeDrive(smoothedStrafe, smoothedDrive, smoothedTurn, gamepad1.right_stick_y, DRIVE_SPEED);

        // Add telemetry to see the effect
        telemetry.addData("Raw Drive", "%.2f", driveInput);
        telemetry.addData("Smoothed Drive", "%.2f", smoothedDrive);
    }

    private void handleIntakeControls() {
        if (gamepad1.aWasPressed()) {
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }

        if (gamepad1.xWasPressed()) {
            intakeState = (intakeState == IntakeState.REVERSE) ? IntakeState.OFF : IntakeState.REVERSE;
        }

        switch (intakeState) {
            case ON:      robot.intake.setIntakeMotorPower(INTAKE_POWER);  break;
            case REVERSE: robot.intake.setIntakeMotorPower(-INTAKE_POWER); break;
            case OFF:     robot.intake.setIntakeMotorPower(0);             break;
        }
    }

    private double calcShooterVelocity() {
        if (robot.vision.isTargetVisible()) {
            double distanceInches = robot.vision.getDistanceToTagMeters()* 39.3701;;
            targetVelocityForDistance = robot.getTargetVelocityForDistance(distanceInches);
            lastKnownGoodVelocity = targetVelocityForDistance;
            return targetVelocityForDistance;
        } else {
            return lastKnownGoodVelocity;
        }
    }

    private void handleLauncherControls() {
        if (gamepad1.right_trigger > 0.8) {
            robot.launcher.setIndexerServoPower(-1.0);
            robot.launcher.setShootingPosition();
        } else if (gamepad1.left_trigger > 0.8) {
            robot.launcher.setIndexerServoPower(1.0);
        } else {
            robot.launcher.setIndexerServoPower(0.0);
            robot.launcher.setStopPosition();
        }

        if (gamepad1.yWasPressed()) { isShooterOn = true; }
        if (gamepad1.bWasPressed()) { isShooterOn = false; }

        if (isShooterOn) {
            // TODO: Replace 1000 with a call to a dynamic velocity calculation method
            robot.launcher.setShooterMotorVelocity(calcShooterVelocity());
        } else {
            robot.launcher.setShooterMotorVelocity(0);
        }
    }
}
