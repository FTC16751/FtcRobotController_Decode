package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// LIMELIGHT imports

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drivetrain;
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
    //private static final double JOYSTICK_DEADBAND = 0.05;
    // --- NEW: Slew Rate Limiter Variables ---
    // This constant defines how much the motor power can change per second.
    // A value of 3.0 means it takes 1/3 of a second to go from 0% to 100% power.
    // Smaller values = smoother/slower ramp. Larger values = more responsive.
    private static final double SLEW_RATE_LIMIT = 5.0; // Units: Power per Second

    // Variables to store the previous loop's power commands
    private double prevSmoothedDrive = 0.0;
    private double prevSmoothedStrafe = 0.0;
    private double prevSmoothedTurn = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime(); // Timer to measure loop time

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
//        telemetry.addData("targetVelocityForDistance", targetVelocityForDistance);
        telemetry.addData("distanceToTagMeters", robot.vision.getDistanceToTagMeters());
        telemetry.addData("distanceToTagInces", robot.vision.getDistanceToTagMeters()*39.3701);
//        telemetry.addData("distanceToTagInches", calcShooterVelocity());
        telemetry.addData("calculated velocity: ", robot.getFlywheelRpmForDistance((robot.vision.getDistanceToTagMeters()*39.3701)));
//        telemetry.addData("Left Front motor position: ", robot.drive.getmotorPosition(robot.drive.leftFrontMotor));
//        telemetry.addData("Left Rearmotor position: ", robot.drive.getmotorPosition(robot.drive.leftRearMotor));
//        telemetry.addData("Right Front motor position: ", robot.drive.getmotorPosition(robot.drive.rightFrontMotor));
//        telemetry.addData("Right Rear motor position: ", robot.drive.getmotorPosition(robot.drive.rightRearMotor));
        telemetry.addData("current X coordinate", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
        telemetry.addData("current Y coordinate", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
        telemetry.addData("current Heading angle", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));

    }

    @Override
    public void stop() {
        robot.stopAll();
        requestOpModeStop();
    }

    private void doDriveControls() {
        if (gamepad1.backWasPressed()) {
            robot.drive.resetPosAndIMU();
        }
        double driveInput = gamepad1.left_stick_y;
        double strafeInput = -gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;

        // --- Apply Deadband ---
        // If the raw input is less than the deadband, treat it as zero.
        double deadbandedDrive = Math.abs(driveInput) > JOYSTICK_DEADBAND ? driveInput : 0.0;
        double deadbandedStrafe = Math.abs(strafeInput) > JOYSTICK_DEADBAND ? strafeInput : 0.0;
        double deadbandedTurn = Math.abs(turnInput) > JOYSTICK_DEADBAND ? turnInput : 0.0;

        // --- Apply Scaling Curve (Cubic) for Smoothing ---
        // Cubing the input provides finer control at low speeds.
        double smoothedDrive = Math.pow(deadbandedDrive, 2);
        double smoothedStrafe = Math.pow(deadbandedStrafe,2);
        double smoothedTurn = Math.pow(deadbandedTurn, 2);


        // --- Apply Slew Rate Limiter for Dampening ---
        // Get the time elapsed since the last loop, in seconds.
        double loopTime = loopTimer.seconds();
        loopTimer.reset(); // Reset the timer for the next loop

        // Calculate the maximum change allowed in motor power for this loop cycle.
        double maxDelta = SLEW_RATE_LIMIT * loopTime;

        // Apply the limiter. Use Range.clip to constrain the new power to be
        // within `maxDelta` of the previous power.
        smoothedDrive  = Range.clip(smoothedDrive,  prevSmoothedDrive - maxDelta,  prevSmoothedDrive + maxDelta);
        smoothedStrafe = Range.clip(smoothedStrafe, prevSmoothedStrafe - maxDelta, prevSmoothedStrafe + maxDelta);
        smoothedTurn   = Range.clip(smoothedTurn,   prevSmoothedTurn - maxDelta,   prevSmoothedTurn + maxDelta);

        // Store the new limited powers as the "previous" values for the next loop.
        prevSmoothedDrive = smoothedDrive;
        prevSmoothedStrafe = smoothedStrafe;
        prevSmoothedTurn = smoothedTurn;


        robot.drive.arcadeDrive(strafeInput, driveInput, turnInput, 0, 1.0);
        //robot.drive.fieldCentricDrive(smoothedStrafe, smoothedDrive, -smoothedTurn, 1.0);
        // Add telemetry to see the effect
//        telemetry.addData("Raw Drive", "%.2f", driveInput);
//        telemetry.addData("Smoothed Drive", "%.2f", smoothedDrive);
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
