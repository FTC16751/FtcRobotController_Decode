package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// LIMELIGHT imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2025;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_IntakeUtil;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_LauncherUtil;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot;

import java.util.List;

@TeleOp(name="coaches p3 opmode", group="Iterative OpMode")
public class P3_Teleop extends OpMode
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
    private static final double INTAKE_POWER = 1.0;


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

        // 3. Display telemetry
        robot.addTelemetry();
        telemetry.update();
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
        robot.drive.arcadeDrive(strafeInput, driveInput, -turnInput, gamepad1.right_stick_y, DRIVE_SPEED);
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
        Pose goal;
        switch (alliance) {
            case RED:
                goal = new Pose(140, 140);
                break;
            case BLUE:
            default:
                goal = new Pose(4, 140);
                break;
        }
        Pose robotPose = follower.getPose();
        double distance = robotPose.distanceFrom(goal);
        return 10 * distance;
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
            robot.launcher.setShooterMotorVelocity(1000);
        } else {
            robot.launcher.setShooterMotorVelocity(0);
        }
    }
}
