package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3RobotConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot3;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.SharedState;

/**
 * P3 Robot Single-Driver TeleOp with Arcade Drive + Vision Auto-Aim
 *
 * No turret (not competition-ready). No PedroPathing — uses arcade drive.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * GAMEPAD 1 - DRIVER CONTROLS
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * --- DRIVE ---
 * Left Stick Y:        Forward / Backward
 * Left Stick X:        Strafe Left / Right
 * Right Stick X:       Rotate
 * Right Stick Button:  Vision snap-to-target (hold) — auto-aims rotation
 *
 * --- INTAKE ---
 * A:                   Toggle intake ON/OFF
 * B:                   Toggle intake REVERSE/OFF
 *
 * --- LAUNCHER ---
 * Right Trigger:       Feed + launch (hold)
 * Left Trigger:        Reverse feeder / unjam (hold)
 * Y:                   Enable AUTO-TARGETING mode (vision-based velocity)
 * X:                   Stop flywheels
 * D-Pad Left:          Near shot velocity preset
 * D-Pad Right:         Far shot velocity preset
 * D-Pad Up:            Increase velocity by 100 ticks/sec
 * D-Pad Down:          Decrease velocity by 100 ticks/sec
 *
 * --- ALLIANCE ---
 * Left Bumper:         Set alliance to BLUE
 * Right Bumper:        Set alliance to RED
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

@Configurable
@TeleOp(name="P3: Robot 3 TeleOp (RUN ME)", group="_P3opmodes")
public class P3_Robot3_TeleOp_SingleDriver extends OpMode {

    // Vision snap-to-target tuning
    private static final double TX_ALIGN_KP = 0.02;
    private static final double TX_ALIGN_TOLERANCE_DEG = 1.0;

    private ElapsedTime runtime = new ElapsedTime();

    // P3 Robot (all subsystems)
    private P3_Robot3 robot;

    // ========================================
    // STATE VARIABLES
    // ========================================

    // Intake
    private enum IntakeState { ON, OFF, REVERSE }
    private IntakeState intakeState = IntakeState.OFF;

    // Launcher
    private enum LauncherMode {
        AUTO_TARGETING,   // Continuously updates velocity from vision
        MANUAL_OVERRIDE   // Velocity set by button presses
    }
    private LauncherMode launcherMode = LauncherMode.MANUAL_OVERRIDE;
    private double requestedMotorVelocity = 0;

    // Vision snap
    private double angleOnTarget = 0.0;

    // Alliance
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;

    // ========================================
    // INIT
    // ========================================

    @Override
    public void init() {
        robot = new P3_Robot3(hardwareMap, telemetry);

        // Launch system config
        robot.setVelocityTolerance(0.97);
        robot.setStallDetectionThreshold(0.80);
        robot.setKeepFlywheelsSpinning(true);
        robot.resetShotCounters();

        // Alliance from autonomous
        alliance = SharedState.alliance != null ? SharedState.alliance : CommonConstants.Alliance.RED;
        robot.configureVisionForTeleOp(alliance);

        telemetry.addData("Status", "Initialized - SINGLE DRIVER MODE");
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    // ========================================
    // MAIN LOOP
    // ========================================

    @Override
    public void loop() {
        robot.update();

        handleDriveControls();
        handleIntakeControls();
        handleLauncherControls();
        handleAllianceControls();

        displayTelemetry();
    }

    // ========================================
    // DRIVE (Arcade + Vision Snap)
    // ========================================

    private void handleDriveControls() {
        double driveInput = gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;

        // Vision snap-to-target (right stick button)
        boolean isSnappingToTarget = gamepad1.right_stick_button && robot.vision.isTargetVisible();
        double txError = robot.vision.getTargetAngleX();

        if (isSnappingToTarget) {
            if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
                angleOnTarget = 0.0;
            } else {
                angleOnTarget = TX_ALIGN_KP * txError;
            }
            turnInput = angleOnTarget;
        } else {
            angleOnTarget = 0.0;
            turnInput = gamepad1.right_stick_x;
        }

        robot.drive.arcadeDrive(strafeInput, -driveInput, turnInput, gamepad1.right_stick_y, 1.0);
    }

    // ========================================
    // INTAKE
    // ========================================

    private void handleIntakeControls() {
        // A: Toggle intake ON/OFF
        if (gamepad1.aWasPressed()) {
            if (intakeState == IntakeState.ON) {
                intakeState = IntakeState.OFF;
                robot.intake.stopIntake();
                robot.indexer.setPower(0);
            } else {
                intakeState = IntakeState.ON;
                robot.intake.startIntake();
                robot.indexer.setPower(.25);
            }
        }

        // B: Toggle intake REVERSE/OFF
        if (gamepad1.bWasPressed()) {
            if (intakeState == IntakeState.REVERSE) {
                intakeState = IntakeState.OFF;
                robot.intake.stopIntake();
            } else {
                intakeState = IntakeState.REVERSE;
                robot.intake.reversIntake();
            }
        }
    }

    // ========================================
    // LAUNCHER
    // ========================================

    private void handleLauncherControls() {
        // --- FEEDER / INDEXER ---
        if (gamepad1.right_trigger > 0.8) {
            robot.indexer.start();
        } else if (gamepad1.left_trigger > 0.8) {
            robot.indexer.setPower(-0.25);
        } else {
            robot.indexer.stop();
        }

        // --- VELOCITY MODE SELECTION ---

        // Y: Switch to AUTO_TARGETING
        if (gamepad1.yWasPressed()) {
            launcherMode = LauncherMode.AUTO_TARGETING;
        }

        // Any manual button: Switch to MANUAL_OVERRIDE
        if (gamepad1.x || gamepad1.dpad_left || gamepad1.dpad_right
                || gamepad1.dpad_up || gamepad1.dpad_down) {
            launcherMode = LauncherMode.MANUAL_OVERRIDE;
        }

        // --- UPDATE VELOCITY ---
        switch (launcherMode) {
            case AUTO_TARGETING:
                requestedMotorVelocity = robot.updateAndGetTargetVelocity();
                break;

            case MANUAL_OVERRIDE:
                if (gamepad1.dpad_left) {
                    requestedMotorVelocity = P3RobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
                } else if (gamepad1.dpad_right) {
                    requestedMotorVelocity = P3RobotConstants.Launcher.FAR_TARGET_VELOCITY;
                } else if (gamepad1.x) {
                    requestedMotorVelocity = 0;
                    robot.stopLaunchSequence();
                } else if (gamepad1.dpad_up) {
                    requestedMotorVelocity += 100;
                    requestedMotorVelocity = Math.min(requestedMotorVelocity, 2200);
                } else if (gamepad1.dpad_down) {
                    requestedMotorVelocity -= 100;
                    requestedMotorVelocity = Math.max(requestedMotorVelocity, 0);
                }
                break;
        }

        // Apply velocity to flywheels
        robot.launcher.setShooterMotorVelocity(requestedMotorVelocity);
    }

    // ========================================
    // ALLIANCE OVERRIDE
    // ========================================

    private void handleAllianceControls() {
        if (gamepad1.leftBumperWasPressed()) {
            if (SharedState.alliance != CommonConstants.Alliance.BLUE) {
                SharedState.alliance = CommonConstants.Alliance.BLUE;
                alliance = CommonConstants.Alliance.BLUE;
                robot.configureVisionForTeleOp(alliance);
            }
        }

        if (gamepad1.rightBumperWasPressed()) {
            if (SharedState.alliance != CommonConstants.Alliance.RED) {
                SharedState.alliance = CommonConstants.Alliance.RED;
                alliance = CommonConstants.Alliance.RED;
                robot.configureVisionForTeleOp(alliance);
            }
        }
    }

    // ========================================
    // TELEMETRY
    // ========================================

    private void displayTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Alliance", alliance);

        // Vision snap
        if (gamepad1.right_stick_button && robot.vision.isTargetVisible()) {
            telemetry.addData("TX Align", "ON | Error: %.1f°", robot.vision.getTargetAngleX());
        }

        telemetry.addData("Intake", intakeState);

        // Launcher
        telemetry.addData("Launcher", launcherMode == LauncherMode.AUTO_TARGETING ? "AUTO" : "MANUAL");
        telemetry.addData("Target Vel", "%.0f", requestedMotorVelocity);
        telemetry.addData("Actual Vel", "%.0f", robot.getCurrentFlywheelVelocity());

        if (requestedMotorVelocity > 100) {
            double pct = (robot.getCurrentFlywheelVelocity() / requestedMotorVelocity) * 100.0;
            telemetry.addData("Vel %", "%.1f%%", pct);
            telemetry.addData("Ready", robot.areFlywheelsReady(requestedMotorVelocity) ? "YES" : "Spinning...");
        }

        // Stats
        telemetry.addData("Shots", "%d / %d", robot.getShotsFired(), robot.getShotsAttempted());
        if (robot.getShotsAborted() > 0) {
            telemetry.addData("Aborted", robot.getShotsAborted());
        }

        // Vision
        if (robot.vision.isTargetVisible()) {
            telemetry.addData("Vision", "%.1f° @ %.1f\"",
                    robot.vision.getTargetAngleX(),
                    robot.vision.getDistanceToTagInches());
        }

        telemetry.update();
    }

    // ========================================
    // STOP
    // ========================================

    @Override
    public void stop() {
        robot.stopAll();
    }
}