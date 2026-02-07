package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3RobotConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot3;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.SharedState;

/**
 * P3 Robot TeleOp with PedroPathing Field-Oriented Drive + Turret Control
 *
 * This OpMode combines PedroPathing's superior field-oriented drive control
 * with the P3_Robot3 class that manages all subsystems including the turret.
 *
 * Uses RobotConfig for multi-robot compatibility - simply change the config
 * in init() to run on different robots with different tuning.
 *
 * KEY FEATURES:
 * - Field-oriented drive using PedroPathing (maintains orientation relative to field)
 * - Dual operator control (Driver + Operator)
 * - Servo-based turret with auto-homing and soft limits
 * - Vision-assisted aiming with Limelight
 * - Automated launch sequences managed by P3_Robot3
 * - LED status indicators managed by P3_Robot3
 * - Multi-robot compatible via RobotConfig
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * GAMEPAD 1 - DRIVER CONTROLS (Field-Oriented Drive)
 * ═══════════════════════════════════════════════════════════════════════════
 * Left Stick:        Strafe (field-oriented) - Y=forward/back, X=left/right
 * Right Stick X:     Robot rotation
 *
 * A:                 Toggle intake ON/OFF
 * B:                 Toggle intake REVERSE/OFF
 * X:                 Emergency stop all systems
 * Y:                 Reset field orientation (set current heading as "forward")
 *
 * Left Bumper:       Precision drive mode (50% speed)
 * Right Bumper:      Turbo drive mode (100% speed)
 *
 * D-Pad Up/Down:     Fine forward/backward adjustments
 * D-Pad Left/Right:  Fine strafe adjustments
 *
 * Start:             Toggle robot-centric vs field-centric drive
 * Back:              Toggle detailed telemetry display
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * GAMEPAD 2 - OPERATOR CONTROLS
 * ═══════════════════════════════════════════════════════════════════════════
 * Right Stick X:     Manual turret rotation (left/right)
 *
 * Right Trigger:     LAUNCH - Initiate automated shot sequence
 * Left Trigger:      Manual reverse indexer (unjam)
 *
 * A:                 Set NEAR shot velocity preset (2500 ticks/sec) + Snap turret home
 * B:                 Set FAR shot velocity preset (3500 ticks/sec)
 * X:                 Stop flywheels (manual override)
 * Y:                 Enable AUTO-TARGETING mode (vision-based velocity)
 *
 * Right Bumper:      Auto-aim turret with Limelight (hold)
 *
 * D-Pad Up:          Increase velocity by 100 ticks/sec
 * D-Pad Down:        Decrease velocity by 100 ticks/sec
 * D-Pad Left:        Manual turret nudge left
 * D-Pad Right:       Manual turret nudge right
 *
 * Start:             Toggle flywheel spin mode (rapid-fire / battery-saver)
 * Back:              Reset turret to center position (snap home)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

@Configurable
@TeleOp(name="P3: Robot 3 TeleOp + Turret", group="_P3opmodes")
@Disabled
public class P3_Robot3_TeleOp extends OpMode {

    private static final RobotConfig ROBOT_CONFIG = RobotConfig.createP3Robot2Config();

    private ElapsedTime runtime = new ElapsedTime();

    // PedroPathing
    private Follower follower;
    private TelemetryManager telemetryM;
    public static Pose startingPose = new Pose(0, 0, 0);

    // P3 Robot (contains all subsystems including turret)
    private P3_Robot3 robot;

    // Limelight for turret auto-aim
    private Limelight3A limelight;

    // ========================================
    // STATE VARIABLES
    // ========================================

    // Drive states
    private boolean fieldCentricMode = true;
    private double driveSpeedMultiplier = P3RobotConstants.TeleOp.DRIVE_SPEED_NORMAL;

    // Driver states
    private enum IntakeState {
        ON, OFF, REVERSE
    }
    private IntakeState intakeState = IntakeState.OFF;

    // Operator states
    private enum LauncherMode {
        AUTO_TARGETING,   // Continuously updates velocity from vision
        MANUAL_OVERRIDE   // Velocity set by button presses
    }
    private LauncherMode launcherMode = LauncherMode.MANUAL_OVERRIDE;

    // Turret homing state
    private boolean turretHomingComplete = false;

    // ========================================
    // WORKING VARIABLES
    // ========================================
    private boolean showDebugTelemetry = true;
    private double requestedMotorVelocity = 0;
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;

    // Button state tracking
    private boolean lastRightTriggerPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize PedroPathing using RobotConfig
        follower = Constants.createFollower(hardwareMap, ROBOT_CONFIG);
        follower.setStartingPose(startingPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize P3 Robot (this creates all subsystems including turret)
        robot = new P3_Robot3(hardwareMap, telemetry);

        // Initialize Limelight for turret auto-aim
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Use your tracking pipeline
        limelight.start();

        // Configure launch system for competition
        robot.setVelocityTolerance(0.97);           // Require 97% of target speed
        robot.setStallDetectionThreshold(0.80);     // Abort if velocity drops below 80%
        robot.setKeepFlywheelsSpinning(true);       // Default to rapid-fire mode
        robot.resetShotCounters();                  // Start fresh

        // Load alliance from autonomous (or default to RED)
        alliance = SharedState.alliance != null ? SharedState.alliance : CommonConstants.Alliance.RED;
        robot.configureVisionForTeleOp(alliance);

        telemetry.addData("Status", "Initialized - PEDRO PATH + TURRET MODE");
        telemetry.addData("Robot Config", ROBOT_CONFIG == RobotConfig.createP3Robot2Config() ? "P3 Robot #2" : "Other");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive Mode", "Field-Centric");
        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("  TURRET HOMING - INIT PHASE");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("Turret will automatically find home position");
        telemetry.addLine("Press START when homing complete");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY in init_loop (before START is pressed)
     * This is where we run the turret homing sequence
     */
    @Override
    public void init_loop() {
        if (!turretHomingComplete) {
            turretHomingComplete = robot.turret.updateHoming();

            telemetry.addLine("═══════════════════════════════════════");
            telemetry.addLine("  TURRET HOMING");
            telemetry.addLine("═══════════════════════════════════════");
            telemetry.addData("Status", turretHomingComplete ? "COMPLETE ✓" : "Searching...");
            telemetry.addData("Phase", robot.turret.getHomingPhase());
            telemetry.addData("Switch Triggered", robot.turret.isHomeSwitchTriggered());
            telemetry.addData("Current Position", "%.4f", robot.turret.getCurrentPosition());

            if (turretHomingComplete) {
                telemetry.addLine();
                telemetry.addData("Home Position", "%.4f", robot.turret.getHomePosition());
                telemetry.addData("Position Safe?", robot.turret.isHomePositionSafe() ? "YES ✓" : "NO - CHECK CALIBRATION!");

                if (!robot.turret.isHomePositionSafe()) {
                    telemetry.addLine();
                    telemetry.addLine("⚠ WARNING: Home position outside safe range!");
                    telemetry.addLine("Expected: 0.356 - 0.644");
                    telemetry.addLine("You may lose travel range or hit limits");
                }

                telemetry.addLine();
                telemetry.addLine("✓ READY TO START");
                telemetry.addLine("Press PLAY to begin TeleOp");
            }

            telemetry.update();
        }
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // Safety check - refuse to start if turret not homed
        if (!robot.turret.isHomed()) {
            telemetry.addLine("═══════════════════════════════════════");
            telemetry.addLine("  ERROR: TURRET NOT HOMED!");
            telemetry.addLine("═══════════════════════════════════════");
            telemetry.addLine("Cannot start TeleOp safely");
            telemetry.addLine("Please restart OpMode");
            telemetry.update();
//            requestOpModeStop();
//            return;
        }

        runtime.reset();
        follower.startTeleopDrive(true);

        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("  TELEOP STARTED");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addData("Turret Status", "HOMED ✓");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START
     */
    @Override
    public void loop() {
        // Update all systems
        follower.update();
        robot.update();  // This updates vision, LEDs, etc.
        telemetry.update();

        // Update vision with current heading for field-relative calculations
        robot.vision.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading()));

        // Handle all controls
        handleDriverControls();
        handleOperatorControls();

        // Display telemetry
        displayTelemetry();
    }

    // ════════════════════════════════════════════════════════════════════════
    // DRIVER CONTROLS (GAMEPAD 1)
    // ════════════════════════════════════════════════════════════════════════

    private void handleDriverControls() {
        handleDriveControls();
        handleDriverIntakeControls();
        handleDriverSystemControls();
    }

    private void handleDriveControls() {
        // Determine drive speed based on bumpers
        if (gamepad1.left_bumper) {
            driveSpeedMultiplier = P3RobotConstants.TeleOp.DRIVE_SPEED_PRECISION;
        } else if (gamepad1.right_bumper) {
            driveSpeedMultiplier = P3RobotConstants.TeleOp.DRIVE_SPEED_TURBO;
        } else {
            driveSpeedMultiplier = P3RobotConstants.TeleOp.DRIVE_SPEED_NORMAL;
        }

        // Get joystick inputs
        double forward = -gamepad1.left_stick_y * driveSpeedMultiplier;
        double strafe = -gamepad1.left_stick_x * driveSpeedMultiplier;
        double turn = -gamepad1.right_stick_x * driveSpeedMultiplier;

        // Apply drive command with field-centric mode
        follower.setTeleOpDrive(
                forward,
                strafe,
                turn,
                fieldCentricMode
        );
    }

    /**
     * Driver intake controls
     */
    private void handleDriverIntakeControls() {
        // A button: Toggle intake ON/OFF
        if (gamepad1.aWasPressed()) {
            if (intakeState == IntakeState.ON) {
                intakeState = IntakeState.OFF;
                robot.intake.stopIntake();
            } else {
                intakeState = IntakeState.ON;
                robot.intake.startIntake();
            }
        }

        // B button: Toggle intake REVERSE/OFF
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

    /**
     * Driver system controls
     */
    private void handleDriverSystemControls() {

        // Y button: Reset field orientation
        if (gamepad1.backWasPressed()) {
            follower.setPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    0  // Reset heading to 0
            ));
        }

        // Start button: Toggle field-centric mode
        if (gamepad1.startWasPressed()) {
            fieldCentricMode = !fieldCentricMode;
        }

//        // Back button: Toggle debug telemetry
//        if (gamepad1.backWasPressed()) {
//            showDebugTelemetry = !showDebugTelemetry;
//        }
    }

    // ════════════════════════════════════════════════════════════════════════
    // OPERATOR CONTROLS (GAMEPAD 2)
    // ════════════════════════════════════════════════════════════════════════

    private void handleOperatorControls() {
        handleOperatorTurretControls();
        handleOperatorLauncherControls();
        handleOperatorConfigControls();
    }

    /**
     * Operator turret controls (gamepad2)
     *
     * Priority order:
     * 1. A button (snap to home) - overrides everything
     * 2. Right stick manual control - overrides auto-aim
     * 3. D-pad manual nudges - overrides auto-aim
     * 4. Right bumper (auto-aim) - lowest priority
     */
    private void handleOperatorTurretControls() {
        // Read control inputs
        double manualInput = gamepad2.right_stick_x;  // Right stick for manual control
        boolean autoAimButton = gamepad2.right_bumper;  // Right bumper for auto-aim
        boolean snapHomeButton = gamepad2.a;  // A button snaps to home
        boolean resetHomeButton = gamepad2.back;  // Back button also snaps home

        // PRIORITY 1: Snap to home (overrides everything)
        if (snapHomeButton || resetHomeButton) {
            robot.turret.snapToHome();
        }
        // PRIORITY 2: Manual stick control (overrides auto-aim)
        else if (Math.abs(manualInput) > 0.05) {
            robot.turret.moveManual(manualInput);
        }
        // PRIORITY 3: D-pad manual nudges (overrides auto-aim)
        else if (gamepad2.dpad_right) {
            robot.turret.moveManual(0.003);  // Nudge right
        }
        else if (gamepad2.dpad_left) {
            robot.turret.moveManual(-0.003);  // Nudge left
        }
        // PRIORITY 4: Auto-aim when button held
        else if (autoAimButton) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double tx = result.getTx();  // Horizontal offset in degrees
                boolean targetValid = result.isValid();
                robot.turret.autoAim(tx, targetValid);
            }
        }
        // Otherwise: Turret holds position (servo does this automatically)
    }

    /**
     * Operator launcher controls
     */
    private void handleOperatorLauncherControls() {
        // Manual indexer control (bypasses automated sequence)
        if (gamepad2.right_trigger > 0.8) {
            robot.indexer.start();
        } else if (gamepad2.left_trigger > 0.8) {
            robot.indexer.setPower(-0.25);
        } else if (gamepad2.left_bumper) {
            robot.indexer.setPower(0.25);
        } else {
            robot.indexer.stop();
        }

        // ----- LAUNCH SEQUENCE (Right Trigger) -----
        // Use edge detection to trigger one shot per button press
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.8;
        boolean shootCommand = rightTriggerPressed && !lastRightTriggerPressed;

        if (shootCommand && !robot.isLaunchSequenceBusy()) {
            // Start a new launch sequence
            robot.launchSequence(true, requestedMotorVelocity);
        } else if (robot.isLaunchSequenceBusy()) {
            // Continue running the sequence
            robot.launchSequence(false, requestedMotorVelocity);
        }

        lastRightTriggerPressed = rightTriggerPressed;

        // ----- VELOCITY MODE SELECTION -----

        // Y button: Switch to AUTO_TARGETING mode
        if (gamepad2.yWasPressed()) {
            launcherMode = LauncherMode.AUTO_TARGETING;
        }

        // Any manual velocity button: Switch to MANUAL_OVERRIDE mode
        if (gamepad2.a || gamepad2.b || gamepad2.x ||
                gamepad2.dpad_up || gamepad2.dpad_down) {
            launcherMode = LauncherMode.MANUAL_OVERRIDE;
        }

        // ----- UPDATE VELOCITY BASED ON MODE -----

        switch (launcherMode) {
            case AUTO_TARGETING:
                // Continuously update velocity from vision/sensors
                requestedMotorVelocity = robot.updateAndGetTargetVelocity();
                break;

            case MANUAL_OVERRIDE:
                // A: Near shot preset
                if (gamepad2.a) {
                    requestedMotorVelocity = P3RobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
                }
                // B: Far shot preset
                else if (gamepad2.b) {
                    requestedMotorVelocity = P3RobotConstants.Launcher.FAR_TARGET_VELOCITY;
                }
                // X: Stop flywheels
                else if (gamepad2.x) {
                    requestedMotorVelocity = 0;
                    robot.stopLaunchSequence();  // Also abort any active sequence
                }
                // D-Pad Up: Increase velocity
                else if (gamepad2.dpad_up) {
                    requestedMotorVelocity += 100;
                    requestedMotorVelocity = Math.min(requestedMotorVelocity, 2200);  // Cap at reasonable max
                }
                // D-Pad Down: Decrease velocity
                else if (gamepad2.dpad_down) {
                    requestedMotorVelocity -= 100;
                    requestedMotorVelocity = Math.max(requestedMotorVelocity, 0);  // Don't go negative
                }
                break;
        }

        // ----- APPLY VELOCITY TO FLYWHEELS -----
        // Only set velocity if not in an active launch sequence
        if (!robot.isLaunchSequenceBusy()) {
            robot.launcher.setShooterMotorVelocity(requestedMotorVelocity);
        }
    }

    /**
     * Operator configuration controls
     */
    private void handleOperatorConfigControls() {
        // Start button: Toggle flywheel spin mode
        if (gamepad2.start) {
            boolean newMode = !robot.isKeepFlywheelsSpinning();
            robot.setKeepFlywheelsSpinning(newMode);

            String modeName = newMode ? "RAPID-FIRE" : "BATTERY-SAVER";
            telemetry.addData("⚙️ Spin Mode", modeName);
        }
    }

    // ════════════════════════════════════════════════════════════════════════
    // SYSTEM UTILITIES
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Emergency stop all robot systems
     */
    private void emergencyStopAllSystems() {
        robot.stopAll();
        intakeState = IntakeState.OFF;
        requestedMotorVelocity = 0;
    }

    // ════════════════════════════════════════════════════════════════════════
    // TELEMETRY DISPLAY
    // ════════════════════════════════════════════════════════════════════════

    private void displayTelemetry() {
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("  P3 PEDROPATH + TURRET TELEOP");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addData("⏱️ Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("🤝 Alliance", alliance);

        // ----- DRIVE STATUS -----
        telemetry.addLine();
        telemetry.addLine("─── DRIVE (GP1) ───");
        telemetry.addData("Mode", fieldCentricMode ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Speed", "%.0f%%", driveSpeedMultiplier * 100);

        Pose currentPose = follower.getPose();
        telemetry.addData("Position", "X: %.1f, Y: %.1f",
                currentPose.getX(), currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("📦 Intake", intakeState);

        // ----- OPERATOR STATUS -----
        telemetry.addLine();
        telemetry.addLine("─── OPERATOR (GP2) ───");

        // Turret status
        telemetry.addData("🎯 Turret", robot.turret.getStatusString());
        telemetry.addData("Home Switch", robot.turret.isHomeSwitchTriggered() ? "TRIGGERED" : "Clear");

        // Show if auto-aiming
        if (gamepad2.right_bumper) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Auto-Aim", "ACTIVE - Error: %.1f°", result.getTx());
            } else {
                telemetry.addData("Auto-Aim", "NO TARGET");
            }
        }

        // Launcher
        telemetry.addData("🚀 Launcher Mode", launcherMode == LauncherMode.AUTO_TARGETING ? "AUTO" : "MANUAL");
        telemetry.addData("Target Velocity", "%.0f", requestedMotorVelocity);
        telemetry.addData("Actual Velocity", "%.0f", robot.getCurrentFlywheelVelocity());

        if (requestedMotorVelocity > 100) {
            double percentOfTarget = (robot.getCurrentFlywheelVelocity() / requestedMotorVelocity) * 100.0;
            telemetry.addData("Velocity %", "%.1f%%", percentOfTarget);

            if (robot.areFlywheelsReady(requestedMotorVelocity)) {
                telemetry.addData("Status", "✓ READY");
            } else {
                telemetry.addData("Status", "⏳ Spinning...");
            }
        }

        // Launch sequence
        if (robot.isLaunchSequenceBusy()) {
            telemetry.addLine("🔄 LAUNCH SEQUENCE ACTIVE");
        }

        // ----- SHOT STATISTICS -----
        telemetry.addLine();
        telemetry.addLine("─── STATISTICS ───");
        telemetry.addData("📊 Shots", "%d / %d", robot.getShotsFired(), robot.getShotsAttempted());
        if (robot.getShotsAborted() > 0) {
            telemetry.addData("⚠️ Aborted", robot.getShotsAborted());
        }
        telemetry.addData("Success Rate", "%.1f%%", robot.getLaunchSuccessRate());

        // ----- CONFIGURATION -----
        telemetry.addLine();
        String spinMode = robot.isKeepFlywheelsSpinning() ? "RAPID-FIRE" : "BATTERY-SAVER";
        telemetry.addData("⚙️ Spin Mode", spinMode);

        // ----- VISION -----
        if (showDebugTelemetry) {
            telemetry.addLine();
            telemetry.addLine("─── VISION DEBUG ───");
            robot.vision.addTelemetry();

            telemetry.addLine();
            telemetry.addLine("─── TURRET DEBUG ───");
            telemetry.addData("Manual Call Count", robot.turret.getManualCallCount());
            telemetry.addData("Last Manual Input", "%.3f", robot.turret.getLastManualInput());
            telemetry.addData("Right Stick X", "%.3f", gamepad2.right_stick_x);
            telemetry.addLine("Soft Limits:");
            telemetry.addLine(robot.turret.getLimitStatus());
        } else if (robot.vision.isTargetVisible()) {
            telemetry.addLine();
            telemetry.addData("👁️ Vision", "Target %.1f° @ %.1f\"",
                    robot.vision.getTargetAngleX(),
                    robot.vision.getDistanceToTagInches());
        }

        // ----- QUICK REFERENCE -----
        if (!showDebugTelemetry) {
            telemetry.addLine();
            telemetry.addLine("─── QUICK REFERENCE ───");
            telemetry.addData("GP1 Y", "Reset field heading");
            telemetry.addData("GP1 Start", "Toggle field/robot centric");
            telemetry.addData("GP2 RT", "Launch");
            telemetry.addData("GP2 R-Bumper", "Auto-aim turret");
            telemetry.addData("GP2 A", "Home turret");
            telemetry.addData("GP1 Back", "Toggle debug");
        }

        // ----- DEBUG TELEMETRY (if enabled) -----
        if (showDebugTelemetry) {
            telemetry.addLine();
            telemetry.addLine("═══ DEBUG MODE ═══");
            robot.addLaunchDebugTelemetry();

            // PedroPath telemetry
            telemetryM.debug("Follower Pose", follower.getPose());
            telemetryM.debug("Follower Velocity", follower.getVelocity());
        }
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (limelight != null) {
            limelight.stop();
        }
    }
}