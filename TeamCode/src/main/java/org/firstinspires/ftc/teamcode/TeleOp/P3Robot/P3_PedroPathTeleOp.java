package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
 * P3 Robot TeleOp with PedroPathing Field-Oriented Drive
 *
 * This OpMode combines PedroPathing's superior field-oriented drive control
 * with the P3_Robot3 class that manages all subsystems and the launch sequence.
 *
 * Uses RobotConfig for multi-robot compatibility - simply change the config
 * in init() to run on different robots with different tuning.
 *
 * KEY FEATURES:
 * - Field-oriented drive using PedroPathing (maintains orientation relative to field)
 * - Dual operator control (Driver + Operator)
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
 * Left Stick X:      Manual turret rotation (left/right)
 * Left Stick Press:  Enable turret auto-aim to vision target
 *
 * Right Trigger:     LAUNCH - Initiate automated shot sequence
 * Left Trigger:      Manual reverse indexer (unjam)
 *
 * A:                 Set NEAR shot velocity preset (2500 ticks/sec)
 * B:                 Set FAR shot velocity preset (3500 ticks/sec)
 * X:                 Stop flywheels (manual override)
 * Y:                 Enable AUTO-TARGETING mode (vision-based velocity)
 *
 * D-Pad Up:          Increase velocity by 100 ticks/sec
 * D-Pad Down:        Decrease velocity by 100 ticks/sec
 * D-Pad Left:        Turret rotate to left preset position
 * D-Pad Right:       Turret rotate to right preset position
 *
 * Left Bumper:       Decrease turret speed (fine control)
 * Right Bumper:      Increase turret speed (coarse control)
 *
 * Start:             Toggle flywheel spin mode (rapid-fire / battery-saver)
 * Back:              Reset turret to center position
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

@Configurable
@TeleOp(name="P3: PedroPath Field-Oriented TeleOp", group="_P3opmodes")
public class P3_PedroPathTeleOp extends OpMode {

    // ========================================
    // ROBOT CONFIGURATION
    // ========================================
    /**
     * CHANGE THIS to switch between robots!
     *
     * Available configurations:
     * - RobotConfig.createP3Robot2Config()       // P3 Robot #2 (current)
     * - RobotConfig.createDefaultP3Config()      // P3 Robot #1
     * - RobotConfig.createDefaultGearGirlsConfig() // Gear Girls robot
     *
     * Each config contains robot-specific tuning for:
     * - Motor directions
     * - Odometry sensor offsets
     * - IMU orientation
     * - PedroPathing PIDF coefficients
     * - Drive velocities
     */
    private static final RobotConfig ROBOT_CONFIG = RobotConfig.createP3Robot2Config();

    // ========================================
    // CONSTANTS - Drive System
    // ========================================
    // Drive speeds are defined in P3RobotConstants.TeleOp

    // ========================================
    // CONSTANTS - Subsystems
    // ========================================
    // All subsystem constants are defined in P3RobotConstants

    // ========================================
    // ROBOT SYSTEMS
    // ========================================
    private ElapsedTime runtime = new ElapsedTime();

    // PedroPathing
    private Follower follower;
    private TelemetryManager telemetryM;
    public static Pose startingPose = new Pose(0, 0, 0);

    // P3 Robot (contains all subsystems)
    private P3_Robot3 robot;

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

    private enum TurretMode {
        MANUAL,           // Direct joystick control
        AUTO_AIM,         // Vision-based auto-aiming
        PRESET_POSITION   // Moving to preset position
    }
    private TurretMode turretMode = TurretMode.MANUAL;

    // ========================================
    // WORKING VARIABLES
    // ========================================
    private boolean showDebugTelemetry = false;
    private double requestedMotorVelocity = 0;
    private double turretSpeed = P3RobotConstants.Turret.MANUAL_SPEED;
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;

    // Button state tracking
    private boolean lastRightTriggerPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize PedroPathing using RobotConfig
        // This is the key: Constants.createFollower() can accept a RobotConfig
        follower = Constants.createFollower(hardwareMap, ROBOT_CONFIG);
        follower.setStartingPose(startingPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize P3 Robot (this creates all subsystems)
        robot = new P3_Robot3(hardwareMap, telemetry);

        // Configure launch system for competition
        robot.setVelocityTolerance(0.97);           // Require 97% of target speed
        robot.setStallDetectionThreshold(0.80);     // Abort if velocity drops below 80%
        robot.setKeepFlywheelsSpinning(true);       // Default to rapid-fire mode
        robot.resetShotCounters();                  // Start fresh

        // Load alliance from autonomous (or default to RED)
        alliance = SharedState.alliance != null ? SharedState.alliance : CommonConstants.Alliance.RED;
        robot.configureVisionForTeleOp(alliance);

        telemetry.addData("Status", "Initialized - PEDRO PATH FIELD-ORIENTED MODE");
        telemetry.addData("Robot Config", ROBOT_CONFIG == RobotConfig.createP3Robot2Config() ? "P3 Robot #2" : "Other");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive Mode", "Field-Centric");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();

        // Start PedroPathing in TeleOp mode
        // Uses brake mode for precise control
        follower.startTeleopDrive();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START
     */
    @Override
    public void loop() {
        // Update all systems
        follower.update();
        robot.update();  // This updates vision, turret, LEDs, etc.
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

    /**
     * Field-oriented drive using PedroPathing
     */
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

        // Add fine adjustments from D-pad
        if (gamepad1.dpad_up) {
            forward += P3RobotConstants.TeleOp.FINE_ADJUSTMENT_SPEED;
        }
        if (gamepad1.dpad_down) {
            forward -= P3RobotConstants.TeleOp.FINE_ADJUSTMENT_SPEED;
        }
        if (gamepad1.dpad_left) {
            strafe -= P3RobotConstants.TeleOp.FINE_ADJUSTMENT_SPEED;
        }
        if (gamepad1.dpad_right) {
            strafe += P3RobotConstants.TeleOp.FINE_ADJUSTMENT_SPEED;
        }

        // Apply drive command with field-centric mode
        follower.setTeleOpDrive(
                forward,
                strafe,
                turn,
                fieldCentricMode  // invert: true = robot-centric, false = field-centric
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
                //robot.launcher.setIndexerServoPower(0.0);
                robot.intake.stopIntakeServos();
            } else {
                intakeState = IntakeState.ON;
                robot.intake.setIntakePower(P3RobotConstants.Intake.INTAKE_SPEED);
                //robot.launcher.setIndexerServoPower(-1.0);
                robot.intake.setIntakeServos();
            }
        }

        // B button: Toggle intake REVERSE/OFF
        if (gamepad1.bWasPressed()) {
            if (intakeState == IntakeState.REVERSE) {
                intakeState = IntakeState.OFF;
                robot.intake.stopIntake();
                //robot.launcher.setIndexerServoPower(0.0);
                robot.intake.stopIntakeServos();
            } else {
                intakeState = IntakeState.REVERSE;
                robot.intake.setIntakePower(P3RobotConstants.Intake.OUTTAKE_SPEED);
                //robot.launcher.setIndexerServoPower(1.0);
                robot.intake.reverseIntakeServos();
            }
        }
    }

    /**
     * Driver system controls
     */
    private void handleDriverSystemControls() {
        // X button: Emergency stop all systems
        if (gamepad1.xWasPressed()) {
            emergencyStopAllSystems();
        }

        // Y button: Reset field orientation
        if (gamepad1.yWasPressed()) {
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

        // Back button: Toggle debug telemetry
        if (gamepad1.backWasPressed()) {
            showDebugTelemetry = !showDebugTelemetry;
        }
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
     * Operator turret controls
     */
    private void handleOperatorTurretControls() {


//        // Left stick: Manual turret control
//        double turretJoystick = gamepad2.left_stick_x;
//
//        // Left stick press: Toggle auto-aim
//        if (gamepad2.left_stick_button && !gamepad2.leftStickButtonWasPressed()) {
//            if (turretMode == TurretMode.AUTO_AIM) {
//                turretMode = TurretMode.MANUAL;
//                robot.turret.holdPosition();
//            } else {
//                turretMode = TurretMode.AUTO_AIM;
//            }
//            gamepad2.rumble(100);
//        }
//
//        // D-pad left/right: Preset positions
//        if (gamepad2.dpad_left) {
//            turretMode = TurretMode.PRESET_POSITION;
//            robot.turret.setTargetAngle(P3RobotConstants.Turret.PRESET_LEFT);
//        } else if (gamepad2.dpad_right) {
//            turretMode = TurretMode.PRESET_POSITION;
//            robot.turret.setTargetAngle(P3RobotConstants.Turret.PRESET_RIGHT);
//        }
//
//        // Bumpers: Adjust turret speed
//        if (gamepad2.left_bumper) {
//            turretSpeed = P3RobotConstants.Turret.FINE_SPEED;
//        } else if (gamepad2.right_bumper) {
//            turretSpeed = P3RobotConstants.Turret.COARSE_SPEED;
//        } else {
//            turretSpeed = P3RobotConstants.Turret.MANUAL_SPEED;
//        }
//
//        // Back button: Return turret to home
//        if (gamepad2.backWasPressed()) {
//            robot.turret.returnToHome();
//            turretMode = TurretMode.PRESET_POSITION;
//            gamepad2.rumble(100);
//        }
//
//        // Execute turret control based on mode
//        switch (turretMode) {
//            case MANUAL:
//                if (Math.abs(turretJoystick) > 0.1) {
//                    robot.turret.setTurretPower(turretJoystick * turretSpeed);
//                } else {
//                    robot.turret.setTurretPower(0);
//                }
//                break;
//
//            case AUTO_AIM:
//                if (robot.vision.isTargetVisible()) {
//                    double txError = robot.vision.getTargetAngleX();
//                    double turretAdjustment = txError * P3RobotConstants.Turret.AUTO_AIM_KP;
//                    robot.turret.setTargetAngle(robot.turret.getCurrentAngle() + turretAdjustment);
//
//                    // Provide haptic feedback when on target
//                    if (Math.abs(txError) <= P3RobotConstants.Turret.AIM_TOLERANCE_DEG) {
//                        if (!gamepad2.isRumbling()) {
//                            gamepad2.rumble(50);
//                        }
//                    }
//                } else {
//                    // Lost vision target - hold position
//                    robot.turret.holdPosition();
//                }
//                break;
//
//            case PRESET_POSITION:
//                // Turret automatically moves to preset - check if arrived
//                if (robot.turret.isAtTarget()) {
//                    turretMode = TurretMode.MANUAL;
//                }
//                break;
//        }
    }

    /**
     * Operator launcher controls
     */
    private void handleOperatorLauncherControls() {
        if (gamepad2.left_bumper) {
            robot.indexer.start();;
        } else if (gamepad2.right_bumper) {
            robot.indexer.reverse();
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
            boolean shotComplete = robot.launchSequence(false, requestedMotorVelocity);
            if (shotComplete) {
            }
        }

        lastRightTriggerPressed = rightTriggerPressed;

        // ----- MANUAL INDEXER CONTROL (Left Trigger) -----
        // This bypasses the automated sequence for manual feeding/unjamming
        if (gamepad2.left_trigger > 0.8) {
            // Manual reverse - useful for clearing jams
            //robot.launcher.setIndexerServoPower(1.0);
            //robot.intake.reverseIntakeServos();
        } else if (!robot.isLaunchSequenceBusy()) {
            // Only stop indexer if launch sequence isn't running
           // robot.launcher.setIndexerServoPower(0.0);
            //robot.intake.stopIntakeServos();
        }

        // ----- VELOCITY MODE SELECTION -----

        // Y button: Switch to AUTO_TARGETING mode
        if (gamepad2.yWasPressed()) {
            launcherMode = LauncherMode.AUTO_TARGETING;
        }

        // Any manual velocity button: Switch to MANUAL_OVERRIDE mode
        if (gamepad2.a || gamepad2.b || gamepad2.x ||
                gamepad2.dpadUpWasPressed() || gamepad2.dpadDownWasPressed()) {
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
                if (gamepad2.aWasPressed()) {
                    requestedMotorVelocity = P3RobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
                }
                // B: Far shot preset
                else if (gamepad2.bWasPressed()) {
                    requestedMotorVelocity = P3RobotConstants.Launcher.FAR_TARGET_VELOCITY;
                }
                // X: Stop flywheels
                else if (gamepad2.x) {
                    requestedMotorVelocity = 0;
                    robot.stopLaunchSequence();  // Also abort any active sequence
                }
                // D-Pad Up: Increase velocity
                else if (gamepad2.dpadUpWasReleased()) {
                    requestedMotorVelocity += 100;
                    requestedMotorVelocity = Math.min(requestedMotorVelocity, 2000);  // Cap at reasonable max
                }
                // D-Pad Down: Decrease velocity
                else if (gamepad2.dpadDownWasReleased()) {
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
        if (gamepad2.startWasPressed()) {
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
        telemetry.addLine("  P3 PEDROPATH FIELD-ORIENTED TELEOP");
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

//        // Turret
//        telemetry.addData("🎯 Turret Mode", turretMode);
//        telemetry.addData("Turret Angle", "%.1f° → %.1f°",
//                robot.turret.getCurrentAngle(),
//                robot.turret.getTargetAngle());
//
//        if (turretMode == TurretMode.AUTO_AIM && robot.vision.isTargetVisible()) {
//            double txError = robot.vision.getTargetAngleX();
//            telemetry.addData("Aim Error", "%.1f°", txError);
//            if (Math.abs(txError) <= P3RobotConstants.Turret.AIM_TOLERANCE_DEG) {
//                telemetry.addData("Status", "✓ ON TARGET");
//            }
//        }

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
        } else if (robot.vision.isTargetVisible()) {
            telemetry.addLine();
            telemetry.addData("👁️ Vision", "Target %.1f° @ %.1f\"",
                    robot.vision.getTargetAngleX(),
                    robot.vision.getDistanceToTagInches());
        }

        // ----- WARNINGS -----
//        if (robot.turret.wasAutoStopTriggered()) {
//            telemetry.addLine();
//            telemetry.addData("🔴 TURRET FAULT", "AUTO-STOP!");
//        } else if (robot.turret.isJammed()) {
//            telemetry.addLine();
//            telemetry.addData("⚠️ WARNING", "Turret jam detected");
//        }

        // ----- QUICK REFERENCE -----
        if (!showDebugTelemetry) {
            telemetry.addLine();
            telemetry.addLine("─── QUICK REFERENCE ───");
            telemetry.addData("GP1 Y", "Reset field heading");
            telemetry.addData("GP1 Start", "Toggle field/robot centric");
            telemetry.addData("GP2 RT", "Launch");
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
    }
}