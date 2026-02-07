package org.firstinspires.ftc.teamcode.TeleOp.GGRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot2;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeSensorFusion002;

import java.util.List;

/**
 * This class implements the TeleOp (driver-controlled) program for the "Gear Girls Bot 2" robot
 * for the FTC DECODE season. It is designed to be the primary program for controlling the robot
 * during a match.
 *
 * Control Scheme:
 * The robot's functions are mapped to Gamepad 1 for the primary driver and Gamepad 2 for copilot.
 *
 * ========================================
 * GAMEPAD 1 - DRIVER CONTROLS
 * ========================================
 *
 * DRIVETRAIN:
 * - Left Stick: Strafe (X) and Drive (Y)
 * - Right Stick X: Turn/Rotate
 * - Right Stick Button (R3): Enable auto-aim (snap to target)
 * - BACK Button: Toggle between Arcade and Field-Centric drive modes
 * - START Button: Reset odometry position and IMU
 *
 * INTAKE SYSTEM:
 * - Button A: Toggle intake ON/OFF (collecting)
 * - Button X: Toggle intake REVERSE/OFF (expelling)
 *
 * LAUNCHER TARGETING (Multi-Tiered):
 * - Button Y: Start flywheels (uses current targeting mode)
 * - Button B: Stop flywheels
 * - DPAD LEFT: Cycle targeting modes (AUTO → PRESET → MANUAL → AUTO)
 * - DPAD RIGHT: Toggle CLOSE ↔ FAR preset (auto-switches to PRESET mode)
 * - DPAD UP: Manual velocity +25 ticks/sec (auto-switches to MANUAL mode)
 * - DPAD DOWN: Manual velocity -25 ticks/sec (auto-switches to MANUAL mode)
 *
 * SPINNER/INDEXER:
 * - LEFT TRIGGER: Rotate spinner LEFT
 * - RIGHT TRIGGER: Rotate spinner RIGHT
 *
 * LAUNCH SYSTEM:
 * - LEFT BUMPER: Fire left flipper (only when at speed)
 * - RIGHT BUMPER: Fire right flipper (only when at speed)
 *
 * ========================================
 * GAMEPAD 2 - COPILOT/COACH CONTROLS
 * ========================================
 * - Button X: Force BLUE alliance (reconfigures vision)
 * - Button B: Force RED alliance (reconfigures vision)
 * - START Button: Reset odometry to vision AprilTag position
 *
 * The main loop() method delegates control logic to a series of private methods
 * (e.g., handleDriveControls(), handleLauncherControls()), each responsible for one
 * part of the robot. This keeps the main loop clean and organized. Real-time data is sent to the
 * Driver Station via the displayTelemetry() method for debugging and monitoring.
 *
 * @version 2.4 - Refactored to match Bot1 structure
 * @author GearGirls Team
 */
@TeleOp(name = "Gear Girls Bot 2 (test)", group = " _GGopmodes")
@Disabled
public class GearGirlsBot2_test extends OpMode {

    //Declare SubSystems
    private GGRobot2 robot;

    // --- Launcher System State Variables ---
    // --- Multi-Tiered Launcher Control ---
    private enum LauncherTargetingMode {
        AUTO,      // Vision/odometry based (uses robot.updateLauncher)
        PRESET,    // CLOSE or FAR preset distances
        MANUAL     // Manual ticks/sec adjustment
    }
    //private LauncherTargetingMode targetingMode = LauncherTargetingMode.AUTO;
    private double manualTargetVelocity = 1800; // For MANUAL mode only
    private GGRobotConstants.LauncherDistance launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;
    private GGRobotConstants.LauncherSystemState launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
    private GGRobotConstants.LauncherTargetingMode targetingMode = GGRobotConstants.LauncherTargetingMode.AUTO;
    private double finalTargetVelocity = 0; // Actual commanded velocity
    private boolean flywheelsRunning = false;

    // --- Drive Control Constants ---
    private static final double JOYSTICK_DEADBAND = 0.03;
    private static final double TX_ALIGN_KP = 0.04; // Proportional gain for tx alignment
    private static final double TX_ALIGN_TOLERANCE_DEG = 1.0; // 1-degree tolerance
    private double angleOnTarget = 0.0;

    // --- Launcher Constants ---
    private static final double VELOCITY_INCREMENT = 25; // 25 ticks/sec increments for manual mode

    // --- Drive Mode ---
    private enum DriveMode {
        FIELD_CENTRIC,
        ARCADE
    }
    private DriveMode driveMode = DriveMode.ARCADE;

    // --- Intake State ---
    private enum IntakeState {
        ON,
        OFF,
        REVERSE
    }
    private IntakeState intakeState = IntakeState.OFF;

    private final ElapsedTime loopTimer = new ElapsedTime();

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // --- ROBOT ---
        robot = new GGRobot2(hardwareMap, telemetry);

        // Load the alliance that was saved by the Autonomous OpMode
        CommonConstants.Alliance alliance = SharedState.alliance; // This loads the value into the static variable.
        // Tell the robot to configure its vision system for that alliance.
        robot.configureVisionForTeleOp(alliance);

        // --- State Initialization ---
        launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
        intakeState = IntakeState.OFF;
        driveMode = DriveMode.ARCADE;
        targetingMode = GGRobotConstants.LauncherTargetingMode.AUTO;
        launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;
        flywheelsRunning = false;

        loopTimer.reset(); // Start the timer in init

        // --- Tell the driver that initialization is complete.---
        telemetry.addLine("=== GEARGIRLSBOT2 ===");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Targeting Mode", targetingMode);
        telemetry.addData("Drive Mode", driveMode);
        telemetry.addLine("Intake Sensor Fusion: Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        loopTimer.reset();
    }

    /**
     * This method is the primary execution loop for the TeleOp mode.
     * It is called repeatedly by the FTC SDK after the "Start" button is pressed on the
     * Driver Station and before "Stop" is pressed.
     *
     * The loop orchestrates all robot actions by calling a series of dedicated handler methods for
     * each major subsystem:
     * - {@code robot.update()}: Updates the state of all robot components, such as reading sensor
     *   values and checking timers.
     * - {@code handleDriveControls()}: Translates gamepad stick inputs into drivetrain movement.
     * - {@code handleIntakeControls()}: Controls the intake motors (on, off, reverse).
     * - {@code handleSpinnerControls()}: Manages the spinner/indexer mechanism.
     * - {@code handleLauncherControls()}: Manages the launcher flywheels and the firing mechanism.
     * - {@code handleCopilotControls()}: Processes copilot/coach inputs from gamepad 2.
     * - {@code displayTelemetry()}: Sends real-time data back to the Driver Station for monitoring.
     *
     * This modular approach keeps the main loop clean and easy to understand, delegating the
     * complex logic for each function to its respective method.
     */
    @Override
    public void loop() {
        // Update robot state (CRITICAL - must be first)
        robot.update();

        // Handle all control inputs
        handleDriveControls();
        handleIntakeControls();
        handleSpinnerControls();
        handleLauncherControls();
        handleCopilotControls();

        // Display comprehensive telemetry
        displayTelemetry();
    }

    /**
     * Handles drivetrain controls with auto-aim functionality.
     *
     * This method processes driver inputs from gamepad 1 to control robot movement:
     * - Left stick controls forward/backward and strafing movement
     * - Right stick controls rotation
     * - Right stick button (R3) enables auto-aim to snap to vision targets
     * - START button resets odometry and IMU
     * - BACK button toggles between Arcade and Field-Centric drive modes
     *
     * The auto-aim feature uses proportional control to automatically align the robot
     * with vision targets when the right stick button is pressed and a target is visible.
     */
    private void handleDriveControls() {
        // START button resets odometry and IMU
        if (gamepad1.start) {
            robot.drive.pinpoint.resetPosAndIMU();
            telemetry.addLine("⚠ ODOMETRY RESET");
        }

        // BACK button toggles drive mode
        if (gamepad1.back) {
            toggleDriveMode();
        }

        // Read joystick inputs with deadband
        double driveInput = applyDeadband(-gamepad1.left_stick_y);
        double strafeInput = applyDeadband(gamepad1.left_stick_x);
        double turnInput = applyDeadband(gamepad1.right_stick_x);

        // Auto-aim: Right stick button enables snap-to-target
        boolean isSnappingToTarget = gamepad1.right_stick_button && robot.vision.isTargetVisible();

        if (isSnappingToTarget) {
            // Get the angle error from the Limelight
            double txError = robot.vision.getTargetAngleX();

            // Calculate turn command using proportional controller
            if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
                // Within tolerance - hold position
                angleOnTarget = 0.0;
            } else {
                // Not aligned - calculate turn power
                angleOnTarget = TX_ALIGN_KP * txError;
            }

            // Override manual turn input with auto-aim
            turnInput = angleOnTarget;
        }

        // Apply drive mode
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            robot.drive.arcadeDrive(strafeInput, driveInput, turnInput, 0, 1.0);
        } else {
            robot.drive.arcadeDrive(strafeInput, driveInput, turnInput, 0, 1.0);
        }
    }

    /**
     * Toggles between Arcade and Field-Centric drive modes.
     * Provides haptic feedback to the driver via gamepad rumble.
     */
    private void toggleDriveMode() {
        if (driveMode == DriveMode.ARCADE) {
            driveMode = DriveMode.FIELD_CENTRIC;
            gamepad1.rumble(200); // Short rumble for field-centric
        } else {
            driveMode = DriveMode.ARCADE;
            gamepad1.rumble(100); // Quick rumble for arcade
        }
    }

    /**
     * Manages the intake system based on gamepad input.
     *
     * The intake can be in three states:
     * - ON: Collecting game elements (forward motion)
     * - REVERSE: Expelling game elements (backward motion)
     * - OFF: Intake motors stopped
     *
     * Controls:
     * - Button A: Toggles between ON and OFF states
     * - Button X: Toggles between REVERSE and OFF states
     *
     * The method updates the {@code intakeState} variable and commands the intake motors
     * accordingly. This state-based approach prevents conflicting commands and makes the
     * intake behavior predictable for the driver.
     */
    private void handleIntakeControls() {
        // Button A toggles between ON and OFF
        if (gamepad1.aWasPressed()) {
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }

        // Button X toggles between REVERSE and OFF
        if (gamepad1.xWasPressed()) {
            intakeState = (intakeState == IntakeState.REVERSE) ? IntakeState.OFF : IntakeState.REVERSE;
        }

        // Command the motors based on current state
        switch (intakeState) {
            case ON:
                robot.intake.setIntakeMotorPower(GGRobotConstants.Intake.INTAKE_SPEED);
                break;
            case REVERSE:
                robot.intake.setIntakeMotorPower(GGRobotConstants.Intake.OUTTAKE_SPEED);
                break;
            case OFF:
                robot.intake.setIntakeMotorPower(0);
                break;
        }
    }

    /**
     * Manages the spinner/indexer mechanism based on gamepad input.
     *
     * The spinner is used to rotate game elements in the storage area to position
     * them for launching. This allows the driver to select which game element to launch next.
     *
     * Controls:
     * - Left Trigger: Rotate spinner to the left (counterclockwise)
     * - Right Trigger: Rotate spinner to the right (clockwise)
     *
     * The spinner commands are sent directly to the robot subsystem, which handles
     * the timing and state management of the rotation sequence.
     */
    private void handleSpinnerControls() {
        // Left trigger rotates spinner left
        if (gamepad1.left_trigger > 0.5) {
            robot.rotateSpinnerLeft();
        }

        // Right trigger rotates spinner right
        if (gamepad1.right_trigger > 0.5) {
            robot.rotateSpinnerRight();
        }
    }

    /**
     * Manages all driver controls for the launcher system.
     *
     * This method handles a multi-tiered targeting system with three modes:
     *
     * AUTO Mode: Uses vision and odometry to automatically calculate optimal launch velocity
     * PRESET Mode: Uses pre-defined velocities for CLOSE or FAR distances
     * MANUAL Mode: Allows direct velocity control via D-pad up/down
     *
     * This method handles the following actions based on gamepad 1 input:
     *
     *   Y Button: Spins up the launcher flywheels to the currently selected target velocity
     *   B Button: Stops the launcher flywheels
     *   D-pad Left: Cycles through targeting modes (AUTO → PRESET → MANUAL → AUTO)
     *   D-pad Right: Toggles between CLOSE and FAR presets (auto-switches to PRESET mode)
     *   D-pad Up: Increases manual velocity by 25 ticks/sec (auto-switches to MANUAL mode)
     *   D-pad Down: Decreases manual velocity by 25 ticks/sec (auto-switches to MANUAL mode)
     *   Left/Right Triggers: Fire the left/right flippers when launcher is at speed
     *
     * The method also includes safety checks to prevent firing when flywheels are not
     * at the correct speed, and an emergency stop feature when both triggers are pressed.
     */
    private void handleLauncherControls() {
        // --- STEP 1: HANDLE DRIVER INPUTS TO CHANGE STATES AND VALUES ---

        // Press D-Pad Left to cycle between AUTO, PRESET, and MANUAL targeting modes
        if (gamepad1.dpadLeftWasPressed()) {
            cycleTargetingMode();
        }

        // D-pad Right toggles between CLOSE and FAR, automatically switches to PRESET mode
        if (gamepad1.dpadRightWasPressed()) {
            launcherDistance = (launcherDistance == GGRobotConstants.LauncherDistance.CLOSE) ?
                    GGRobotConstants.LauncherDistance.FAR : GGRobotConstants.LauncherDistance.CLOSE;
            targetingMode = GGRobotConstants.LauncherTargetingMode.PRESET;
        }

        // D-pad Up/Down adjusts manual velocity, automatically switches to MANUAL mode
        if (gamepad1.dpadUpWasPressed()) {
            manualTargetVelocity += VELOCITY_INCREMENT;
            manualTargetVelocity = Range.clip(manualTargetVelocity, 0, 3000);
            targetingMode = GGRobotConstants.LauncherTargetingMode.MANUAL;
        }
        if (gamepad1.dpadDownWasPressed()) {
            manualTargetVelocity -= VELOCITY_INCREMENT;
            manualTargetVelocity = Range.clip(manualTargetVelocity, 0, 3000);
            targetingMode = GGRobotConstants.LauncherTargetingMode.MANUAL;
        }

        // Press 'Y' to activate the launcher, 'B' to deactivate it
        if (gamepad1.yWasPressed()) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.ACTIVE;
            flywheelsRunning = true;
        }
        if (gamepad1.bWasPressed()) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
            flywheelsRunning = false;
        }

        // --- STEP 2: Delegate launcher orchestration to the GGRobot2 class ---
        // The robot calculates the appropriate velocity based on the current mode
        switch (targetingMode) {
            case AUTO:
                // Use vision/odometry to calculate velocity
                finalTargetVelocity = robot.updateLauncher(launcherSystemState, targetingMode, launcherDistance);
                break;
            case PRESET:
                // Use preset distance settings
                finalTargetVelocity = robot.updateLauncher(launcherSystemState, targetingMode, launcherDistance);
                break;
            case MANUAL:
                // Use manual velocity setting
                if (launcherSystemState == GGRobotConstants.LauncherSystemState.ACTIVE) {
                    robot.launcher.setMotorVelocity(manualTargetVelocity, manualTargetVelocity);
                    finalTargetVelocity = manualTargetVelocity;
                }
                break;
        }

        // --- STEP 3: Handle Firing Requests ---
        final double LAUNCHER_VELOCITY_TOLERANCE = 25.0;
        final double MINIMUM_SAFE_VELOCITY = 500.0;

        boolean isSpeedCorrect = (robot.launcher.getLeftMotorVelocity() >= (finalTargetVelocity - LAUNCHER_VELOCITY_TOLERANCE));
        boolean isSpeedSafe = (robot.launcher.getLeftMotorVelocity() > MINIMUM_SAFE_VELOCITY);
        boolean isLauncherReady = (launcherSystemState == GGRobotConstants.LauncherSystemState.ACTIVE) && isSpeedCorrect && isSpeedSafe;

        // shoot both Both bumpers pressed
        if (gamepad1.leftBumperWasPressed() && gamepad1.rightBumperWasPressed()) {
            robot.triggerLeftFlipper();
            robot.triggerRightFlipper();
        }
        // Only allow firing if launcher is ready
        else if (isLauncherReady) {
            // Left bumper fires left flipper
            if (gamepad1.leftBumperWasPressed()) {
                robot.triggerLeftFlipper();
            }

            // Right bumper fires right flipper
            if (gamepad1.rightBumperWasPressed()) {
                robot.triggerRightFlipper();
            }
        } else {
            // Visual feedback that launcher isn't ready
            if (gamepad1.leftBumperWasPressed() || gamepad1.rightBumperWasPressed()) {
                telemetry.addLine("⚠ LAUNCHER NOT READY - Wait for speed");
            }
        }
    }

    /**
     * Cycles through the three targeting modes: AUTO → PRESET → MANUAL → AUTO
     * Provides haptic feedback to indicate the mode change.
     */
    private void cycleTargetingMode() {
        switch (targetingMode) {
            case AUTO:
                targetingMode = GGRobotConstants.LauncherTargetingMode.PRESET;
                break;
            case PRESET:
                targetingMode = GGRobotConstants.LauncherTargetingMode.MANUAL;
                break;
            case MANUAL:
                targetingMode = GGRobotConstants.LauncherTargetingMode.AUTO;
                break;
        }
    }

    /**
     * Handles copilot/coach controls on Gamepad 2.
     *
     * These controls allow a coach or copilot to override critical settings during a match:
     * - Alliance color override (useful if autonomous didn't run or alliance color changed)
     * - Vision-based odometry reset (useful for correcting drift during the match)
     *
     * Controls:
     * - Button X: Force BLUE alliance and reconfigure vision system
     * - Button B: Force RED alliance and reconfigure vision system
     * - START Button: Reset odometry to current AprilTag vision position
     */
    private void handleCopilotControls() {
        if (gamepad2 == null) return; // Safety check

        // Button X: Force BLUE alliance
        if (gamepad2.xWasPressed()) {
            if (SharedState.alliance != CommonConstants.Alliance.BLUE) {
                SharedState.alliance = CommonConstants.Alliance.BLUE;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("⚠ COPILOT: Alliance switched to BLUE");
            }
        }

        // Button B: Force RED alliance
        if (gamepad2.bWasPressed()) {
            if (SharedState.alliance != CommonConstants.Alliance.RED) {
                SharedState.alliance = CommonConstants.Alliance.RED;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("⚠ COPILOT: Alliance switched to RED");
            }
        }

        // START: Reset odometry to vision position
        if (gamepad2.start) {
            robot.resetOdometryToVision();
            telemetry.addLine("⚠ COPILOT: Odometry reset to vision");
        }
    }

    /**
     * Displays comprehensive telemetry data to the Driver Station.
     *
     * This method provides real-time feedback on all robot systems including:
     * - Drive system status and odometry
     * - Intake state and power levels
     * - Intake sensor fusion and game element inventory
     * - Launcher targeting mode and flywheel speeds
     * - Flipper and spinner status
     * - Vision system data and auto-aim status
     * - Quick reference guide for controls
     *
     * The telemetry is organized into clear sections with visual indicators (✓, ⚠)
     * to help drivers quickly understand the robot's current state during matches.
     */
    private void displayTelemetry() {
        telemetry.addLine("=== GEARGIRLSBOT2 ===");
        telemetry.addLine();

        // --- Drive Telemetry ---
        telemetry.addLine("--- DRIVETRAIN ---");
        telemetry.addData("Drive Mode", driveMode);
        robot.drive.addTelemetry();
        telemetry.addLine();

        // --- Intake Telemetry ---
        telemetry.addLine("--- INTAKE ---");
        telemetry.addData("State", intakeState);
        telemetry.addData("Motor Power", "%.0f%%", robot.intake.getCurrentPower() * 100);
        telemetry.addData("Running", robot.intake.isRunning() ? "YES" : "NO");
        telemetry.addLine();

        // --- INTAKE SENSOR FUSION TELEMETRY ---
        telemetry.addLine("--- INTAKE INVENTORY (Sensor Fusion) ---");
        if (robot.intakeSensors != null) {
            // Use the built-in telemetry method from IntakeSensorFusion002
            robot.intakeSensors.addTelemetry();

            // Add high-level API test results
            List<IntakeSensorFusion002.ArtifactColor> inventory = robot.getIntakeInventory();
            telemetry.addData("Inventory API", inventory.toString());

            // Test individual slot queries
            IntakeSensorFusion002.ArtifactColor leftColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT);
            IntakeSensorFusion002.ArtifactColor rightColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT);
            telemetry.addData("Left Slot Query", leftColor);
            telemetry.addData("Right Slot Query", rightColor);
        } else {
            telemetry.addData("Status", "Sensor Fusion NOT AVAILABLE");
        }
        telemetry.addLine();

        // --- Launcher Targeting Telemetry ---
        telemetry.addLine("--- LAUNCHER TARGETING ---");
        telemetry.addData("System State", launcherSystemState);
        telemetry.addData("Targeting Mode", targetingMode);

        switch (targetingMode) {
            case AUTO:
                telemetry.addData("Auto Source", "Vision/Odometry");
                telemetry.addData("Distance to Goal", "%.1f in", robot.getDistanceToGoal());
                break;
            case PRESET:
                telemetry.addData("Preset Distance", launcherDistance);
                telemetry.addData("(DPad→ to toggle)", "CLOSE ↔ FAR");
                break;
            case MANUAL:
                telemetry.addData("Manual Setting", "%.0f ticks/sec", manualTargetVelocity);
                telemetry.addData("(DPad↑↓ to adjust)", "");
                break;
        }

        telemetry.addData("Final Target", "%.0f ticks/sec", finalTargetVelocity);
        telemetry.addLine();

        // --- Flywheel Telemetry ---
        telemetry.addLine("--- FLYWHEELS ---");
        telemetry.addData("Status", flywheelsRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Left Actual", "%.0f ticks/sec", robot.launcher.getLeftMotorVelocity());
        telemetry.addData("Right Actual", "%.0f ticks/sec", robot.launcher.getRightMotorVelocity());
        telemetry.addData("Left RPM", "%.0f RPM", robot.launcher.getLeftMotorVelocityRPM());
        telemetry.addData("Right RPM", "%.0f RPM", robot.launcher.getRightMotorVelocityRPM());

        // Calculate and display velocity error
        if (flywheelsRunning) {
            double leftError = Math.abs(robot.launcher.getLeftMotorVelocity() - finalTargetVelocity);
            double rightError = Math.abs(robot.launcher.getRightMotorVelocity() - finalTargetVelocity);
            telemetry.addData("Left Error", "%.0f ticks/sec", leftError);
            telemetry.addData("Right Error", "%.0f ticks/sec", rightError);

            // At-speed indicator
            boolean atSpeed = (leftError < 25 && rightError < 25);
            telemetry.addData("At Speed", atSpeed ? "✓ READY ✓" : "⚠ WAIT");
        }
        telemetry.addLine();

        // --- Flippers & Spinner Status ---
        telemetry.addLine("--- FLIPPERS & SPINNER ---");
        telemetry.addData("Flippers Busy", robot.areFlippersBusy() ? "YES" : "NO");
        telemetry.addData("Left Flipper", robot.flippers.getLeftState());
        telemetry.addData("Right Flipper", robot.flippers.getRightState());
        telemetry.addData("Spinner State", robot.spinner.getSpinnerState());
        telemetry.addData("Spinner Position", "%.3f", robot.spinner.getSpinnerPosition());
        telemetry.addLine();

        // --- Vision System ---
        telemetry.addLine("--- VISION / AUTO-AIM ---");
        telemetry.addData("Target Visible", robot.vision.isTargetVisible() ? "YES" : "NO");
        if (robot.vision.isTargetVisible()) {
            telemetry.addData("Distance", "%.1f inches", robot.vision.getDistanceToTagInches());
            telemetry.addData("Angle X (tx)", "%.1f°", robot.vision.getTargetAngleX());
            if (gamepad1.right_stick_button) {
                telemetry.addData("Auto-Aim", "ACTIVE | Error: %.1f°", robot.vision.getTargetAngleX());
            } else {
                telemetry.addData("Auto-Aim", "Ready (Press R3)");
            }
        } else {
            telemetry.addData("Auto-Aim", "No target visible");
        }

        // Add full vision telemetry
        robot.vision.addTelemetry();
        telemetry.addLine();

        // --- Quick Reference ---
        telemetry.addLine("--- QUICK CONTROLS ---");
        telemetry.addLine("SPINNER: LB=Left | RB=Right");
        telemetry.addLine("TARGETING: DPad← cycle modes");
        telemetry.addLine("           DPad→ toggle CLOSE↔FAR");
        telemetry.addLine("           DPad↑↓ Manual ±25");
        telemetry.addLine("FIRE: LT=Left | RT=Right");

        // This command sends all queued telemetry data to the Driver Station
        telemetry.update();
    }

    /**
     * Applies deadband to joystick input to prevent drift from small unintentional movements.
     *
     * @param value The raw joystick value
     * @return The joystick value with deadband applied (0.0 if within deadband threshold)
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Clean shutdown
        if (robot != null) {
            robot.stopAll();
        }
    }
}