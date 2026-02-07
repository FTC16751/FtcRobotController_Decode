package org.firstinspires.ftc.teamcode.TeleOp.GGRobot;

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
 * Primary TeleOp program for Gear Girls Bot 2 - FTC DECODE Season
 *
 * GAMEPAD 1 (DRIVER):
 * • Drivetrain: Left stick (move), Right stick X (turn), R3 (auto-aim), BACK (toggle drive mode), START (reset)
 * • Intake: A (on/off), X (reverse/off)
 * • Launcher: Y (spin up), B (stop), DPad← (cycle modes), DPad→ (close/far), DPad↑↓ (manual adjust)
 * • Spinner: LT (left), RT (right)
 * • Fire: LB (left), RB (right)
 *
 * GAMEPAD 2 (COPILOT):
 * • X (force BLUE), B (force RED), START (vision reset)
 *
 * @version 3.0 - Cleaned up and enhanced telemetry
 * @author GearGirls Team
 */
@TeleOp(name = "Gear Girls Bot 2 (RUN ME)", group = " _GGopmodes")
public class GearGirlsBot2_improved extends OpMode {

    // ========================================
    // ROBOT & SUBSYSTEMS
    // ========================================
    private GGRobot2 robot;

    // ========================================
    // DRIVE SYSTEM
    // ========================================
    private enum DriveMode { FIELD_CENTRIC, ARCADE }
    private DriveMode driveMode = DriveMode.ARCADE;

    private static final double JOYSTICK_DEADBAND = 0.03;
    private static final double TX_ALIGN_KP = 0.04;
    private static final double TX_ALIGN_TOLERANCE_DEG = 1.0;

    // ========================================
    // INTAKE SYSTEM
    // ========================================
    private enum IntakeState { ON, OFF, REVERSE }
    private IntakeState intakeState = IntakeState.OFF;

    // ========================================
    // LAUNCHER SYSTEM
    // ========================================
    private GGRobotConstants.LauncherTargetingMode targetingMode = GGRobotConstants.LauncherTargetingMode.AUTO;
    private GGRobotConstants.LauncherSystemState launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
    private GGRobotConstants.LauncherDistance launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;

    private double manualTargetVelocity = 1800;
    private double finalTargetVelocity = 0;
    private boolean flywheelsRunning = false;

    private static final double VELOCITY_INCREMENT = 25;
    private static final double LAUNCHER_VELOCITY_TOLERANCE = 25.0;
    private static final double MINIMUM_SAFE_VELOCITY = 500.0;

    // ========================================
    // TIMING
    // ========================================
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ========================================
    // INITIALIZATION
    // ========================================
    @Override
    public void init() {
        robot = new GGRobot2(hardwareMap, telemetry);

        // Load alliance from autonomous
        CommonConstants.Alliance alliance = SharedState.alliance;
        robot.configureVisionForTeleOp(alliance);

        // Reset all states
        resetSystemStates();

        loopTimer.reset();

        // Initialization complete message
        telemetry.addLine("🤖 ═══ GEARGIRLSBOT2 ═══");
        telemetry.addData("✅ Status", "Initialized");
        telemetry.addData("🎯 Alliance", alliance);
        telemetry.addData("🔧 Mode", "Ready for Start");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    // ========================================
    // MAIN LOOP
    // ========================================
    @Override
    public void loop() {
        robot.update();

        handleDriveControls();
        handleIntakeControls();
        handleSpinnerControls();
        handleLauncherControls();
        handleCopilotControls();

        displayTelemetry();
    }

    // ========================================
    // DRIVE CONTROLS
    // ========================================
    private void handleDriveControls() {
        // Reset odometry
        if (gamepad1.start) {
            robot.drive.pinpoint.resetPosAndIMU();
            telemetry.addLine("⚠️ ODOMETRY RESET");
        }

        // Toggle drive mode
        if (gamepad1.back) {
            toggleDriveMode();
        }

        // Read joystick inputs
        double driveInput = applyDeadband(-gamepad1.left_stick_y);
        double strafeInput = applyDeadband(gamepad1.left_stick_x);
        double turnInput = applyDeadband(gamepad1.right_stick_x);

        // Auto-aim override
        if (gamepad1.right_stick_button && robot.vision.isTargetVisible()) {
            double txError = robot.vision.getTargetAngleX();
            turnInput = (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) ? 0.0 : TX_ALIGN_KP * txError;
        }

        // Execute drive command
        robot.drive.arcadeDrive(strafeInput, driveInput, turnInput, 0, 1.0);
    }

    private void toggleDriveMode() {
        if (driveMode == DriveMode.ARCADE) {
            driveMode = DriveMode.FIELD_CENTRIC;
            gamepad1.rumble(200);
        } else {
            driveMode = DriveMode.ARCADE;
            gamepad1.rumble(100);
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }

    // ========================================
    // INTAKE CONTROLS
    // ========================================
    private void handleIntakeControls() {
        if (gamepad1.a) {
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }

        if (gamepad1.x) {
            intakeState = (intakeState == IntakeState.REVERSE) ? IntakeState.OFF : IntakeState.REVERSE;
        }

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

    // ========================================
    // SPINNER CONTROLS
    // ========================================
    private void handleSpinnerControls() {

        if (gamepad1.left_trigger > 0.2) {
            robot.rotateSpinnerLeft();
        }
        if (gamepad1.right_trigger > 0.2) {
            robot.rotateSpinnerRight();
        }
    }

    // ========================================
    // LAUNCHER CONTROLS
    // ========================================
    private void handleLauncherControls() {
        // Cycle targeting modes
        if (gamepad1.dpad_left) {
            cycleTargetingMode();
        }

        // Toggle preset distance
        if (gamepad1.dpad_right) {
            launcherDistance = (launcherDistance == GGRobotConstants.LauncherDistance.CLOSE) ?
                    GGRobotConstants.LauncherDistance.FAR : GGRobotConstants.LauncherDistance.CLOSE;
            targetingMode = GGRobotConstants.LauncherTargetingMode.PRESET;
        }

        // Manual velocity adjustment
        if (gamepad1.dpad_up) {
            manualTargetVelocity = Range.clip(manualTargetVelocity + VELOCITY_INCREMENT, 0, 3000);
            targetingMode = GGRobotConstants.LauncherTargetingMode.MANUAL;
        }
        if (gamepad1.dpad_down) {
            manualTargetVelocity = Range.clip(manualTargetVelocity - VELOCITY_INCREMENT, 0, 3000);
            targetingMode = GGRobotConstants.LauncherTargetingMode.MANUAL;
        }

        // Activate/deactivate launcher
        if (gamepad1.y) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.ACTIVE;
            flywheelsRunning = true;
        }
        if (gamepad1.b) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
            flywheelsRunning = false;
        }

        // Update launcher velocity based on mode
        switch (targetingMode) {
            case AUTO:
            case PRESET:
                finalTargetVelocity = robot.updateLauncher(launcherSystemState, targetingMode, launcherDistance);
                break;
            case MANUAL:
                if (launcherSystemState == GGRobotConstants.LauncherSystemState.ACTIVE) {
                    robot.launcher.setMotorVelocity(manualTargetVelocity, manualTargetVelocity);
                    finalTargetVelocity = manualTargetVelocity;
                }
                break;
        }

        // Handle firing
        handleFiring();
    }

    private void handleFiring() {
        boolean isSpeedCorrect = (robot.launcher.getLeftMotorVelocity() >= (finalTargetVelocity - LAUNCHER_VELOCITY_TOLERANCE));
        boolean isSpeedSafe = (robot.launcher.getLeftMotorVelocity() > MINIMUM_SAFE_VELOCITY);
        boolean isLauncherReady = (launcherSystemState == GGRobotConstants.LauncherSystemState.ACTIVE) && isSpeedCorrect && isSpeedSafe;

        // Fire both flippers
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            if (isLauncherReady) {
                robot.purgeAllBalls();
            } else {
                telemetry.addLine("⚠️ LAUNCHER NOT READY");
            }
        }
        // Fire individual flippers
        else if (isLauncherReady) {
            if (gamepad1.left_bumper) {
                robot.triggerLeftFlipper();
            }
            if (gamepad1.right_bumper) {
                robot.triggerRightFlipper();
            }
        } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
            telemetry.addLine("⚠️ LAUNCHER NOT READY");
        }
    }

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

    // ========================================
    // COPILOT CONTROLS
    // ========================================
    private void handleCopilotControls() {
        if (gamepad2 == null) return;

        if (gamepad2.x) {
            if (SharedState.alliance != CommonConstants.Alliance.BLUE) {
                SharedState.alliance = CommonConstants.Alliance.BLUE;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("⚠️ COPILOT: Switched to BLUE");
            }
        }

        if (gamepad2.b) {
            if (SharedState.alliance != CommonConstants.Alliance.RED) {
                SharedState.alliance = CommonConstants.Alliance.RED;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("⚠️ COPILOT: Switched to RED");
            }
        }

        if (gamepad2.start) {
            robot.resetOdometryToVision();
            telemetry.addLine("⚠️ COPILOT: Vision reset");
        }
    }

    // ========================================
    // TELEMETRY - ENHANCED WITH ICONS
    // ========================================
    private void displayTelemetry() {
        telemetry.addLine("🤖 ═══ GEARGIRLSBOT2 ═══");

        displayDriveTelemetry();
        displayIntakeTelemetry();
        displayInventoryTelemetry();
        displayLauncherTelemetry();
        displayFlywheelTelemetry();
        displayFlippersSpinnerTelemetry();
        displayVisionTelemetry();
        displayQuickReference();

        telemetry.update();
    }

    private void displayDriveTelemetry() {
        telemetry.addLine("\n🚗 DRIVE");
        telemetry.addData("  Mode", driveMode);
        telemetry.addData("  Status", "Active");
    }

    private void displayIntakeTelemetry() {
        telemetry.addLine("\n📥 INTAKE");
        String stateIcon = intakeState == IntakeState.ON ? "🟢" :
                intakeState == IntakeState.REVERSE ? "🔴" : "⚪";
        telemetry.addData("  State", stateIcon + " " + intakeState);
        telemetry.addData("  Power", "%.0f%%", robot.intake.getCurrentPower() * 100);
    }

    private void displayInventoryTelemetry() {
        telemetry.addLine("\n📦 INVENTORY");
        if (robot.intakeSensors != null) {
            List<IntakeSensorFusion002.ArtifactColor> inventory = robot.getIntakeInventory();
            IntakeSensorFusion002.ArtifactColor leftColor;
            IntakeSensorFusion002.ArtifactColor centerColor;
            IntakeSensorFusion002.ArtifactColor rightColor;
            if (robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.LEFT)) {
                leftColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT);
            } else {
                leftColor = IntakeSensorFusion002.ArtifactColor.EMPTY;
            }
            if (robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.CENTER)) {
                centerColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.CENTER);
            } else {
                centerColor = IntakeSensorFusion002.ArtifactColor.EMPTY;
            }
            if (robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.RIGHT)) {
                rightColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT);
            } else {
               rightColor = IntakeSensorFusion002.ArtifactColor.EMPTY;

            }

            String leftIcon = getColorIcon(leftColor);
            String rightIcon = getColorIcon(rightColor);
            String centerIcon = getColorIcon(centerColor);

            telemetry.addData("  Left", leftIcon + " " + leftColor);
            telemetry.addData("  Center", centerIcon + " " + centerColor);
            telemetry.addData("  Right", rightIcon + " " + rightColor);
        } else {
            telemetry.addData("  Status", "❌ Unavailable");
        }
    }

    private void displayLauncherTelemetry() {
        telemetry.addLine("\n🎯 LAUNCHER");
        String stateIcon = launcherSystemState == GGRobotConstants.LauncherSystemState.ACTIVE ? "🟢" : "⚪";
        telemetry.addData("  State", stateIcon + " " + launcherSystemState);

        String modeIcon = targetingMode == GGRobotConstants.LauncherTargetingMode.AUTO ? "🤖" :
                targetingMode == GGRobotConstants.LauncherTargetingMode.PRESET ? "📍" : "✋";
        telemetry.addData("  Mode", modeIcon + " " + targetingMode);

        switch (targetingMode) {
            case AUTO:
                telemetry.addData("  Distance", "%.1f in", robot.getDistanceToGoal());
                break;
            case PRESET:
                telemetry.addData("  Preset", launcherDistance);
                break;
            case MANUAL:
                telemetry.addData("  Manual", "%.0f ticks/s", manualTargetVelocity);
                break;
        }

        telemetry.addData("  Target", "%.0f ticks/s", finalTargetVelocity);
    }

    private void displayFlywheelTelemetry() {
        telemetry.addLine("\n⚡ FLYWHEELS");
        String statusIcon = flywheelsRunning ? "🟢" : "⚪";
        telemetry.addData("  Status", statusIcon + " " + (flywheelsRunning ? "RUNNING" : "STOPPED"));

        if (flywheelsRunning) {
            double leftVel = robot.launcher.getLeftMotorVelocity();
            double rightVel = robot.launcher.getRightMotorVelocity();
            double leftError = Math.abs(leftVel - finalTargetVelocity);
            double rightError = Math.abs(rightVel - finalTargetVelocity);
            boolean atSpeed = (leftError < 25 && rightError < 25);

            telemetry.addData("  Left", "%.0f (±%.0f)", leftVel, leftError);
            telemetry.addData("  Right", "%.0f (±%.0f)", rightVel, rightError);
            telemetry.addData("  Ready", atSpeed ? "✅ READY" : "⏳ WAIT");
        }
    }

    private void displayFlippersSpinnerTelemetry() {
        telemetry.addLine("\n🔄 FLIPPERS & SPINNER");
        String busyIcon = robot.areFlippersBusy() ? "⏳" : "✅";
        telemetry.addData("  Flippers", busyIcon + " " + (robot.areFlippersBusy() ? "BUSY" : "READY"));
        telemetry.addData("  Spinner", robot.spinner.getSpinnerState());
    }

    private void displayVisionTelemetry() {
        telemetry.addLine("\n👁️ VISION");
        boolean targetVisible = robot.vision.isTargetVisible();
        String visIcon = targetVisible ? "🟢" : "🔴";
        telemetry.addData("  Target", visIcon + " " + (targetVisible ? "VISIBLE" : "NO TARGET"));

        if (targetVisible) {
            telemetry.addData("  Distance", "%.1f in", robot.vision.getDistanceToTagInches());
            telemetry.addData("  Angle", "%.1f°", robot.vision.getTargetAngleX());

            if (gamepad1.right_stick_button) {
                telemetry.addData("  Auto-Aim", "🎯 ACTIVE");
            }
        }
    }

    private void displayQuickReference() {
        telemetry.addLine("\n📋 CONTROLS");
        telemetry.addLine("  🎮 DPad← Cycle | DPad→ Close/Far");
        telemetry.addLine("  🔥 LB+RB Fire Both");
    }

    // ========================================
    // HELPER METHODS
    // ========================================
    private String getColorIcon(IntakeSensorFusion002.ArtifactColor color) {
        switch (color) {
            case PURPLE: return "🟣";
            case GREEN: return "🟢";
            case UNKNOWN: return "❓";
            default: return "⚪";
        }
    }

    private void resetSystemStates() {
        launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
        intakeState = IntakeState.OFF;
        driveMode = DriveMode.ARCADE;
        targetingMode = GGRobotConstants.LauncherTargetingMode.AUTO;
        launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;
        flywheelsRunning = false;
    }

    // ========================================
    // SHUTDOWN
    // ========================================
    @Override
    public void stop() {
        if (robot != null) {
            robot.stopAll();
        }
    }
}