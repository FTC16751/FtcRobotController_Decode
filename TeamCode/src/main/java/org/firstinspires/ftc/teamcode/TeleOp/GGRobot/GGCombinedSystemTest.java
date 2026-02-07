package org.firstinspires.ftc.teamcode.TeleOp.GGRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeUtilV2;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchFlippers;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LauncherMotors;

/**
 * Combined System Test OpMode
 *
 * Tests all three subsystems together: Intake, LaunchFlippers, and Flywheels
 *
 * Control Scheme (Gamepad 1):
 *
 * INTAKE SYSTEM:
 * - Button A: Toggle intake ON/OFF (collecting)
 * - Button X: Toggle intake REVERSE/OFF (expelling)
 *
 * FLYWHEEL SYSTEM:
 * - Button Y: Start flywheels at target velocity
 * - Button B: Stop flywheels
 * - DPAD UP: Increase flywheel velocity by 25 RPM
 * - DPAD DOWN: Decrease flywheel velocity by 25 RPM
 *
 * LAUNCH FLIPPER SYSTEM:
 * - LEFT BUMPER: Trigger left flipper
 * - RIGHT BUMPER: Trigger right flipper
 * - LEFT TRIGGER + RIGHT TRIGGER: Emergency stop all flippers
 *
 * TIMING ADJUSTMENTS:
 * - DPAD LEFT: Decrease flipper hold time
 * - DPAD RIGHT: Increase flipper hold time
 * - BACK/SELECT: Reset flipper hold time to default
 */
@TeleOp(name = "GG Combined System Test", group = " _GGopmodes")
@Disabled
public class GGCombinedSystemTest extends OpMode {

    // --- Subsystems ---
    private IntakeUtilV2 intake;
    private LaunchFlippers flippers;
    private LauncherMotors launcher;

    // --- Intake State ---
    private enum IntakeState {
        ON,
        OFF,
        REVERSE
    }
    private IntakeState intakeState = IntakeState.OFF;

    // --- Flywheel State ---
    private double targetVelocity = 1200; // Starting velocity in RPM
    private static final double VELOCITY_INCREMENT = 25; // 25 RPM increments
    private boolean flywheelsRunning = false;

    @Override
    public void init() {
        // Initialize all subsystems
        intake = new IntakeUtilV2(hardwareMap);
        flippers = new LaunchFlippers(hardwareMap);
        launcher = new LauncherMotors(hardwareMap);

        // Optional: Configure flipper timing if needed
        // flippers.setFlipHoldTime(0.3);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("All systems ready for testing");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update all subsystems
        handleIntakeControls();
        handleFlywheelControls();
        handleFlipperControls();

        // CRITICAL: Update flippers state machine for automatic retract
        flippers.update();

        // Display telemetry
        displayTelemetry();
    }

    /**
     * Handles intake motor controls
     * A = Toggle intake ON/OFF
     * X = Toggle intake REVERSE/OFF
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

        // Set motor power based on final state
        switch (intakeState) {
            case ON:
                intake.setIntakeMotorPower(GGRobotConstants.Intake.INTAKE_SPEED);
                break;
            case REVERSE:
                intake.setIntakeMotorPower(GGRobotConstants.Intake.OUTTAKE_SPEED);
                break;
            case OFF:
                intake.stop();
                break;
        }
    }

    /**
     * Handles flywheel motor controls
     * Y = Start flywheels
     * B = Stop flywheels
     * DPAD UP/DOWN = Adjust velocity
     */
    private void handleFlywheelControls() {
        // Button Y starts the flywheels
        if (gamepad1.y) {
            launcher.setMotorVelocity(targetVelocity, targetVelocity);
            flywheelsRunning = true;
        }

        // Button B stops the flywheels
        if (gamepad1.b) {
            launcher.setMotorVelocity(0, 0);
            flywheelsRunning = false;
        }

        // DPAD UP increases velocity
        if (gamepad1.dpad_up) {
            targetVelocity += VELOCITY_INCREMENT;
            if (flywheelsRunning) {
                launcher.setMotorVelocity(targetVelocity, targetVelocity);
            }
        }

        // DPAD DOWN decreases velocity
        if (gamepad1.dpad_down) {
            targetVelocity -= VELOCITY_INCREMENT;
            if (targetVelocity < 0) {
                targetVelocity = 0;
            }
            if (flywheelsRunning) {
                launcher.setMotorVelocity(targetVelocity, targetVelocity);
            }
        }
    }

    /**
     * Handles launch flipper controls
     * LEFT BUMPER = Trigger left flipper
     * RIGHT BUMPER = Trigger right flipper
     * BOTH TRIGGERS = Emergency stop
     * DPAD LEFT/RIGHT = Adjust hold time
     */
    private void handleFlipperControls() {
        // Emergency stop if both triggers pressed
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5) {
            flippers.emergencyStop();
        } else {
            // Left bumper triggers left flipper
            if (gamepad1.left_bumper) {
                flippers.trigger(LaunchFlippers.FlipperSide.LEFT);
            }

            // Right bumper triggers right flipper
            if (gamepad1.right_bumper) {
                flippers.trigger(LaunchFlippers.FlipperSide.RIGHT);
            }
        }

        // DPAD LEFT decreases hold time
        if (gamepad1.dpad_left) {
            double newTime = flippers.getFlipHoldTime() - 0.05;
            if (newTime >= 0.1) { // Minimum hold time
                flippers.setFlipHoldTime(newTime);
            }
        }

        // DPAD RIGHT increases hold time
        if (gamepad1.dpad_right) {
            double newTime = flippers.getFlipHoldTime() + 0.05;
            if (newTime <= 2.0) { // Maximum hold time
                flippers.setFlipHoldTime(newTime);
            }
        }

        // BACK button resets hold time to default
        if (gamepad1.back) {
            flippers.resetFlipHoldTime();
        }
    }

    /**
     * Displays comprehensive telemetry for all subsystems
     */
    private void displayTelemetry() {
        telemetry.addLine("=== COMBINED SYSTEM TEST ===");
        telemetry.addLine();

        // --- Intake Telemetry ---
        telemetry.addLine("--- INTAKE ---");
        telemetry.addData("State", intakeState);
        telemetry.addData("Motor Power", "%.0f%%", intake.getCurrentPower() * 100);
        telemetry.addData("Running", intake.isRunning() ? "YES" : "NO");
        telemetry.addLine();

        // --- Flywheel Telemetry ---
        telemetry.addLine("--- FLYWHEELS ---");
        telemetry.addData("Status", flywheelsRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Target Velocity", "%.0f RPM", targetVelocity);
        telemetry.addData("Left Actual", "%.0f RPM", launcher.getLeftMotorVelocity());
        telemetry.addData("Right Actual", "%.0f RPM", launcher.getRightMotorVelocity());
        telemetry.addData("Left RPM ", "%.0f RPM", launcher.getLeftMotorVelocityRPM());
        telemetry.addData("Right RPM", "%.0f RPM", launcher.getRightMotorVelocityRPM());

        // Calculate velocity error
        if (flywheelsRunning) {
            double leftError = Math.abs(launcher.getLeftMotorVelocity() - targetVelocity);
            double rightError = Math.abs(launcher.getRightMotorVelocity() - targetVelocity);
            telemetry.addData("Left Error", "%.0f RPM", leftError);
            telemetry.addData("Right Error", "%.0f RPM", rightError);

            // Indicate if at target speed
            boolean atSpeed = (leftError < 50 && rightError < 50);
            telemetry.addData("At Speed", atSpeed ? "YES ✓" : "NO");
        }
        telemetry.addLine();

        // --- Flipper Telemetry ---
        telemetry.addLine("--- FLIPPERS ---");
        telemetry.addData("Status", flippers.getStatus());
        telemetry.addData("System Busy", flippers.isBusy() ? "YES" : "NO");
        telemetry.addData("Left Busy", flippers.isLeftBusy() ? "YES" : "NO");
        telemetry.addData("Right Busy", flippers.isRightBusy() ? "YES" : "NO");
        telemetry.addData("Hold Time", "%.2f sec", flippers.getFlipHoldTime());
        telemetry.addData("Left Position", "%.3f", flippers.getLeftPosition());
        telemetry.addData("Right Position", "%.3f", flippers.getRightPosition());
        telemetry.addLine();

        // --- Controls Reference ---
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("INTAKE: A=On/Off | X=Reverse/Off");
        telemetry.addLine("FLYWHEELS: Y=Start | B=Stop");
        telemetry.addLine("            DPad↑↓=Adjust Speed");
        telemetry.addLine("FLIPPERS: LB=Left | RB=Right");
        telemetry.addLine("          LT+RT=Emergency Stop");
        telemetry.addLine("          DPad←→=Timing | Back=Reset");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean shutdown - stop all systems
        if (intake != null) {
            intake.stop();
        }
        if (launcher != null) {
            launcher.setMotorVelocity(0, 0);
        }
        if (flippers != null) {
            flippers.emergencyStop();
        }
    }
}