package org.firstinspires.ftc.teamcode.utilities.P3Robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * Turret Calibration Tool
 *
 * Use this OpMode to:
 * 1. Verify PWM range is set correctly
 * 2. Find the optimal home position
 * 3. Test mechanical range
 * 4. Verify limit switch behavior
 *
 * Controls:
 * - DPad Up/Down: Move servo position by 0.01
 * - DPad Left/Right: Move servo position by 0.001 (fine adjustment)
 * - A: Jump to position 0.50 (theoretical center)
 * - B: Jump to position 0.00 (min)
 * - Y: Jump to position 1.00 (max)
 * - X: Mark current position as "home candidate"
 */
@TeleOp(name = "Turret Calibration", group = "Setup")
public class TurretCalibration extends LinearOpMode {

    private ServoImplEx turretServo;
    private DigitalChannel magSwitch;
    private double currentPos = 0.5;
    private double homeCandidatePos = -1.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        turretServo = hardwareMap.get(ServoImplEx.class, "turret");
        magSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Set PWM range for 5-turn servo
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.addLine("=================================");
        telemetry.addLine("    TURRET CALIBRATION TOOL");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("This tool helps you:");
        telemetry.addLine("1. Find optimal home position");
        telemetry.addLine("2. Verify full mechanical travel");
        telemetry.addLine("3. Test limit switch");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("DPad Up/Down: ±0.01");
        telemetry.addLine("DPad Left/Right: ±0.001");
        telemetry.addLine("A: Jump to 0.50");
        telemetry.addLine("B: Jump to 0.00");
        telemetry.addLine("Y: Jump to 1.00");
        telemetry.addLine("X: Mark as home candidate");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read previous state for edge detection
            boolean prevDpadUp = gamepad1.dpad_up;
            boolean prevDpadDown = gamepad1.dpad_down;
            boolean prevDpadLeft = gamepad1.dpad_left;
            boolean prevDpadRight = gamepad1.dpad_right;

            // Handle controls
            if (gamepad1.dpad_up) {
                currentPos += 0.01;
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                currentPos -= 0.01;
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                currentPos += 0.001;
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                currentPos -= 0.001;
                sleep(100);
            }

            // Jump to presets
            if (gamepad1.a) {
                currentPos = 0.50;
                sleep(200);
            }
            if (gamepad1.b) {
                currentPos = 0.00;
                sleep(200);
            }
            if (gamepad1.y) {
                currentPos = 1.00;
                sleep(200);
            }

            // Mark home candidate
            if (gamepad1.x) {
                homeCandidatePos = currentPos;
                sleep(200);
            }

            // Clamp position
            if (currentPos > 1.0) currentPos = 1.0;
            if (currentPos < 0.0) currentPos = 0.0;

            // Update servo
            turretServo.setPosition(currentPos);

            // Display status
            displayStatus();
        }
    }

    private void displayStatus() {
        telemetry.addLine("=== CURRENT STATUS ===");
        telemetry.addData("Servo Position", "%.4f", currentPos);
        telemetry.addLine();

        // Switch status
        boolean switchTriggered = !magSwitch.getState();  // Active-low
        telemetry.addData("Limit Switch", switchTriggered ? "TRIGGERED ✓" : "Not triggered");
        telemetry.addLine();

        // Calculate turret angle (assumes 450° range, INVERTED: lower position = positive angle)
        double centerOffset = (0.5 - currentPos) * 450.0;
        telemetry.addData("Turret Angle (from 0.5)", "%.1f°", centerOffset);
        telemetry.addLine();

        // Home candidate analysis
        if (homeCandidatePos >= 0) {
            telemetry.addLine("=== HOME CANDIDATE ===");
            telemetry.addData("Marked Position", "%.4f", homeCandidatePos);

            // Check if in safe range for ±160° travel
            boolean inSafeRange = (homeCandidatePos >= 0.356 && homeCandidatePos <= 0.644);
            telemetry.addData("Safe for ±160°?", inSafeRange ? "YES ✓" : "NO ⚠");

            if (inSafeRange) {
                double minPos = homeCandidatePos - 0.3556;
                double maxPos = homeCandidatePos + 0.3556;
                telemetry.addData("Left Limit (-160°)", "%.4f", minPos);
                telemetry.addData("Right Limit (+160°)", "%.4f", maxPos);

                // Calculate buffers
                double leftBuffer = minPos - 0.0;
                double rightBuffer = 1.0 - maxPos;
                telemetry.addData("Left Buffer", "%.4f (%.1f°)", leftBuffer, leftBuffer * 450.0);
                telemetry.addData("Right Buffer", "%.4f (%.1f°)", rightBuffer, rightBuffer * 450.0);
            } else {
                telemetry.addLine();
                telemetry.addLine("⚠ WARNING ⚠");
                telemetry.addLine("Home must be 0.356-0.644");
                telemetry.addLine("for full ±160° travel!");
                if (homeCandidatePos < 0.356) {
                    telemetry.addLine("→ Re-mesh gears CLOCKWISE");
                } else {
                    telemetry.addLine("→ Re-mesh gears COUNTER-CW");
                }
            }

            telemetry.addLine();
        }

        // Instructions
        telemetry.addLine("=== CALIBRATION STEPS ===");
        telemetry.addLine("1. Move turret to face FORWARD");
        telemetry.addLine("2. Adjust until switch triggers");
        telemetry.addLine("3. Press X to mark position");
        telemetry.addLine("4. Check if position is safe");
        telemetry.addLine();
        telemetry.addLine("IDEAL: Home around 0.50");
        telemetry.addLine("ACCEPTABLE: 0.356 - 0.644");

        telemetry.update();
    }
}