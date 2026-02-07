package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * MINIMAL Servo Test - Diagnose Basic Servo Function
 *
 * This OpMode bypasses all the Turret class logic to test if the servo
 * itself is working properly.
 *
 * Controls:
 * - DPad Up: Increase position by 0.05
 * - DPad Down: Decrease position by 0.05
 * - A: Set to 0.25
 * - B: Set to 0.50
 * - Y: Set to 0.75
 *
 * WHAT TO CHECK:
 * 1. Does servo move when you press buttons?
 * 2. Can you move it by hand? (Should have STRONG resistance)
 * 3. Does position value in telemetry match what you command?
 */
@TeleOp(name = "DEBUG: Minimal Servo Test", group = "Diagnostics")
@Disabled
public class MinimalServoTest extends LinearOpMode {

    private ServoImplEx servo;
    private double targetPosition = 0.5;

    @Override
    public void runOpMode() {
        // Initialize servo
        servo = hardwareMap.get(ServoImplEx.class, "turret");

        // Set PWM range for 5-turn servo
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.addLine("=================================");
        telemetry.addLine("  MINIMAL SERVO DIAGNOSTIC TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("This tests ONLY the servo, nothing else.");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("DPad Up: +0.05");
        telemetry.addLine("DPad Down: -0.05");
        telemetry.addLine("A: Jump to 0.25");
        telemetry.addLine("B: Jump to 0.50");
        telemetry.addLine("Y: Jump to 0.75");
        telemetry.addLine();
        telemetry.addLine("WHAT TO CHECK:");
        telemetry.addLine("1. Does servo MOVE?");
        telemetry.addLine("2. Can you move it by HAND?");
        telemetry.addLine("   (Should be HARD to move!)");
        telemetry.addLine("3. Does it hold position?");
        telemetry.addLine();
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        // Set initial position
        servo.setPosition(targetPosition);

        while (opModeIsActive()) {
            // Read previous button states for edge detection
            boolean prevDpadUp = gamepad1.dpad_up;
            boolean prevDpadDown = gamepad1.dpad_down;

            // Handle controls
            if (gamepad1.dpad_up && !prevDpadUp) {
                targetPosition += 0.05;
                if (targetPosition > 1.0) targetPosition = 1.0;
                servo.setPosition(targetPosition);
                sleep(100);
            }

            if (gamepad1.dpad_down && !prevDpadDown) {
                targetPosition -= 0.05;
                if (targetPosition < 0.0) targetPosition = 0.0;
                servo.setPosition(targetPosition);
                sleep(100);
            }

            // Jump to presets
            if (gamepad1.a) {
                targetPosition = 0.25;
                servo.setPosition(targetPosition);
                sleep(200);
            }
            if (gamepad1.b) {
                targetPosition = 0.50;
                servo.setPosition(targetPosition);
                sleep(200);
            }
            if (gamepad1.y) {
                targetPosition = 0.75;
                servo.setPosition(targetPosition);
                sleep(200);
            }

            // Display status
            telemetry.addLine("=== SERVO STATUS ===");
            telemetry.addData("Target Position", "%.3f", targetPosition);
            telemetry.addData("Actual Position", "%.3f", servo.getPosition());
            telemetry.addLine();

            // PWM info
            telemetry.addLine("=== PWM INFO ===");
            telemetry.addData("PWM Enabled", servo.isPwmEnabled() ? "YES ✓" : "NO ✗");

            telemetry.addLine();
            telemetry.addLine("=== DIAGNOSTICS ===");

            // Check if servo is moving
            double difference = Math.abs(targetPosition - servo.getPosition());
            if (difference > 0.001) {
                telemetry.addLine("⚠ Position mismatch!");
                telemetry.addData("Difference", "%.4f", difference);
            } else {
                telemetry.addLine("✓ Position matches");
            }

            telemetry.addLine();
            telemetry.addLine("=== PHYSICAL CHECK ===");
            telemetry.addLine("Try to move servo by HAND:");
            telemetry.addLine("• If EASY to move → PROBLEM!");
            telemetry.addLine("  (Power issue or wrong mode)");
            telemetry.addLine("• If HARD to move → GOOD!");
            telemetry.addLine("  (Servo is holding position)");

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("DPad Up/Down: Adjust ±0.05");
            telemetry.addLine("A/B/Y: Jump to preset");

            telemetry.update();
        }
    }
}