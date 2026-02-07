package org.firstinspires.ftc.teamcode.TeleOp.P3Robot.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * ABSOLUTE MINIMAL TEST - NO TURRET CLASS
 *
 * This tests the servo with ZERO abstraction to see what's actually happening
 *
 * Controls:
 * - Right Stick X: Direct servo position control
 *   - Stick Right → Servo position INCREASES (0.5 → 0.6 → 0.7)
 *   - Stick Left → Servo position DECREASES (0.5 → 0.4 → 0.3)
 */
@TeleOp(name = "ULTRA MINIMAL Servo Test", group = "Diagnostics")
@Disabled
public class UltraMinimalServoTest extends LinearOpMode {

    private ServoImplEx servo;
    private double servoPosition = 0.6611;  // Start at center

    @Override
    public void runOpMode() {
        // Initialize servo
        servo = hardwareMap.get(ServoImplEx.class, "turret");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.addLine("=================================");
        telemetry.addLine("  ULTRA MINIMAL SERVO TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("Right Stick X controls servo:");
        telemetry.addLine("  Right → Position increases");
        telemetry.addLine("  Left → Position decreases");
        telemetry.addLine();
        telemetry.addLine("Watch BOTH:");
        telemetry.addLine("  1. Servo position number");
        telemetry.addLine("  2. Physical turret direction");
        telemetry.addLine();
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        servo.setPosition(servoPosition);

        while (opModeIsActive()) {
            // Read joystick
            double stickValue = gamepad2.right_stick_x;

            // SUPER SIMPLE: Add stick value directly (scaled down)
            if (Math.abs(stickValue) > 0.05) {  // Deadband
                servoPosition += stickValue * 0.005;  // Very slow movement

                // Clamp to 0.0 - 1.0
                if (servoPosition > 1.0) servoPosition = 1.0;
                if (servoPosition < 0.0) servoPosition = 0.0;

                servo.setPosition(servoPosition);
            }

            // Display
            telemetry.addLine("=== RAW VALUES ===");
            telemetry.addData("Joystick RAW", "%.3f", gamepad2.right_stick_x);
            telemetry.addData("Stick Value", "%.3f", stickValue);
            telemetry.addLine();

            telemetry.addLine("=== SERVO ===");
            telemetry.addData("Commanded Position", "%.4f", servoPosition);
            telemetry.addData("Actual Position", "%.4f", servo.getPosition());
            telemetry.addLine();

            telemetry.addLine("=== WHAT TO WATCH ===");
            telemetry.addLine("Push stick RIGHT:");
            telemetry.addLine("  • Position should INCREASE");
            telemetry.addLine("  • Watch which way turret moves");
            telemetry.addLine();
            telemetry.addLine("Push stick LEFT:");
            telemetry.addLine("  • Position should DECREASE");
            telemetry.addLine("  • Watch which way turret moves");
            telemetry.addLine();

            // Direction indicator
            if (stickValue > 0.1) {
                telemetry.addLine(">>> STICK RIGHT - Pos increasing");
            } else if (stickValue < -0.1) {
                telemetry.addLine("<<< STICK LEFT - Pos decreasing");
            } else {
                telemetry.addLine("--- STICK CENTERED ---");
            }

            telemetry.update();
        }
    }
}