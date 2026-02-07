package org.firstinspires.ftc.teamcode.TeleOp.P3Robot.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_TurretUtil;

/**
 * TeleOp mode for controlling the turret using gamepad joysticks.
 *
 * Controls:
 * - Right Stick X: Rotate turret (left = CCW, right = CW)
 * - A Button: Return turret to home (0 degrees)
 * - B Button: Emergency stop
 * - D-pad Left/Right: Fine adjustment (5 degrees at a time)
 */
@TeleOp(name="Turret Control", group="TeleOp")
@Disabled
public class TurretTeleOp extends LinearOpMode {

    // Turret subsystem
    private P3_TurretUtil turret;

    // Control sensitivity settings
    private static final double JOYSTICK_DEADZONE = 0.1;  // Ignore small joystick movements
    private static final double ROTATION_SPEED = 90.0;     // Max degrees per second
    private static final double FINE_ADJUST_ANGLE = 5.0;   // Degrees per D-pad press

    // Timing for smooth control
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime dpadTimer = new ElapsedTime();
    private static final double DPAD_COOLDOWN = 0.2; // Seconds between D-pad presses

    @Override
    public void runOpMode() {
        // Initialize the turret
        telemetry.addData("Status", "Initializing turret...");
        telemetry.update();

        turret = new P3_TurretUtil(hardwareMap);

        telemetry.addData("Status", "Ready! Position turret at front and press START");
        telemetry.addData("Controls", "Right Stick X = Rotate");
        telemetry.addData("", "A = Home, B = E-Stop, D-pad = Fine Adjust");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // === JOYSTICK CONTROL (Continuous Rotation) ===
            handleJoystickControl();

            // === BUTTON CONTROLS ===
            handleButtonControls();

            // === D-PAD CONTROLS (Fine Adjustment) ===
            handleDpadControls();

            // === UPDATE TURRET ===
            turret.update();

            // === TELEMETRY ===
            displayTelemetry();
        }
    }

    /**
     * Handles joystick input for continuous turret rotation.
     * Right stick X-axis controls rotation direction and speed.
     */
    private void handleJoystickControl() {
        // Get joystick input (right stick X-axis)
        double stickX = gamepad1.right_stick_x;

        turret.setTurretPower(stickX);
    }

    /**
     * Handles button inputs for special turret commands.
     */
    private void handleButtonControls() {
        // A button - Return to home position
        if (gamepad1.aWasPressed()) {
            turret.returnToHome();
        }

        // B button - Emergency stop
        if (gamepad1.bWasPressed()) {
            turret.emergencyStop();
        }

        // Y button - Recalibrate (use with caution!)
        if (gamepad1.yWasPressed() && gamepad1.startWasPressed()) {
            // Require both Y and Start to prevent accidental recalibration
            turret.recalibrate();
        }
    }

    /**
     * Handles D-pad inputs for fine angle adjustments.
     */
    private void handleDpadControls() {
        // Only process D-pad if enough time has passed (debounce)
            // D-pad right - Rotate CW by small amount
            if (gamepad1.dpadRightWasPressed()) {
                turret.rotateRelative(FINE_ADJUST_ANGLE);
                dpadTimer.reset();
            }

            // D-pad left - Rotate CCW by small amount
            if (gamepad1.dpadLeftWasPressed()) {
                turret.rotateRelative(-FINE_ADJUST_ANGLE);
                dpadTimer.reset();
            }
    }

    /**
     * Displays telemetry information to the driver station.
     */
    private void displayTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addLine();

        // Turret information
        turret.addTelemetry(telemetry);
        telemetry.addLine();

        // Control hints
        telemetry.addData("Right Stick X", "%.2f", gamepad1.right_stick_x);
        telemetry.addLine("─────────────────────");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Right Stick X = Rotate");
        telemetry.addLine("  A = Home Position");
        telemetry.addLine("  B = Emergency Stop");
        telemetry.addLine("  D-pad L/R = Fine Adjust ±5°");

        telemetry.update();
    }
}
