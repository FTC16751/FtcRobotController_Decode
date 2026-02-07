package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.utilities.P3Robot.Turret;

/**
 * Example TeleOp with Turret Control
 *
 * Controls:
 * - Right Stick X: Manual turret control
 * - Right Bumper (hold): Auto-aim with Limelight
 * - A Button: Snap turret to home (0°)
 * - Left Stick Y: Drive forward/back (example - implement your drive here)
 * - Left Stick X: Drive strafe (example - implement your drive here)
 */
@TeleOp(name = "Turret TeleOp Example")
@Disabled
public class TurretTeleOpExample extends LinearOpMode {

    // Hardware
    private Turret turret;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize hardware
        ServoImplEx turretServo = hardwareMap.get(ServoImplEx.class, "turret");
        DigitalChannel magSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Create turret controller
        turret = new Turret(turretServo, magSwitch);

        // Configure Limelight
        limelight.pipelineSwitch(0);  // Use your tracking pipeline
        limelight.start();

        // ========== HOMING PHASE ==========
        telemetry.addLine("=== TURRET HOMING ===");
        telemetry.addLine("Turret will automatically find home position");
        telemetry.addLine("Press PLAY when ready");
        telemetry.update();

        // Run homing during init_loop
        while (!isStarted() && !isStopRequested()) {
            boolean homingComplete = turret.updateHoming();

            telemetry.addData("Homing Status", homingComplete ? "COMPLETE ✓" : "Searching...");
            telemetry.addData("Switch Triggered", turret.isHomeSwitchTriggered());

            if (homingComplete) {
                telemetry.addData("Home Position", "%.3f", turret.getHomePosition());
                telemetry.addData("Position Safe?", turret.isHomePositionSafe() ? "YES ✓" : "NO - CHECK CALIBRATION!");

                if (!turret.isHomePositionSafe()) {
                    telemetry.addLine();
                    telemetry.addLine("⚠ WARNING: Home position outside safe range!");
                    telemetry.addLine("Expected: 0.356 - 0.644");
                    telemetry.addLine("You may lose travel range or hit limits");
                }
            }

            telemetry.update();
        }

        // Safety check
        if (!turret.isHomed()) {
            telemetry.addLine("ERROR: Turret not homed!");
            telemetry.addLine("Cannot start TeleOp safely");
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }

        waitForStart();

        // ========== MAIN CONTROL LOOP ==========
        while (opModeIsActive()) {

            // Read controls
            double manualInput = gamepad1.right_stick_x;  // Positive = right
            boolean autoAimButton = gamepad1.right_bumper;
            boolean snapHomeButton = gamepad1.a;

            // PRIORITY 1: Snap to home (overrides everything)
            if (snapHomeButton) {
                turret.snapToHome();
            }
            // PRIORITY 2: Manual control (overrides auto-aim)
            else if (Math.abs(manualInput) > 0.05) {
                turret.moveManual(manualInput);
            }
            else if (gamepad1.dpadRightWasPressed()) {
                turret.moveManual(1);
            }else if (gamepad1.dpadLeftWasPressed()) {
                turret.moveManual(-1);
            }
            // PRIORITY 3: Auto-aim when button held
            else if (autoAimButton) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    double tx = result.getTx();  // Horizontal offset in degrees
                    boolean targetValid = result.isValid();
                    turret.autoAim(tx, targetValid);
                }
            }
            // Otherwise: Turret holds position (servo does this automatically)

            // Display telemetry
            updateTelemetry();
        }
    }

    /**
     * Display comprehensive turret status
     */
    private void updateTelemetry() {
        // Read controls for display
        double manualInput = gamepad1.right_stick_x;
        boolean autoAimButton = gamepad1.right_bumper;
        boolean snapHomeButton = gamepad1.a;

        telemetry.addLine("=== DEBUG: INPUTS ===");
        telemetry.addData("Right Stick X RAW", "%.3f", gamepad1.right_stick_x);
        telemetry.addData("Manual Input", "%.3f", manualInput);
        telemetry.addData("Above Deadband?", Math.abs(manualInput) > 0.05 ? "YES" : "NO");
        telemetry.addData("Auto-Aim Button", autoAimButton);
        telemetry.addData("Snap Home Button", snapHomeButton);

        telemetry.addLine();
        telemetry.addLine("=== TURRET STATUS ===");
        telemetry.addData("Position", turret.getStatusString());
        telemetry.addData("Homed", turret.isHomed() ? "YES ✓" : "NO");
        telemetry.addData("Current Servo Pos", "%.4f", turret.getCurrentPosition());

        if (turret.isHomed()) {
            telemetry.addData("Home Switch", turret.isHomeSwitchTriggered() ? "TRIGGERED" : "Clear");
            telemetry.addData("moveManual() calls", turret.getManualCallCount());
            telemetry.addData("Last input to moveManual", "%.3f", turret.getLastManualInput());
            telemetry.addLine();
            telemetry.addLine("Soft Limits:");
            telemetry.addLine(turret.getLimitStatus());
        }

        telemetry.addLine();
        telemetry.addLine("=== LIMELIGHT ===");
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            telemetry.addData("Target Valid", result.isValid());
            if (result.isValid()) {
                telemetry.addData("Horizontal Offset (tx)", "%.2f°", result.getTx());
                telemetry.addData("Vertical Offset (ty)", "%.2f°", result.getTy());
            }
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Right Stick X: Manual");
        telemetry.addLine("Right Bumper: Auto-Aim");
        telemetry.addLine("A Button: Snap Home");

        telemetry.update();
    }
}