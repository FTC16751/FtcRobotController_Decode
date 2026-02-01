package org.firstinspires.ftc.teamcode.TeleOp.GGRobot;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchFlippers;

/**
 * Example OpMode demonstrating the usage of the LaunchFlippers subsystem.
 *
 * Controls:
 * - Gamepad1.A: Trigger left flipper
 * - Gamepad1.B: Trigger right flipper
 * - Gamepad1.X: Trigger both flippers
 * - Gamepad1.Y: Emergency stop (retract all)
 * - Gamepad1.DPAD_UP: Increase flip hold time
 * - Gamepad1.DPAD_DOWN: Decrease flip hold time
 * - Gamepad1.DPAD_LEFT: Reset flip hold time to default
 * - Gamepad1.DPAD_RIGHT: Toggle servo travel time adjustment mode
 * - Gamepad1.LEFT_BUMPER: Increase servo travel time (when in adjustment mode)
 * - Gamepad1.RIGHT_BUMPER: Decrease servo travel time (when in adjustment mode)
 */
@TeleOp(name = "LaunchFlippers Test", group = "Test")
public class LaunchFlippersTestOpMode extends OpMode {

    private LaunchFlippers flippers;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;

    private boolean adjustTravelTime = false; // Toggle for which timing to adjust

    @Override
    public void init() {
        // Initialize the flippers subsystem
        flippers = new LaunchFlippers(hardwareMap);

        // Optional: Configure timing (uncomment to customize)
        // flippers.setFlipHoldTime(0.3);

        // Optional: Reverse servo directions if needed (uncomment if needed)
        // flippers.setLeftReversed(true);
        // flippers.setRightReversed(false);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press A/B/X to test flippers");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Button Controls with Edge Detection ---

        // Left flipper (A button)
        if (gamepad1.a && !aPressed) {
            flippers.trigger(LaunchFlippers.FlipperSide.LEFT);
            telemetry.addData("Action", "Left flipper triggered");
        }
        aPressed = gamepad1.a;

        // Right flipper (B button)
        if (gamepad1.b && !bPressed) {
            flippers.trigger(LaunchFlippers.FlipperSide.RIGHT);
            telemetry.addData("Action", "Right flipper triggered");
        }
        bPressed = gamepad1.b;

        // Both flippers (X button)
        if (gamepad1.x && !xPressed) {
            flippers.trigger(LaunchFlippers.FlipperSide.BOTH);
            telemetry.addData("Action", "Both flippers triggered");
        }
        xPressed = gamepad1.x;

        // Emergency stop (Y button)
        if (gamepad1.y && !yPressed) {
            flippers.emergencyStop();
            telemetry.addData("Action", "EMERGENCY STOP");
        }
        yPressed = gamepad1.y;

        // Timing adjustment controls
        if (gamepad1.dpad_up && !dpadUpPressed) {
            double newTime = flippers.getFlipHoldTime() + 0.05;
            flippers.setFlipHoldTime(newTime);
            telemetry.addData("Action", "Increased hold time to %.2f", newTime);
        }
        dpadUpPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            double newTime = flippers.getFlipHoldTime() - 0.05;
            flippers.setFlipHoldTime(newTime);
            telemetry.addData("Action", "Decreased hold time to %.2f", newTime);
        }
        dpadDownPressed = gamepad1.dpad_down;

        if (gamepad1.dpad_left && !dpadLeftPressed) {
            flippers.resetFlipHoldTime();
            telemetry.addData("Action", "Reset hold time to default");
        }
        dpadLeftPressed = gamepad1.dpad_left;

        // Toggle servo travel time adjustment mode
        if (gamepad1.dpad_right && !dpadRightPressed) {
            adjustTravelTime = !adjustTravelTime;
            telemetry.addData("Action", adjustTravelTime ?
                    "Servo travel time mode ON (use bumpers)" : "Hold time mode ON (use dpad)");
        }
        dpadRightPressed = gamepad1.dpad_right;

        // Servo travel time adjustments (when mode is active)
        if (adjustTravelTime) {
            if (gamepad1.left_bumper && !leftBumperPressed) {
                double newTime = flippers.getServoTravelTime() + 0.05;
                flippers.setServoTravelTime(newTime);
                telemetry.addData("Action", "Increased travel time to %.2f", newTime);
            }
            leftBumperPressed = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !rightBumperPressed) {
                double newTime = flippers.getServoTravelTime() - 0.05;
                flippers.setServoTravelTime(newTime);
                telemetry.addData("Action", "Decreased travel time to %.2f", newTime);
            }
            rightBumperPressed = gamepad1.right_bumper;
        }

        // --- CRITICAL: Update the flippers state machine ---
        // This MUST be called every loop for the automatic retract to work
        flippers.update();

        // --- Telemetry ---
        telemetry.addData("Status", flippers.getStatus());
        telemetry.addData("Busy", flippers.isBusy() ? "YES" : "NO");
        telemetry.addData("Left Busy", flippers.isLeftBusy() ? "YES" : "NO");
        telemetry.addData("Right Busy", flippers.isRightBusy() ? "YES" : "NO");
        telemetry.addData("Debug", flippers.getDebugInfo());
        telemetry.addData("Left Position", "%.3f", flippers.getLeftPosition());
        telemetry.addData("Right Position", "%.3f", flippers.getRightPosition());
        telemetry.addData("Hold Time", "%.2fs", flippers.getFlipHoldTime());
        telemetry.addData("Travel Time", "%.2fs", flippers.getServoTravelTime());
        telemetry.addData("Adjust Mode", adjustTravelTime ? "TRAVEL TIME" : "HOLD TIME");
        telemetry.addData("", ""); // Blank line
        telemetry.addData("Controls", "A=Left | B=Right | X=Both | Y=Stop");
        telemetry.addData("Hold Time", "DPAD↑↓ = Adjust | DPAD← = Reset");
        telemetry.addData("Travel Time", "DPAD→ = Toggle | Bumpers = Adjust");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean shutdown - retract flippers
        if (flippers != null) {
            flippers.emergencyStop();
        }
    }
}