package org.firstinspires.ftc.teamcode.TeleOp.P3Robot.old;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Dual-Operator P3 TeleOp - Driver + Operator Control Split
 *
 * DESIGN PHILOSOPHY:
 * This robot uses a rotating turret for aiming. Unlike fixed-shooter designs where
 * the entire robot must align to the target, this robot only needs the TURRET to aim.
 * Therefore:
 * - DRIVER: Focuses purely on positioning and intake (no aiming responsibility)
 * - OPERATOR: Handles all aiming via turret control (plus launching and feeding)
 *
 * This OpMode splits control responsibilities between two gamepads:
 * - DRIVER (Gamepad 1): Drivetrain and Intake
 * - OPERATOR (Gamepad 2): Turret, Launcher, and Feeder
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * GAMEPAD 1 - DRIVER CONTROLS
 * ═══════════════════════════════════════════════════════════════════════════
 * Left Stick:        Drive (Y) and Strafe (X)
 * Right Stick:       Turn (X) - Manual rotation
 * A:                 Toggle intake ON/OFF
 * B:                 Toggle intake REVERSE/OFF
 * X:                 Emergency stop all systems
 * Y:                 (Reserved for future use)
 * D-Pad:             Fine driving adjustments (reserved)
 * Left Bumper:       Switch to BLUE alliance (config override)
 * Right Bumper:      Switch to RED alliance (config override)
 * Start:             (Reserved for future use)
 * Back:              Toggle detailed telemetry display
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * GAMEPAD 2 - OPERATOR CONTROLS
 * ═══════════════════════════════════════════════════════════════════════════
 * Left Stick X:      Manual turret rotation (left/right)
 * Left Stick Press:  Enable turret auto-aim to vision target
 * Right Stick:       (Reserved for future use - possible manual aim override)
 * Right Trigger:     LAUNCH - Initiate automated shot sequence
 * Left Trigger:      Manual reverse indexer (unjam)
 * A:                 Set NEAR shot velocity preset
 * B:                 Set FAR shot velocity preset
 * X:                 Stop flywheels (manual override)
 * Y:                 Enable AUTO-TARGETING mode (vision-based velocity)
 * D-Pad Up:          Increase velocity by 100 ticks/sec
 * D-Pad Down:        Decrease velocity by 100 ticks/sec
 * D-Pad Left:        Turret rotate to left preset position
 * D-Pad Right:       Turret rotate to right preset position
 * Left Bumper:       Decrease turret speed (fine control)
 * Right Bumper:      Increase turret speed (coarse control)
 * Start:             Toggle flywheel spin mode (rapid-fire / battery-saver)
 * Back:              Reset turret to center position
 * Both Bumpers+Back: Clear turret jam fault
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

@TeleOp(name="P3: Dual Operator TeleOp", group=" _P3opmodes")
@Disabled
public class P3_DualOperatorTeleOp extends OpMode {



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Could add pre-match checks here
        // Example: verify turret is at center, flywheels at zero, etc.
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }



}
