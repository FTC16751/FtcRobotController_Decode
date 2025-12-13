/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.TeleOp.GGRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;

/**
 * This class implements the TeleOp (driver-controlled) program for the "Gear Girls Bot 1.1" robot
 * for the FTC DECODE season. It is designed to be the primary program for controlling the robot
 * during a match.
 *
 * Control Scheme:
 * The robot's functions are mapped to Gamepad 1 for a single driver.
 *
 * Drivetrain:
 * - Left Stick (X/Y): Controls strafing (left/right) and forward/backward movement.
 * - Right Stick (X): Controls the robot's rotation (turning).
 *
 * Intake System:
 * - Button A: Toggles the intake motors between ON (collecting) and OFF.
 * - Button X: Toggles the intake motors between REVERSE (expelling) and OFF.
 * - D-pad Down: Toggles the intake diverter servo between the LEFT and RIGHT positions.
 * - D-pad Right: Sets the intake diverter servo to the CENTER position.
 *
 * Launcher System:
 * - Button Y: Activates the launcher, spinning the flywheels up to the target speed.
 * - Button B: Deactivates the launcher, stopping the flywheels.
 * - D-pad Up: Toggles the target launch distance between CLOSE and FAR, adjusting the flywheel speed.
 * - Left/Right Bumpers: Fires a game element by activating the left or right feeder servo, but only
 *   if the launcher is active and the flywheels are at the correct speed.
 *
 *
 * The main loop() method delegates control logic to a series of private methods
 * (e.g., doDriveControls(), doLauncherControls()), each responsible for one
 * part of the robot. This keeps the main loop clean and organized. Real-time data is sent to the
 * Driver Station via the doTelemetry() method for debugging and monitoring.
 *
 */
@TeleOp(name = "Gear Girls Telop (RUN ME)", group = " _GGopmodes")
//@Disabled
public class GearGirlsBot1 extends OpMode {
    //Declare SubSystems
    private GGRobot robot;
    private double manualLauncherVelocity;
    private GGRobotConstants.LauncherDistance launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;
    private GGRobotConstants.LauncherSystemState launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
    private GGRobotConstants.LauncherTargetingMode targetingMode = GGRobotConstants.LauncherTargetingMode.AUTO;
    double finalTargetVelocity = 0;
    private static final double kP_TURN = 0.03;  // tune on field

    private enum DiverterDirection {
        LEFT,
        RIGHT,
        CENTER;
    } private DiverterDirection diverterDirection = DiverterDirection.CENTER;

    private enum IntakeState {
        ON,
        OFF,
        REVERSE; // Add the new REVERSE state
    } private IntakeState intakeState = IntakeState.OFF;
    private static final double JOYSTICK_DEADBAND = 0.05;
    // --- NEW: Slew Rate Limiter Variables ---
    // This constant defines how much the motor power can change per second.
    // A value of 3.0 means it takes 1/3 of a second to go from 0% to 100% power.
    // Smaller values = smoother/slower ramp. Larger values = more responsive.
    private static final double SLEW_RATE_LIMIT = 3.0; // Units: Power per Second

    // Variables to store the previous loop's power commands
    private double prevSmoothedDrive = 0.0;
    private double prevSmoothedStrafe = 0.0;
    private double prevSmoothedTurn = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime(); // Timer to measure loop time
    // --- NEW: Constants for the Simple TX Auto-Align ---
    // This gain determines how aggressively the robot turns to correct its aim.
    // Start with a small value and tune it for your robot's weight and drivetrain.
    private static final double TX_ALIGN_KP = 0.04; // Proportional gain for tx alignment.

    // A deadband to prevent the robot from "buzzing" or oscillating when it's very close to the target.
    private static final double TX_ALIGN_TOLERANCE_DEG = 1.0; // The 1-degree tolerance you requested.
    private enum DriveMode { FIELD_CENTRIC, ARCADE }
    private DriveMode DRIVEMODE = DriveMode.ARCADE;
    double angleOnTarget = 0.0;
    /**
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {

        // --- ROBOT ---
        robot = new GGRobot(hardwareMap, telemetry);

        // Load the alliance that was saved by the Autonomous OpMode
        CommonConstants.Alliance alliance = SharedState.alliance; // This loads the value into the static variable.
        // 3. Tell the robot to configure its vision system for that alliance.
        robot.configureVisionForTeleOp(alliance);

        // -- initialize set positions
        robot.feeder.setLeftFeederPower(GGRobotConstants.Feeder.STOP_SPEED);
        robot.feeder.setRightFeederPower(GGRobotConstants.Feeder.STOP_SPEED); // Also initialize the right feeder

        // --- State Initialization ---
        launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
        intakeState = IntakeState.OFF;
        diverterDirection = DiverterDirection.CENTER;
        launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;

        loopTimer.reset(); // Start the timer in init

        // --- Tell the driver that initialization is complete.---
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", alliance);

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
     * - {@code doDriveControls()}: Translates gamepad stick inputs into drivetrain movement.
     * - {@code doDiverterControls()}: Manages the position of the intake diverter servo.
     * - {@code doIntakeControls()}: Controls the intake motors (on, off, reverse).
     * - {@code doLauncherControls()}: Manages the launcher flywheels and the firing mechanism.
     * - {@code doTelemetry()}: Sends real-time data back to the Driver Station for monitoring.
     *
     * This modular approach keeps the main loop clean and easy to understand, delegating the
     * complex logic for each function to its respective method.
     *
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // --- Update robot state machines ---
        robot.update();

        // --- Control Logic ---
        handleDriveControls();
        handleDiverterControls();
        handleIntakeControls();
        handleLauncherControls();
        handleCopilotControls();
        displayTelemetry();
    }



    /**
     * Controls the robot's drivetrain based on the gamepad inputs.
     * This method uses arcade drive, where the left stick controls translational movement
     * (forward/backward and strafing) and the right stick controls rotational movement (turning).
     * The inputs are scaled by the DRIVE_SPEED constant to adjust the overall speed.
     */
    private void handleDriveControls() {

        if (gamepad1.startWasPressed()) {
            robot.drive.pinpoint.resetPosAndIMU();
        }

        //i want to use the gamepad1.back button to toggle between using drivemode of arcadeDrive and fieldCentricDrive toogle drive mode should be within this telop
        if (gamepad1.backWasPressed()) {
            toggleDriveMode();
        }

        double driveInput  = -gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;
        boolean isSnappingToTarget = gamepad1.right_stick_button && robot.vision.isTargetVisible();
        double txError = robot.vision.getTargetAngleX();;

        if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
            angleOnTarget = 0.0;
        } else {
            angleOnTarget = TX_ALIGN_KP * txError;
        }
        if (isSnappingToTarget) {
            turnInput = angleOnTarget;
            telemetry.addData("TX Align", "ON | Error: %.1f deg", txError);
        } else {
            // normal right-stick turning
            turnInput = gamepad1.right_stick_x;
        }

//        if (isSnappingToTarget) {
//            //camera-relative auto-aim using just the 'tx' value.
//
//            // Get the angle error directly from the vision subsystem.
//            double txError = robot.vision.getTargetAngleX();
//
//            // Check if we are already within our tolerance.
//            if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
//                // We are aimed correctly, so don't turn.
//                turnInput = 0.0;
//            } else {
//                // We are not aimed. Calculate the turn power using the P-controller.
//                turnInput = TX_ALIGN_KP * txError;
//            }
//            telemetry.addData("TX Align", "ON | Error: %.1f deg", txError);
//
//        } else {
//            // use normal turning
//            turnInput = gamepad1.right_stick_x;
//            telemetry.addData("AutoAim", "OFF");
//        }



        // =========================================================================
        //robot.drive.fieldCentricDrive(strafeInput, driveInput, turnInput, 1.0);
        robot.drive.arcadeDrive(strafeInput, driveInput, turnInput, 0, 1.0);
        if (DRIVEMODE == DriveMode.ARCADE) {
            robot.drive.arcadeDrive(strafeInput, driveInput, turnInput, 0, 1.0);
        } else if (DRIVEMODE == DriveMode.FIELD_CENTRIC) {
            robot.drive.fieldCentricDrive(strafeInput, driveInput, turnInput, 1.0);
        }
    }
    private void toggleDriveMode() {
        if (DRIVEMODE == DriveMode.ARCADE) {
            DRIVEMODE = DriveMode.FIELD_CENTRIC;;
        } else {
            DRIVEMODE =DriveMode.ARCADE;
        }
    }

    /**
     * Manages the intake mechanism based on gamepad input.
     * This method uses a state machine (`intakeState`) to control the intake motors.
     * - Pressing the 'A' button on gamepad 1 toggles the intake between ON and OFF.
     * - Pressing the 'X' button on gamepad 1 toggles the intake between REVERSE (outtake) and OFF.
     * The motor power is set based on the resulting state (ON, OFF, or REVERSE) at the end of each loop cycle,
     * using predefined speeds from {@link GGRobotConstants.Intake}.
     */
    private void handleIntakeControls() {
        // Press 'a' to toggle the intake between ON and OFF.
        if (gamepad1.aWasPressed()) {
            // If the intake is ON, turn it OFF; otherwise, turn it ON.
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }

        // Press 'x' to toggle the intake between REVERSE and OFF.
        if (gamepad1.xWasPressed()) {
            intakeState = (intakeState == IntakeState.REVERSE) ? IntakeState.OFF : IntakeState.REVERSE;
        }

        // Set motor power based on the final state once per loop
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
     * Manages the intake diverter servo based on gamepad input.
     * The diverter is used to direct incoming game elements to either the left or right side of the robot,
     * or it can be centered.
     *
     * Controls:
     * - D-pad Down: Toggles the diverter's target position between {@code LEFT} and {@code RIGHT}.
     * - D-pad Right: Sets the diverter's target position to {@code CENTER}.
     *
     * The method updates the {@code diverterDirection} state variable and then sets the servo's
     * physical position based on the final state at the end of the method call. This ensures the
     * servo is only commanded once per loop cycle.
     */
    private void handleDiverterControls() {
        // Press D-pad Down to toggle between LEFT and RIGHT
        if (gamepad1.dpadDownWasPressed()) {
            diverterDirection = (diverterDirection == DiverterDirection.LEFT) ?
                    DiverterDirection.RIGHT : DiverterDirection.LEFT;
        }
        // Press D-pad Right to center the diverter
        if (gamepad1.dpadRightWasPressed()) {
            diverterDirection = DiverterDirection.CENTER;
        }

        // Set the position based on the final state once per loop
        switch (diverterDirection) {
            case LEFT:
                robot.intake.setDiverterRight();
                break;
            case RIGHT:
                robot.intake.setDiverterLeft();
                break;
            case CENTER:
                robot.intake.setDiverterCenter();
                break;
        }

    }

    /**
     * Manages all driver controls for the launcher system.
     *
     * This method handles the following actions based on gamepad 1 input:
     *
     *   Y Button: Spins up the launcher flywheels to the currently selected target velocity
     *       (either CLOSE or FAR).
     *   B Button: Stops the launcher flywheels.
     *   D-pad Up (pressed): Toggles the launcher's target distance between CLOSE and FAR,
     *       updating the target velocity and minimum required velocity for launching accordingly.
     *   Left/Right Bumpers: Initiates the launch sequence for the left and right sides,
     *       respectively, by calling the {@code handleLaunch} state machine.
     */
    private void handleLauncherControls() {
        // --- STEP 1: HANDLE DRIVER INPUTS TO CHANGE STATES AND VALUES ---

        // Press D-Pad Left to cycle between AUTO and PRESET targeting modes.
        if (gamepad1.dpadLeftWasPressed()) {
            targetingMode = (targetingMode == GGRobotConstants.LauncherTargetingMode.AUTO) ?
                    GGRobotConstants.LauncherTargetingMode.PRESET : GGRobotConstants.LauncherTargetingMode.AUTO;
        }

        // Toggle between presets CLOSE or FAR
        if (gamepad1.dpadUpWasPressed()) {
            launcherDistance = (launcherDistance == GGRobotConstants.LauncherDistance.CLOSE) ?
                    GGRobotConstants.LauncherDistance.FAR : GGRobotConstants.LauncherDistance.CLOSE;
        }

        // Press 'Y' to activate the launcher, 'B' to deactivate it.
        if (gamepad1.yWasPressed()) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.ACTIVE;
        }
        if (gamepad1.bWasPressed()) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
        }

        // --- STEP 2:Delegate all orchestration to the GGRobot class ---
        // Call the new master method, passing it the current state of our OpMode.
        // The robot will figure out what to do and command the hardware.
        finalTargetVelocity = robot.updateLauncher(launcherSystemState, targetingMode, launcherDistance);


        // --- STEP 3: Handle Firing Requests (This logic stays in the OpMode) ---
        final double LAUNCHER_VELOCITY_TOLERANCE_RPM = 125.0;
        final double MINIMUM_SAFE_VELOCITY = 500.0; // A new constant: launcher isn't "ready" unless it's at least this fast.

        boolean isSpeedCorrect = (robot.launcher.getLeftMotorVelocity() >= (finalTargetVelocity - LAUNCHER_VELOCITY_TOLERANCE_RPM));
        boolean isSpeedSafe = (robot.launcher.getLeftMotorVelocity() > MINIMUM_SAFE_VELOCITY);

        // A launcher is only truly ready if the system is ACTIVE, the speed is within tolerance, AND the speed is above a safe minimum.
        boolean isLauncherReady = (launcherSystemState == GGRobotConstants.LauncherSystemState.ACTIVE) && isSpeedCorrect && isSpeedSafe;

        if (gamepad1.left_trigger > 0.2) {
            robot.feeder.setLeftFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        } else if (gamepad1.right_trigger > 0.2) {
            robot.feeder.setRightFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        } else {
            // A shot is only allowed if the system is active AND the motors are up to speed.
            if (isLauncherReady) {
                if (gamepad1.leftBumperWasPressed()) {
                    robot.feeder.triggerLeftFeeder();
                }
                if (gamepad1.rightBumperWasPressed()) {
                    robot.feeder.triggerRightFeeder();
                }
            }
        }

    }

    private void handleCopilotControls() {
        // ---Manual Alliance Override ---
        if (gamepad2.x) { // Using 'x' for Blue
            if (SharedState.alliance != CommonConstants.Alliance.BLUE) {
                SharedState.alliance = CommonConstants.Alliance.BLUE;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("CO-PILOT OVERRIDE: Alliance switched to BLUE");
            }
        }

        if (gamepad2.b) { // Using 'b' for Red
            if (SharedState.alliance != CommonConstants.Alliance.RED) {
                SharedState.alliance = CommonConstants.Alliance.RED;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("CO-PILOT OVERRIDE: Alliance switched to RED");
            }
        }

        if(gamepad2.start) {
            robot.resetOdometryToVision();
            telemetry.addLine("CO-PILOT OVERRIDE: resetOdometryToVision");
        }
    }
    private void displayTelemetry() {
        telemetry.addLine("--- Drive ---");
        robot.drive.addTelemetry();
        telemetry.addLine("--- Launcher ---");
        telemetry.addData("Launcher State", launcherSystemState);
        telemetry.addData("TARGETING MODE", targetingMode)
                .addData("(D-Pad Left to Cycle)", "");
        telemetry.addLine();
        if (targetingMode == GGRobotConstants.LauncherTargetingMode.PRESET) {
            telemetry.addData("Preset Distance Setting", launcherDistance);
        }


        telemetry.addLine("--- Flywheels ---");
        telemetry.addData("Odometry Distance to Goal ", robot.getDistanceToGoal());
        telemetry.addData("Odometry Distance to Goal FIELD RELATIVE ", robot.getDistanceToGoalFieldRelative());

        telemetry.addData("Target Velocity", "%.0f RPM", finalTargetVelocity);
        telemetry.addData("Left Velocity", "%.2f", robot.launcher.getLeftMotorVelocity());
        telemetry.addData("Right Velocity", "%.2f", robot.launcher.getRightMotorVelocity());

        telemetry.addLine("--- Intake ---");
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Diverter Position", diverterDirection);
        // This command sends all queued telemetry data to the Driver Station.
        robot.vision.addTelemetry();
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stopAll();
    }

}