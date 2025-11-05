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

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LauncherMotors;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2025;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeUtil;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchIndexer;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;


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
 * Software Architecture:
 * This OpMode follows a state machine pattern for managing its subsystems, which ensures robust
 * and predictable behavior. Each major subsystem (Drive, Intake, Launcher, etc.) is encapsulated
 * in its own utility class in the org.firstinspires.ftc.teamcode.utilities package.
 *
 * The main loop() method delegates control logic to a series of private methods
 * (e.g., doDriveControls(), doLauncherControls()), each responsible for one
 * part of the robot. This keeps the main loop clean and organized. Real-time data is sent to the
 * Driver Station via the doTelemetry() method for debugging and monitoring.
 *
 */
@TeleOp(name = "Gear Girls Bot 1.1", group = " _ arealTeleop")
//@Disabled
public class GearGirlsBot1 extends OpMode {
    //Declare SubSystems
    private GGRobot robot;
    private double manualLauncherVelocity;

    private enum LauncherSystemState {
        IDLE,
        ACTIVE
    } private LauncherSystemState launcherSystemState = LauncherSystemState.IDLE;

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

    // Declare the launcherDistance variable using the enum from the constants file.
    private GGRobotConstants.LauncherDistance launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;
    public double distanceToGoal;
    public double calculatedTargetVelocity;
    /**
     * Code to run ONCE when the driver hits INIT
     */

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // --- ROBOT ---
        robot = new GGRobot(hardwareMap, telemetry);

        // -- initialize set positions
        robot.feeder.setLeftFeederPower(GGRobotConstants.Feeder.STOP_SPEED);
        robot.feeder.setRightFeederPower(GGRobotConstants.Feeder.STOP_SPEED); // Also initialize the right feeder

        // --- State Initialization ---
        launcherSystemState = LauncherSystemState.IDLE;
        intakeState = IntakeState.OFF;
        diverterDirection = DiverterDirection.CENTER;
        launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;

        // --- Tell the driver that initialization is complete.---
        telemetry.addData("Status", "Initialized");
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
        if (gamepad1.startWasPressed()) {
            robot.drive.pinpoint.resetPosAndIMU();
        }

        // 1. Get the distance to the goal by calling the new method on the robot object.
        distanceToGoal = robot.getDistanceToGoal();
        // 2. Get the calculated target velocity using the other new method.
        calculatedTargetVelocity = robot.getTargetVelocityForDistance(distanceToGoal);

        displayTelemetry();
    }


    /**
     * Controls the robot's drivetrain based on the gamepad inputs.
     * This method uses arcade drive, where the left stick controls translational movement
     * (forward/backward and strafing) and the right stick controls rotational movement (turning).
     * The inputs are scaled by the DRIVE_SPEED constant to adjust the overall speed.
     */
    private void handleDriveControls() {
        robot.drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,gamepad1.right_stick_y,GGRobotConstants.Drive.DRIVE_SPEED);
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
        // --- Intake Control Logic ---

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
                robot.intake.setDiverterLeft();
                break;
            case RIGHT:
                robot.intake.setDiverterRight();
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
        // --- Step 1: Handle Inputs and Update State ---
        // Handle the distance toggle.
//        if (gamepad1.dpadUpWasPressed()) {
//            if (launcherDistance == GGRobotConstants.LauncherDistance.CLOSE) {
//                launcherDistance = GGRobotConstants.LauncherDistance.FAR;
//            } else {
//                launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;
//            }
//        }
        // Handle the automatic distance toggle (gamepad1)
        if (gamepad1.dpadUpWasPressed()) {
            launcherDistance = (launcherDistance == GGRobotConstants.LauncherDistance.CLOSE) ?
                    GGRobotConstants.LauncherDistance.FAR : GGRobotConstants.LauncherDistance.CLOSE;
            // When toggling, update the manual velocity to match the new preset
            manualLauncherVelocity = launcherDistance.targetVelocity;
        }

        // --- NEW: Manual Velocity Control (gamepad2) ---
        // Increase target velocity by 100 RPM when d-pad up is pressed
        if (gamepad2.dpadUpWasPressed()) {
            manualLauncherVelocity += 10;
        }
        // Decrease target velocity by 100 RPM when d-pad down is pressed
        if (gamepad2.dpadDownWasPressed()) {
            manualLauncherVelocity -= 10;
        }
        // --- End of New Code ---

        // Press 'Y' to activate the launcher, 'B' to deactivate it.
        // Using separate 'if' statements makes the logic more robust.
        if (gamepad1.yWasPressed()) {
            launcherSystemState = LauncherSystemState.ACTIVE;
        }
        if (gamepad1.bWasPressed()) {
            launcherSystemState = LauncherSystemState.IDLE;
        }

        if (gamepad1.left_trigger > 0.2) {
            robot.feeder.setLeftFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        }
        if (gamepad1.right_trigger > 0.2) {
            robot.feeder.setRightFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        }

        if (gamepad2.aWasPressed()) {
            robot.feeder.setLeftFeederPower(1.0);
        }
        if (gamepad2.b) {
            robot.feeder.setRightFeederPower(1);
        }

        // --- Step 2: Perform Actions Based on State ---
        // This section reads the current state and commands the hardware.
        boolean isLauncherReady = false;
        switch (launcherSystemState) {
            case IDLE:
                robot.launcher.setMotorVelocity(0, 0);
                break;
            case ACTIVE:
                // Always command the motors to the target velocity.
                robot.launcher.setMotorVelocity(calculatedTargetVelocity, calculatedTargetVelocity);

               // robot.launcher.setMotorVelocity(launcherDistance.targetVelocity, launcherDistance.targetVelocity);

                // Check if the motors are currently ready for a shot.
                // This check happens every loop, so it knows if velocity has dropped.
                if (robot.launcher.getLeftMotorVelocity() > (calculatedTargetVelocity-25) && robot.launcher.getRightMotorVelocity() > (calculatedTargetVelocity-25)) {
                    isLauncherReady = true;
                }
                break;
        }

        // --- Step 3: Handle Feeder (Shot) Requests ---
        // A shot is only allowed if the system is active AND the motors are up to speed.
        //if (isLauncherReady) {
            if (gamepad1.leftBumperWasPressed()) {
                robot.feeder.triggerLeftFeeder();
            }
            if (gamepad1.rightBumperWasPressed()) {
                // Trigger the right feeder for a set duration
               robot.feeder.triggerRightFeeder();
            }
        //}

    }



    /**
     * Displays critical robot data on the driver station's telemetry screen.
     * This method is called repeatedly during the main loop to provide real-time feedback
     * to the drivers. It shows the current state of the launcher state machine, the selected
     * launch distance (close or far), the target velocity for the launcher motors, and the
     * actual current velocity of both the left and right launcher motors. This information
     * is crucial for debugging and monitoring the robot's performance during a match.
     */
    private void displayTelemetry() {
        telemetry.addData("--- Launcher ---", "");
        telemetry.addData("State", launcherSystemState);
        telemetry.addData("Distance Setting", launcherDistance);
        telemetry.addData("Target Velocity", launcherDistance.targetVelocity);
        telemetry.addData("MANUAL Target Velocity", "%.0f RPM", manualLauncherVelocity);
        telemetry.addData("MANUAL Target Velocity", manualLauncherVelocity);

        telemetry.addData("Left Velocity", "%.2f", robot.launcher.getLeftMotorVelocity());
        telemetry.addData("Right Velocity", "%.2f", robot.launcher.getRightMotorVelocity());

        telemetry.addData("--- Intake ---", "");
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Diverter Position", diverterDirection);
        telemetry.addData("distance to Goal ", distanceToGoal);
        telemetry.addData("Target Velocity", calculatedTargetVelocity);

        // This command sends all queued telemetry data to the Driver Station.
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