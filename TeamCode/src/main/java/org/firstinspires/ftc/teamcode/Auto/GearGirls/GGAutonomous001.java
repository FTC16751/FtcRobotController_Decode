/*
 * Copyright (c) 2025 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auto.GearGirls;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchIndexer.FeederSide;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="GG Sample Decode Auto", group="GGBot")
//@Disabled
public class GGAutonomous001 extends OpMode
{
    private static final double LAUNCHER_TARGET_VELOCITY = GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
    private static final double LAUNCHER_MIN_VELOCITY = GGRobotConstants.Launcher.CLOSE_MIN_VELOCITY;
    private GGRobot robot;
    private VisionUtil vision;


    /*
     * The number of seconds that we wait between each of our 3 shots from the launcher. This
     * can be much shorter, but the longer break is reasonable since it maximizes the likelihood
     * that each shot will score.
     */
    final double TIME_BETWEEN_SHOTS = 2;
    final double FEED_TIME = 0.20; //The feeder servos run this long when a shot is requested.


    int shotsToFire = 2; //The number of shots to fire in this auto.
    // Define the sequence of shots for this specific autonomous routine.
// This is now a "playlist" of shots. It's easy to read and modify.
    private final List<FeederSide> shotSequence = new ArrayList<>(Arrays.asList(
            FeederSide.LEFT,
            FeederSide.LEFT,
            FeederSide.RIGHT
    ));
    // This variable will track which shot we are currently on.
    private int currentShotIndex = 0;
    /*
     * Here we create three timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();


    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "LaunchState." It reflects the current condition of the shooter motor when we request a shot.
     * It starts at IDLE. When a shot is requested from the user, it'll move into WAIT_FOR_SPEED_AND_FIRE then COOLDOWN.
     * We can use higher level code to cycle through these states, but this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits."
     */
    private enum LaunchState {
        IDLE,
        WAIT_FOR_SPEED_AND_FIRE,
        COOLDOWN,
    }

    /*
     * Here we create the instance of LaunchState that we use in code. This creates a unique object
     * which can store the current condition of the shooter. In other applications, you may have
     * multiple copies of the same enum which have different names. Here we just have one.
     */
    private LaunchState launchState;

    private enum AutonomousState {
        START_DRIVING_AWAY_FROM_GOAL,
        WAIT_FOR_DRIVE_TO_FINISH,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_OFF_LINE,
        COMPLETE, START_INITIAL_DRIVE, START_ALIGNING_TO_TAG, WAIT_FOR_ALIGNMENT;
    }
    private AutonomousState autonomousState;

    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE;
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;
    private boolean b_pressed = false;
    private boolean x_pressed = false;


    // This is the NEW, CORRECT declaration
    private VisionUtil.MotifPattern detectedMotifPattern = VisionUtil.MotifPattern.UNKNOWN;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        // -- Robot --
        robot = new GGRobot(hardwareMap, telemetry);

        vision = new VisionUtil(hardwareMap, "limelight");

        robot.drive.resetHeading();  // Reset heading to set a baseline for Auto

        autonomousState = AutonomousState.START_INITIAL_DRIVE;
        launchState = LaunchState.IDLE;

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        // Check for the "rising edge" of the 'b' button
        if (gamepad1.b && !b_pressed) {
            alliance = Alliance.RED;
        }
        b_pressed = gamepad1.b; // Update the state for the next loop

        // Check for the "rising edge" of the 'x' button
        if (gamepad1.x && !x_pressed) {
            alliance = Alliance.BLUE;
        }
        x_pressed = gamepad1.x; // Update the state for the next loop

        // CRITICAL: Always update the vision utility in every loop cycle.
        vision.update();
        // The OpMode doesn't need to know about tag IDs (21, 22, 23) anymore!
        detectedMotifPattern = vision.getMotifPattern();
        // Build the shot sequence based on the result.
        shotSequence.clear();

        switch (detectedMotifPattern) {
            case GPP21:
                shotSequence.addAll(Arrays.asList(FeederSide.RIGHT, FeederSide.LEFT, FeederSide.LEFT));
                break;
            case PGP22:
                shotSequence.addAll(Arrays.asList(FeederSide.LEFT, FeederSide.RIGHT, FeederSide.LEFT));
                break;
            case PPG23:
                shotSequence.addAll(Arrays.asList(FeederSide.LEFT, FeederSide.LEFT, FeederSide.RIGHT));
            case UNKNOWN: // Default path if RIGHT or if no tag is seen
            default:
                shotSequence.addAll(Arrays.asList(FeederSide.RIGHT, FeederSide.LEFT, FeederSide.LEFT));
                break;
        }

        telemetry.addData("Detected Motif Pattern", detectedMotifPattern);
        telemetry.addData("Planned Shot Sequence", shotSequence.toString());
//      telemetry.update();


        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.update();
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {

    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        robot.drive.update();
        robot.feeder.update();
        vision.update();
        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState) {
            case START_INITIAL_DRIVE:
                robot.drive.drive(-24, 0.5, 0.25);
                //drive.drive_p3(-48,0,0,.5);
                autonomousState = AutonomousState.LAUNCH;
                break;
            case START_ALIGNING_TO_TAG:
                robot.drive.driveToTagAsync(vision, 24, 12.0, 0.5); // This is non-blocking.
                autonomousState = AutonomousState.WAIT_FOR_ALIGNMENT;
                break;
            case WAIT_FOR_ALIGNMENT:
                // We wait here until it's done.
                if (!robot.drive.isBusy()) {
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;
            case LAUNCH:
                // Check if we still have shots left in our sequence.
                if (currentShotIndex < shotSequence.size()) {
                    // Get the side for the CURRENT shot from our "playlist".
                    FeederSide nextShot = shotSequence.get(currentShotIndex);
                    //start launch sequence
                    launch(true, nextShot);
                    autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                } else {
                    // No more shots in the sequence, move on.
                    robot.launcher.setMotorVelocity(0, 0);
                    robot.intake.setIntakeMotorPower(0);
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case WAIT_FOR_LAUNCH:
                // Get the side for the shot we are currently waiting for.
                FeederSide currentShot = shotSequence.get(currentShotIndex);

                // Call the launch state machine. It will return true once a shot is fully complete
                boolean isShotComplete = launch(false, currentShot);
                if (isShotComplete) {
                    // A shot was just completed.
                    // Move to the next shot in the sequence.
                    currentShotIndex++;
                    // Go back to the LAUNCH state to either fire the next shot
                    // or determine that the sequence is finished.
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;

            case DRIVING_OFF_LINE:
                //robot.drive.strafe(6,.5,.25);
                robot.drive.driveRobotDistanceStrafeLeftInches(-10,.5);
                autonomousState = AutonomousState.COMPLETE;
                break;

            case COMPLETE:
                robot.drive.stopRobot();
                break;
        }
        robot.feeder.update();

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it will run once every loop.
         * No matter what state our robot is in. This is the huge advantage of using state machines.
         * We can have code inside of our state machine that runs only when necessary, and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         */
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.update();
    }

    /*
    * This code runs ONCE after the driver hits STOP.
    */
    @Override
    public void stop () {

    }
    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the next ball.
     * @param shotRequested "true" if the user would like to fire a new shot, and "false" if a shot
     *                      has already been requested and we need to continue to move through the
     *                      state machine and launch the ball.
     * @return "true" for one cycle after a ball has been successfully launched, "false" otherwise.
     */
    boolean launch(boolean shotRequested, FeederSide sideToFire){

        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    // When a shot is requested, start spinning up the launcher motors
                    robot.launcher.setMotorVelocity(LAUNCHER_TARGET_VELOCITY, LAUNCHER_TARGET_VELOCITY);
                    launchState = LaunchState.WAIT_FOR_SPEED_AND_FIRE;
                }
                break;
            case WAIT_FOR_SPEED_AND_FIRE:
                // In the WAIT_FOR_SPEED_AND_FIRE state, we wait for the launcher motors to reach the minimum speed.
                if (robot.launcher.getLeftMotorVelocity() > LAUNCHER_MIN_VELOCITY && robot.launcher.getRightMotorVelocity() > LAUNCHER_MIN_VELOCITY){
                    // Once at speed, trigger the feeder.
                    // The feeder's internal timer and update() method will handle the rest.
                    robot.intake.setIntakeMotorPower(GGRobotConstants.Intake.INTAKE_SPEED);
                    // Use the 'sideToFire' parameter to trigger the correct feeder.
                    if (sideToFire == FeederSide.LEFT) {
                        robot.feeder.triggerLeftFeeder();
                    } else {
                        robot.feeder.triggerRightFeeder();
                    }
                    shotTimer.reset();
                    // Now we move to the COOLDOWN state to wait for the feeder to finish.
                    launchState = LaunchState.COOLDOWN;
                }
                break;
            case COOLDOWN:
                // Wait for the time between shots to elapse.
                // This allows the launcher to regain speed if necessary.
                if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    launchState = LaunchState.IDLE; // Ready for another shot request
                    return true; // Signal that the sequence is complete
                }
                break;
        }
        return false;
    }

}



