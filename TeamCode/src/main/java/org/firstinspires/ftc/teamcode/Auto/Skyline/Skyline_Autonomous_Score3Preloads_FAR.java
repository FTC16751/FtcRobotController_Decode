/*
 * Copyright (c) 2025 Base 10 Assets, LLC
 * All rights reserved.
 * ... (copyright header remains the same)
 */

package org.firstinspires.ftc.teamcode.Auto.Skyline;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.Skyline.Skyline_Robot;

/*
 * This file includes an autonomous routine for the Skyline robot.
 * It uses the Skyline_Robot class to orchestrate complex actions like launching and driving.
 * This OpMode is only responsible for the high-level sequence of events.
 */

@Autonomous(name="SKYLINE: Score 3 Preloads Far", group="SkylineBot",preselectTeleOp="SKYLINE: Teleop (V2 RUN ME)")
public class Skyline_Autonomous_Score3Preloads_FAR extends OpMode
{
    // --- Main Robot Object ---
    private Skyline_Robot robot;

    // --- Autonomous Constants ---
    // Constants for THIS specific path.
    private static final double LAUNCHER_TARGET_VELOCITY = 1680;
    private static final double LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY-25;
    private static final double FEED_TIME = .40; // Adjusted feed time
    private static final double DRIVE_SPEED = 0.5;
    private static final double ROTATE_SPEED = 0.2;

    // --- State Variables ---
    private int shotsToFire = 3;

    private enum AutonomousState {
        // --- UPDATED STATE MACHINE ---
        REQUEST_SHOT,           // State to begin a new shot sequence
        WAIT_FOR_SHOT_COMPLETION, // State to wait for the launchSequence to finish
        // -----------------------------
        TURN_TO_FACE_GOAL,
        DRIVE_TO_PARK,
        DRIVING_OFF_LINE,
        COMPLETE
    }
    private AutonomousState autonomousState;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    /*
     * Code to run ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        // Initialize the robot object. It handles all hardware setup.
        robot = new Skyline_Robot(hardwareMap, telemetry);

        // Set the starting state for our autonomous routine
        autonomousState = AutonomousState.TURN_TO_FACE_GOAL;

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        // Allow the driver to select the alliance before the match starts
        if (gamepad1.b) { alliance = Alliance.RED; }
        if (gamepad1.x) { alliance = Alliance.BLUE; }

        telemetry.addData("Press X for BLUE, B for RED", "");
        telemetry.addData("Selected Alliance", alliance);
    }

    /*
     * Code to run ONCE when the driver hits START.
     */
    @Override
    public void start() {
        // Any setup that needs to happen right at the start can go here.
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        // Always update the robot's internal state machines first
        robot.update();

        // Run the main autonomous state machine
        switch (autonomousState){
            case TURN_TO_FACE_GOAL:
                // Note: The drive() and rotate() methods would ideally be moved into DriveUtil2026
                // and be non-blocking. For this example, we assume they exist as before.
                ;
                double robotRotationAngle = (alliance == Alliance.RED) ? 17 : -15;
                if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,1)){
                    autonomousState = AutonomousState.REQUEST_SHOT;
                }
                break;
            case REQUEST_SHOT:
                telemetry.addData("REQUEST SHOT", "Shot %d of 3 requested...", (4 - shotsToFire));

                // This state runs only ONCE per shot.
                if (shotsToFire > 0) {
                    // Tell the robot to begin its launch sequence.
                    // We only need to send the 'true' command once.
                    robot.launchSequence(true, LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY, FEED_TIME);

                    telemetry.addData("Shooting", "Shot %d of 3 requested...", (4 - shotsToFire));

                    // Immediately move to the waiting state.
                    autonomousState = AutonomousState.WAIT_FOR_SHOT_COMPLETION;
                } else {
                    telemetry.addData("DONE SHOOTING", "Shot %d of 3 requested...", (4 - shotsToFire));

                    // All shots have been fired, move on to driving.
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case WAIT_FOR_SHOT_COMPLETION:
                telemetry.addData("WAIT FOR SHOT COMPLETION", "Shot %d of 3 requested...", (4 - shotsToFire));

                // In this state, we continuously "tick" the launch sequence by calling it
                // with 'false'. It will return 'true' for one cycle when it's done.
                boolean isShotComplete = robot.launchSequence(false, LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY, FEED_TIME);

                if (isShotComplete) {
                    telemetry.addData("SHOT COMPLETE", "Shot %d of 3 requested...", (4 - shotsToFire));

                    // The shot is finished! Decrement the counter and go back to request the next one.
                    shotsToFire--;
                    autonomousState = AutonomousState.REQUEST_SHOT;
                }
                // If not complete, we do nothing and just stay in this state, letting the sequence run.
                break;




            case DRIVING_OFF_LINE:
                if(drive(DRIVE_SPEED, 12, DistanceUnit.INCH, 1)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                robot.stopAll();
                requestOpModeStop();
                break;
        }

        telemetry.addData("Auto State", autonomousState);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
        requestOpModeStop();
        robot.stopAll();

    }

    // These drive methods are still here for now, but should ideally be moved
    // into DriveUtil2026 as non-blocking actions.
    boolean drive(double speed, double distance, DistanceUnit unit, double holdTime) {
        robot.drive.drive_p3(distance,0,0,speed);
        return true; // Placeholder
    }
    boolean rotate(double speed, double angle, AngleUnit unit, double holdTime) {
        robot.drive.drive_p3(0,0,angle,speed);
        return true; // Placeholder
    }
}
