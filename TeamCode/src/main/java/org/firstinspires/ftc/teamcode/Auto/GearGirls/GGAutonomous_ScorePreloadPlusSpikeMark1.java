package org.firstinspires.ftc.teamcode.Auto.GearGirls;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchIndexer;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil;

/**
 * A simplified, explicit state-machine-based autonomous OpMode.
 * This structure is designed to be easy to read and follow for new programmers.
 * It uses dedicated helper methods for each of the four starting paths.
 *
 * @author Your Team Name
 * @version 004
 */
@Autonomous(name="GG AUTO: Score 6", group="GGBot")
public class GGAutonomous_ScorePreloadPlusSpikeMark1 extends OpMode {

    // --- Subsystems ---
    private GGRobot robot;

    // --- OpMode State and Configuration ---
    private GGRobotConstants.Alliance alliance = GGRobotConstants.Alliance.RED;
    private GGRobotConstants.Location location = GGRobotConstants.Location.CLOSE;
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;

    // --- Master State Machine ---
    private enum AutonomousState { PRE_START, RUNNING_PATH, COMPLETE }
    private AutonomousState autonomousState = AutonomousState.PRE_START;

    // --- Path-Specific State Machines ---
    private enum RedCloseState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, PARK }
    private RedCloseState redCloseState = RedCloseState.DRIVE_TO_SCORE;

    private enum RedFarState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, PARK }
    private RedFarState redFarState = RedFarState.DRIVE_TO_SCORE;

    // (Similar enums for Blue paths would go here)
    private enum BlueCloseState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3, WAIT_A_BIT, GO_TO_BLUE_CLOSE_SPIKEMARK1_END, DRIVE_TO_SCORE_AFTER_SPIKE_MARK1, SHOOT_SEQUENCE2, GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN, WAIT_A_LITTLE, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3a, PARK }
    private BlueCloseState blueCloseState = BlueCloseState.DRIVE_TO_SCORE;
    // private enum BlueFarState { ... }

    // --- Action-Specific Variables ---
    private int shotsFired = 0;

    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot(hardwareMap, telemetry);
        robot.vision.setPipeline(0); // Ensure correct pipeline is active
        telemetry.addData(">", "Robot Initialized. Ready for selections.");
    }

    @Override
    public void init_loop() {
        robot.vision.setMotifDetectionMode();
        robot.vision.update(); // Continuously update vision to detect the motif

        // --- Driver Selections ---
        if (gamepad1.x) { alliance = GGRobotConstants.Alliance.BLUE; }
        if (gamepad1.b) { alliance = GGRobotConstants.Alliance.RED; }
        if (gamepad1.y) { location = GGRobotConstants.Location.CLOSE; }
        if (gamepad1.a) { location = GGRobotConstants.Location.FAR; }

        // --- Vision Detection ---
        detectedMotif = robot.vision.getMotifPattern();

        // --- Telemetry Feedback ---
        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addData("Detected Motif", detectedMotif);
        telemetry.addLine("\nReady to Start!");
        telemetry.update();
    }

    @Override
    public void start() {
        // Set the robot's starting position based on the final selections
        if (location == GGRobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_CLOSE : GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else { // FAR
            robot.drive.pinpoint.setPosition((alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_FAR : GGRobotConstants.Waypoints.START_BLUE_FAR);
        }

        // Set the pipeline to ONLY look for our alliance's scoring tags.
        robot.vision.setTargetingAlliance(alliance);

        // Start spinning up the launcher motor immediately.
        robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY, GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY);

        // Transition to the main execution state
        autonomousState = AutonomousState.RUNNING_PATH;
    }

    //================================================================================
    // MAIN LOOP
    //================================================================================

    @Override
    public void loop() {
        robot.update();

        switch (autonomousState) {
            case RUNNING_PATH:
                robot.intake.setIntakeMotorPower(1);
                if (alliance == GGRobotConstants.Alliance.RED) {
                    if (location == GGRobotConstants.Location.CLOSE) {
                        runRedClosePath();
                    } else { // FAR
                        runRedFarPath();
                    }
                } else { // BLUE
                    if (location == GGRobotConstants.Location.CLOSE) {
                        runBlueClosePath();
                    } else { // FAR
                        runBlueFarPath();
                    }
                }
                break;

            case COMPLETE:
                // Path is done. Stop all motors and end the OpMode.
                robot.stopAll();
                requestOpModeStop();
                break;
        }

        // Add telemetry from the robot object
        robot.addTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save the selected alliance for TeleOp to use.
        SharedState.alliance = this.alliance;
        if (robot != null) {
            robot.stopAll();
        }
    }

    //================================================================================
    // PATH-SPECIFIC HELPER METHODS
    //================================================================================

    /**
     * Runs the state machine for the RED alliance, CLOSE starting position.
     * This method contains the "script" for this specific path.
     */
    private void runRedClosePath() {
        telemetry.addData("Current Path", "Red Close");

        switch (redCloseState) {
            case DRIVE_TO_SCORE:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_AWAY, 0.5, 0.25)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    redCloseState = RedCloseState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                // This state handles firing all three pre-loaded artifacts.
                // We keep the launcher spinning throughout the sequence.
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY, GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY);

                // Only start a new shot if the previous one is finished.
                //if (!robot.feeder.isBusy()) {
                    if (shotsFired < 3) {
                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
                        if(robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.AUTO)) {
                            shotsFired++;
                        }
                    } else {
                        // All 3 shots are fired. Turn off the launcher and move to the park state.
                        robot.launcher.setMotorVelocity(0, 0);
                        redCloseState = RedCloseState.PARK;
                    }
                //}
                // While robot.feeder.isBusy() is true, we do nothing and let the shot finish.
                break;

            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_PARK, 0.5, 0.25)) {
                    // Once parking is complete, the entire autonomous routine is done.
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", redCloseState);
    }

    /**
     * Runs the state machine for the RED alliance, FAR starting position.
     */
    private void runRedFarPath() {
        telemetry.addData("Current Path", "Red Far");
        // This is a placeholder. You would implement the logic for the Red Far path here,
        // using the redFarState enum and the RED_FAR waypoints from GGRobotConstants.
        // For now, we'll just mark it as complete.
        autonomousState = AutonomousState.COMPLETE;
    }


    private void runBlueClosePath() {
        telemetry.addData("Current Path", "Blue Close");

        switch (blueCloseState) {
            case DRIVE_TO_SCORE:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY, 0.5, 0.25)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY, 0.5, 0.25);
                // This state handles firing all three pre-loaded artifacts.
                // We keep the launcher spinning throughout the sequence.
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY, GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY);

                // Only start a new shot if the previous one is finished.
                if(!robot.feeder.isBusy()) {
                    if (shotsFired < 3) {
                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
                        if (robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.AUTO)) {
                            shotsFired++;
                        }
                    } else {
                        // All 3 shots are fired. Turn off the launcher and move to the park state.
                        robot.launcher.setMotorVelocity(0, 0);
                        blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN;
                    }
                }
                break;

            case GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_ALIGN, 0.5, 0.25)) {
                    robot.intake.setIntakeMotorPower(1);
                    robot.intake.setDiverterLeft();
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1;
                }
                break;
            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL1, 0.5, 0.25)) {
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2;
                }
                break;
            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL2, 0.25, 0.25)) {
                    robot.feeder.reverseLeftFeeder();
                    robot.intake.setDiverterCenter();
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3a;
                }
                break;
            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3a:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL3a, 0.25, 0.25)) {
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3;
                }
                break;
            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL3, 0.25, 0.25)) {
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_END;
                }
                break;
            case GO_TO_BLUE_CLOSE_SPIKEMARK1_END:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_END, 0.25, 0.25)) {
                    // Once parking is complete, the entire autonomous routine is done.
                    blueCloseState = BlueCloseState.DRIVE_TO_SCORE_AFTER_SPIKE_MARK1;
                }
                break;
            case DRIVE_TO_SCORE_AFTER_SPIKE_MARK1:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY, 0.5, 0.25)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE2;
                }
                break;
            case SHOOT_SEQUENCE2:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY, 0.5, 0.25);
                // This state handles firing all three pre-loaded artifacts.
                // We keep the launcher spinning throughout the sequence.
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY, GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY);

                // Only start a new shot if the previous one is finished.
                if (shotsFired < 3) {
                    LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
                    if(robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.AUTO)) {
                        shotsFired++;
                    }
                } else {
                    // All 3 shots are fired. Turn off the launcher and move to the park state.
                    robot.launcher.setMotorVelocity(0, 0);
                    blueCloseState = BlueCloseState.PARK;
                }
                break;
            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

        }
        telemetry.addData("Path State", blueCloseState);
    }

    /**
     * Runs the state machine for the BLUE alliance, FAR starting position.
     */
    private void runBlueFarPath() {
        telemetry.addData("Current Path", "Blue Far");
        // This is a placeholder. You would implement the logic for the Red Far path here,
        // using the redFarState enum and the RED_FAR waypoints from GGRobotConstants.
        // For now, we'll just mark it as complete.
        autonomousState = AutonomousState.COMPLETE;
    }
    /**
     * A helper method to determine which side to shoot from based on the shot number and motif.
     * @param shotNumber The shot we are on (0, 1, or 2).
     * @param motif The detected vision pattern.
     * @return The FeederSide to use for the shot.
     */
    private LaunchIndexer.FeederSide getSideForShot(int shotNumber, VisionUtil.MotifPattern motif) {
        // This logic determines the shot order based on the pre-load and vision result.
        // Assuming Green is pre-loaded on the RIGHT, Purple on the LEFT.
        switch (motif) {
            case GPP21: // Green, Purple, Purple
                if (shotNumber == 0) return LaunchIndexer.FeederSide.RIGHT;
                else return LaunchIndexer.FeederSide.LEFT;
            case PGP22: // Purple, Green, Purple
                if (shotNumber == 1) return LaunchIndexer.FeederSide.RIGHT;
                else return LaunchIndexer.FeederSide.LEFT;
            case PPG23: // Purple, Purple, Green
            case UNKNOWN: // Default to a safe order if vision fails
            default:
                if (shotNumber == 2) return LaunchIndexer.FeederSide.RIGHT;
                else return LaunchIndexer.FeederSide.LEFT;
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
