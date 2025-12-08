package org.firstinspires.ftc.teamcode.Auto.GearGirls;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchIndexer;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;

/**
 * A simplified, explicit state-machine-based autonomous OpMode.
 * This structure is designed to be easy to read and follow for new programmers.
 * It uses dedicated helper methods for each of the four starting paths.
 *
 * @author Your Team Name
 * @version 004
 */
@Autonomous(name="GG AUTO: Score 3 (Simplified State Machine)", group="GGBot")
public class GGAutonomous_ScorePreload extends OpMode {

    // --- Subsystems ---
    private GGRobot robot;

    // --- OpMode State and Configuration ---
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    private GGRobotConstants.Location location = GGRobotConstants.Location.CLOSE;
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;

    // --- Master State Machine ---
    private enum AutonomousState { PRE_START, RUNNING_PATH, COMPLETE }
    private AutonomousState autonomousState = AutonomousState.PRE_START;

    // --- Path-Specific State Machines ---
    private enum RedCloseState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, PARK } private RedCloseState redCloseState = RedCloseState.DRIVE_TO_SCORE;

    private enum RedFarState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, PARK } private RedFarState redFarState = RedFarState.DRIVE_TO_SCORE;
    private enum BlueCloseState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, PARK }  private BlueCloseState blueCloseState = BlueCloseState.DRIVE_TO_SCORE;
    private enum BlueFarState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, PARK } private BlueFarState blueFarState = BlueFarState.DRIVE_TO_SCORE;

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
        if (gamepad1.x) { alliance = CommonConstants.Alliance.BLUE; }
        if (gamepad1.b) { alliance = CommonConstants.Alliance.RED; }
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
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_CLOSE : GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else { // FAR
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_FAR : GGRobotConstants.Waypoints.START_BLUE_FAR);
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
                if (alliance == CommonConstants.Alliance.RED) {
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
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_AWAY, 0.5, 0.25);
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
        // x= 8.3816 y= 12.0461 h= -25.3986
        switch (redFarState) {
            case DRIVE_TO_SCORE:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.5, 0.25)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    redFarState = RedFarState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.5, 0.25);
                if (!robot.feeder.isBusy()) {
                    if (shotsFired < 3) {
                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
                        if(robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.FAR)) {
                            shotsFired++;
                        }
                    } else {
                        // All 3 shots are fired. Turn off the launcher and move to the park state.
                        robot.launcher.setMotorVelocity(0, 0);
                        redFarState = RedFarState.PARK;
                    }
                }
                // While robot.feeder.isBusy() is true, we do nothing and let the shot finish.
                break;

            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_PARK, 0.5, 0.25)) {
                    // Once parking is complete, the entire autonomous routine is done.
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", redFarState);
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
        // x= 8.3816 y= 12.0461 h= -25.3986
        switch (blueFarState) {
            case DRIVE_TO_SCORE:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.5, 0.25)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    blueFarState = BlueFarState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                if (!robot.feeder.isBusy()) {
                    if (shotsFired < 3) {
                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
                        if(robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.FAR)) {
                            shotsFired++;
                        }
                    } else {
                        // All 3 shots are fired. Turn off the launcher and move to the park state.
                        robot.launcher.setMotorVelocity(0, 0);
                        blueFarState = BlueFarState.PARK;
                    }
                }
                // While robot.feeder.isBusy() is true, we do nothing and let the shot finish.
                break;

            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_PARK, 0.5, 0.25)) {
                    // Once parking is complete, the entire autonomous routine is done.
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", blueFarState);
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
