package org.firstinspires.ftc.teamcode.Auto.GearGirls;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.AutoAction;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchIndexer;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="GG Pinpoint Action Auto try 3", group="GGBot")
public class GGAutonomous003 extends OpMode {

    // --- Subsystems ---
    private GGRobot robot;
    private VisionUtil vision;

    private enum Alliance {
        RED,
        BLUE;
    }
    private Alliance alliance = Alliance.RED;
    private boolean b_pressed = false;
    private boolean x_pressed = false;

    private enum Location {
        CLOSE,
        FAR;
    }
    private Location location = Location.FAR;
    private boolean y_pressed = false;
    private boolean a_pressed = false;

    // --- State Machine and Action Playlist ---
    private enum AutonomousState {
        EXECUTE_SEQUENCE,
        COMPLETE
    }
    private AutonomousState autonomousState = AutonomousState.EXECUTE_SEQUENCE;
    private List<AutoAction> actionSequence = new ArrayList<>();
    private int currentActionIndex = 0;
    AutoAction currentAction = null;// = actionSequence.get(currentActionIndex);

    // --- Game-Specific Variables ---
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;
    private VisionUtil.MotifPattern lastDetectedMotif = VisionUtil.MotifPattern.UNKNOWN;

    // --- Define Autonomous Waypoints ---
    private static final Pose2D START_POS = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
    private static final Pose2D DRIVE_AWAY_FROM_GOAL_RED = new Pose2D(DistanceUnit.INCH,5.7963, 30.3506, AngleUnit.DEGREES,-95);
    private static final Pose2D STRAFE_OFF_SHOT_LINE1 = new Pose2D(DistanceUnit.INCH,22, 10, AngleUnit.DEGREES,-95);
    private static final Pose2D STRAFE_OFF_SHOT_LINE2 = new Pose2D(DistanceUnit.INCH,22, 10, AngleUnit.DEGREES,-95);

    private static final Pose2D DRIVE_AWAY_FROM_GOAL_BLUE = new Pose2D(DistanceUnit.INCH,5.7963, -30.3506, AngleUnit.DEGREES,95);
    private static final Pose2D STRAFE_OFF_SHOT_LINE_BLUE = new Pose2D(DistanceUnit.INCH,22, -10, AngleUnit.DEGREES,95);

    private static final Pose2D DRIVE_AWAY_FROM_FAR_WALL_BLUE = new Pose2D(DistanceUnit.INCH,7.2, -5, AngleUnit.DEGREES,21);
    private static final Pose2D DRIVE_AWAY_FROM_FAR_lINE_BLUE = new Pose2D(DistanceUnit.INCH,30, 0, AngleUnit.DEGREES,0);
    private static final Pose2D DRIVE_AWAY_FROM_FAR_WALL_RED = new Pose2D(DistanceUnit.INCH,7.2, 0, AngleUnit.DEGREES,-21);
    private static final Pose2D DRIVE_AWAY_FROM_FAR_lINE_RED = new Pose2D(DistanceUnit.INCH,30, 0, AngleUnit.DEGREES,0);

    // Add other positions as needed...


    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot(hardwareMap, telemetry);
        vision = new VisionUtil(hardwareMap, "limelight");
        vision.setPipeline(0);
        buildActionSequence();
        telemetry.addData(">", "Robot Initialized. Detecting Motif...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        boolean selectionChanged = false; // A flag to track if we need to rebuild the plan

        // --- Alliance Selection ---
        if (gamepad1.xWasPressed() && alliance != Alliance.BLUE) {
            alliance = Alliance.BLUE;
            selectionChanged = true; // The selection changed, so we must rebuild.
        }
        if (gamepad1.bWasPressed() && alliance != Alliance.RED) {
            alliance = Alliance.RED;
            selectionChanged = true; // The selection changed, so we must rebuild.
        }

        // --- Location Selection ---
        if (gamepad1.yWasPressed() && location != Location.CLOSE) {
            location = Location.CLOSE;
            selectionChanged = true; // The selection changed, so we must rebuild.
        }
        if (gamepad1.aWasPressed() && location != Location.FAR) {
            location = Location.FAR;
            selectionChanged = true; // The selection changed, so we must rebuild.
        }

        // --- Vision Detection ---
        vision.update();
        VisionUtil.MotifPattern currentMotif = vision.getMotifPattern();
        if (currentMotif != detectedMotif) {
            detectedMotif = currentMotif;
            selectionChanged = true; // The vision result changed, so we must rebuild.
        }

        // --- Rebuild the Action Sequence IF NEEDED ---
        // This block runs if the vision result OR any driver selection has changed.
        if (selectionChanged) {
            buildActionSequence();
        }

        // --- Selections Display ---
        telemetry.addLine("--- Autonomous Plan ---");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Location", location);
        telemetry.addData("Detected Motif", detectedMotif);
        telemetry.addLine(); // Adds a blank line for spacing

        // --- Path Summary Display ---
        telemetry.addData("Planned Actions", actionSequence.size());

        // Safely show the first and last actions
        if (!actionSequence.isEmpty()) {
            telemetry.addData("1. First Action", actionSequence.get(0).description);
            telemetry.addData(actionSequence.size() + ". Last Action", actionSequence.get(actionSequence.size() - 1).description);
        } else {
            telemetry.addData("Status", "WARNING: No actions in plan!");
        }

        // --- Instructions Display ---
        telemetry.addLine();
        telemetry.addLine("--- Driver Selections ---");
        telemetry.addData("SELECT ALLIANCE: X = BLUE", "or B = RED");
        telemetry.addData("SELECT LOCATION: Y = CLOSE", "or A = FAR");
        telemetry.update();

    }

    @Override
    public void start() {
        // Reset the sequence and set the starting state
        currentActionIndex = 0;
        autonomousState = AutonomousState.EXECUTE_SEQUENCE;
        robot.launcher.setMotorVelocity(GGRobotConstants.LauncherDistance.AUTO.targetVelocity, GGRobotConstants.LauncherDistance.AUTO.targetVelocity);
    }

    //================================================================================
    // MAIN LOOP
    //================================================================================

    @Override
    public void loop() {
        // These updates are always required for non-blocking parts (like the feeder)
        robot.update();
        robot.drive.pinpoint.update();
        //vision.update();

        switch (autonomousState) {
            case EXECUTE_SEQUENCE:
                // Check if we are done with all actions.
                if (currentActionIndex >= actionSequence.size()) {
                    autonomousState = AutonomousState.COMPLETE;
                    break;
                }
                // Get the current action from our "playlist".
                currentAction = actionSequence.get(currentActionIndex);
                telemetry.addData("Executing Action", (currentActionIndex + 1) + ": " + currentAction.description);

                boolean isActionComplete = false;
                if (currentAction.type == AutoAction.ActionType.DRIVE_TO_POINT) {
                    if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), currentAction.targetPose, 0.5, .25)){
                        currentActionIndex++;
                    }
                } else if (currentAction.type == AutoAction.ActionType.SHOOT) {
                    if (location == Location.CLOSE) {
                        if (robot.launchSequence(true, currentAction.feederSide, GGRobotConstants.LauncherDistance.AUTO)) {
                            currentActionIndex++;
                        }
                    } else if (location == Location.FAR){
                        if (robot.launchSequence(true, currentAction.feederSide, GGRobotConstants.LauncherDistance.FAR)) {
                            currentActionIndex++;
                        }
                    }
                }
                break;

            case COMPLETE:
                robot.stopAll();
                requestOpModeStop(); // End the OpMode
                break;
        }

        telemetry.addData("Auto State", autonomousState);
        telemetry.addData("currentActionIndex ", currentActionIndex);
        // Add a null check for currentAction
        if (currentAction != null) {
            telemetry.addData("Current Action", currentAction.description);
            telemetry.addData("Action Type", currentAction.type);
        } else if (currentActionIndex < actionSequence.size()){
            telemetry.addData("Current Action", "Starting...");
        } else {
            telemetry.addData("Current Action", "Finished");
        }
        robot.addTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) vision.stop();
        if (robot != null) robot.stopAll();
    }

    //================================================================================
    // AUTONOMOUS PATH PLANNING
    //================================================================================

    private void buildActionSequence() {
        actionSequence.clear();

        // 1. Determine location-specific waypoints first
        final Pose2D driveAwayPose;
        final Pose2D parkPose;

        if (location == Location.CLOSE) {
            driveAwayPose = (alliance == Alliance.RED) ? DRIVE_AWAY_FROM_GOAL_RED : DRIVE_AWAY_FROM_GOAL_BLUE;
            parkPose = (alliance == Alliance.RED) ? STRAFE_OFF_SHOT_LINE1 : STRAFE_OFF_SHOT_LINE_BLUE;
        } else { // Handles Location.FAR and any other potential locations
            driveAwayPose = (alliance == Alliance.RED) ? DRIVE_AWAY_FROM_FAR_WALL_RED : DRIVE_AWAY_FROM_FAR_WALL_BLUE;
            parkPose = (alliance == Alliance.RED) ? DRIVE_AWAY_FROM_FAR_lINE_RED : DRIVE_AWAY_FROM_FAR_lINE_BLUE;
        }

        // 2. Add the initial drive action
        String driveDescription = "DRIVE away from " + alliance + " goal on " + location + " side";
        actionSequence.add(AutoAction.createDriveAction(driveDescription, driveAwayPose));

        // 3. Add shooting sequence based on the detected motif
        addShootingSequence();

        // 4. Add the final parking action
        actionSequence.add(AutoAction.createDriveAction("Strafe to park", parkPose));
    }

    /**
     * Helper method to add the appropriate shooting actions based on the detected motif.
     */
    private void addShootingSequence() {
        switch (detectedMotif) {
            case GPP21: // green purple purple
                addShootActions(LaunchIndexer.FeederSide.RIGHT, LaunchIndexer.FeederSide.LEFT, LaunchIndexer.FeederSide.LEFT);
                break;
            case PGP22: // Purple Green Purple
                addShootActions(LaunchIndexer.FeederSide.LEFT, LaunchIndexer.FeederSide.RIGHT, LaunchIndexer.FeederSide.LEFT);
                break;
            case PPG23: // Purple Purple Green
            case UNKNOWN:
            default:
                addShootActions(LaunchIndexer.FeederSide.LEFT, LaunchIndexer.FeederSide.LEFT, LaunchIndexer.FeederSide.RIGHT);
                break;
        }
    }

    /**
     * Variadic helper to add multiple shoot actions to the sequence.
     * @param sides The sequence of feeder sides to shoot from.
     */
    private void addShootActions(LaunchIndexer.FeederSide... sides) {
        for (LaunchIndexer.FeederSide side : sides) {
            actionSequence.add(AutoAction.createShootAction("Shoot " + side.name(), side));
        }
    }

}

