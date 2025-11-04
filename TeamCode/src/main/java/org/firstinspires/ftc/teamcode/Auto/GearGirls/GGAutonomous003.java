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
    private enum AutonStartPosition {
        RED_CLOSE,
        BLUE_CLOSE,
        RED_FAR,
        BLUE_FAR
    };
    private AutonStartPosition StartPosition = AutonStartPosition.RED_CLOSE;


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

    Pose2D startPos;
    // --- Define Autonomous Waypoints ---
    /* START Poses */
    private static final Pose2D START_RED_CLOSE_TO_GOAL = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,54);
    private static final Pose2D START_RED_FAR_FROM_GOAL = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
    private static final Pose2D START_BLUE_CLOSE_TO_GOAL = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,-54);
    private static final Pose2D START_BLUE_FAR_FROM_GOAL = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);

    /* Red Alliance Poses */
    private static final Pose2D RED_DRIVE_AWAY_FROM_GOAL = new Pose2D(DistanceUnit.INCH,-13.7, 17, AngleUnit.DEGREES,-45);
    //*OLD KEPT FOR HISTORICAL    private static final Pose2D DRIVE_AWAY_FROM_GOAL_RED = new Pose2D(DistanceUnit.INCH,5.7963, 30.3506, AngleUnit.DEGREES,-95);
    private static final Pose2D RED_MOVE_OFF_SHOTLINE_CLOSE_TO_GOAL = new Pose2D(DistanceUnit.INCH,10, 15.6, AngleUnit.DEGREES,-45);
    private static final Pose2D RED_DRIVE_AWAY_FROM_WALL_FAR_FROM_GOAL = new Pose2D(DistanceUnit.INCH,7.2, 0, AngleUnit.DEGREES,-21);
    private static final Pose2D RED_MOVE_OFF_SHOTLINE_FAR_FROM_GOAL = new Pose2D(DistanceUnit.INCH,30, 0, AngleUnit.DEGREES,0);
    private static final Pose2D RED_DRIVE_TO_SPIKE_MARK_ONE = new Pose2D(DistanceUnit.INCH,-34, 28, AngleUnit.DEGREES,-90);

    /* Blue Alliance Poses */
    private static final Pose2D BLUE_DRIVE_AWAY_FROM_GOAL = new Pose2D(DistanceUnit.INCH,5.7963, -30.3506, AngleUnit.DEGREES,95);
    private static final Pose2D BLUE_MOVE_OFF_SHOTLINE_CLOSE_TO_GOAL = new Pose2D(DistanceUnit.INCH,22, -10, AngleUnit.DEGREES,95);

    private static final Pose2D BLUE_DRIVE_AWAY_FROM_WALL_FAR_FROM_GOAL = new Pose2D(DistanceUnit.INCH,7.2, -5, AngleUnit.DEGREES,21);
    private static final Pose2D BLUE_MOVE_OFF_SHOTLIE_FAR_FROM_GOAL = new Pose2D(DistanceUnit.INCH,30, 0, AngleUnit.DEGREES,0);





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
        robot.drive.pinpoint.update();
        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        boolean selectionChanged = false; // A flag to track if we need to rebuild the plan
        boolean startLocationChanged = false;

        // --- Alliance Selection ---
        if (gamepad1.xWasPressed() && alliance != Alliance.BLUE) {
            alliance = Alliance.BLUE;
            selectionChanged = true; // The selection changed, so we must rebuild.
            startLocationChanged = true;
        }
        if (gamepad1.bWasPressed() && alliance != Alliance.RED) {
            alliance = Alliance.RED;
            selectionChanged = true; // The selection changed, so we must rebuild.
            startLocationChanged = true;
        }

        // --- Location Selection ---
        if (gamepad1.yWasPressed() && location != Location.CLOSE) {
            location = Location.CLOSE;
            selectionChanged = true; // The selection changed, so we must rebuild.
            startLocationChanged = true;
        }
        if (gamepad1.aWasPressed() && location != Location.FAR) {
            location = Location.FAR;
            selectionChanged = true; // The selection changed, so we must rebuild.
            startLocationChanged = true;
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
        if (startLocationChanged) {
            robot.drive.pinpoint.setPosition(startPos);
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
        telemetry.addLine("--- Robot Position  ---");
        telemetry.addData("X coordinate (IN)", robot.drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", robot.drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", robot.drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();

    }

    @Override
    public void start() {
        robot.drive.pinpoint.setPosition(startPos);
        robot.drive.pinpoint.update();
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
        telemetry.addData("X coordinate (IN)", robot.drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", robot.drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", robot.drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
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
                } else if (currentAction.type == AutoAction.ActionType.DRIVE_AND_INTAKE) {
                    if(robot.driveAndIntakeSequence(true, currentAction.intakeDriveDistance)){
                        currentActionIndex++;
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
            driveAwayPose = (alliance == Alliance.RED) ? RED_DRIVE_AWAY_FROM_GOAL : BLUE_DRIVE_AWAY_FROM_GOAL;
            parkPose = (alliance == Alliance.RED) ? RED_MOVE_OFF_SHOTLINE_CLOSE_TO_GOAL : BLUE_MOVE_OFF_SHOTLINE_CLOSE_TO_GOAL;
            startPos = (alliance == Alliance.RED) ? START_RED_CLOSE_TO_GOAL : START_BLUE_CLOSE_TO_GOAL;
        } else { // Handles Location.FAR and any other potential locations
            driveAwayPose = (alliance == Alliance.RED) ? RED_DRIVE_AWAY_FROM_WALL_FAR_FROM_GOAL : BLUE_DRIVE_AWAY_FROM_WALL_FAR_FROM_GOAL;
            parkPose = (alliance == Alliance.RED) ? RED_MOVE_OFF_SHOTLINE_FAR_FROM_GOAL : BLUE_MOVE_OFF_SHOTLIE_FAR_FROM_GOAL;
            startPos = (alliance == Alliance.RED) ? START_RED_FAR_FROM_GOAL : START_BLUE_FAR_FROM_GOAL;
        }

        // 2. Add the initial drive action
        String driveDescription = "DRIVE away from " + alliance + " goal on " + location + " side";
        actionSequence.add(AutoAction.createDriveAction(driveDescription, driveAwayPose));

        // 3. Add shooting sequence based on the detected motif
        addShootingSequence();

        // 4. Add the final parking action
        //actionSequence.add(AutoAction.createDriveAction("Strafe to park", parkPose));
        actionSequence.add(AutoAction.createDriveAction("Drive to Spike Mark 1",RED_DRIVE_TO_SPIKE_MARK_ONE));
        actionSequence.add(AutoAction.createDriveAndIntakeAction("Intake from Floor", 12.0));
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

