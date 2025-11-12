package org.firstinspires.ftc.teamcode.Auto.GearGirls.earlyIdeas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.AutoAction;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="GEAR GIRLS AUTO: Shoot 3 Preloads", group="GGBot")
@Disabled
public class GearGirlsAutonomous_OVERCOMPLICATED extends OpMode {

    // This is your local 'alliance' variable used for selection in init_loop

    // --- Subsystems ---
    private GGRobot robot;
    private VisionUtil vision;

    // Use the public enums from the constants file now
    private GGRobotConstants.Alliance alliance = GGRobotConstants.Alliance.RED;
    private GGRobotConstants.Location location = GGRobotConstants.Location.FAR;
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;

    // The OpMode only needs to own the sequence (the script) and know which line it's on.
    private List<AutoAction> actionSequence = new ArrayList<>();
    private int currentActionIndex = 0;

    // --- State Machine and Action Playlist ---
    private enum AutonomousState {
        EXECUTE_SEQUENCE,
        COMPLETE
    }
    private AutonomousState autonomousState = AutonomousState.EXECUTE_SEQUENCE;

    AutoAction currentAction = null;// = actionSequence.get(currentActionIndex);

    Pose2D startPos;


    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot(hardwareMap, telemetry);
        vision = new VisionUtil(hardwareMap, telemetry);
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
        if (gamepad1.xWasPressed() && alliance != GGRobotConstants.Alliance.BLUE) {
            alliance = GGRobotConstants.Alliance.BLUE;
            selectionChanged = true; // The selection changed, so we must rebuild.
            startLocationChanged = true;
        }
        if (gamepad1.bWasPressed() && alliance != GGRobotConstants.Alliance.RED) {
            alliance = GGRobotConstants.Alliance.RED;
            selectionChanged = true; // The selection changed, so we must rebuild.
            startLocationChanged = true;
        }

        // --- Location Selection ---
        if (gamepad1.yWasPressed() && location != GGRobotConstants.Location.CLOSE) {
            location = GGRobotConstants.Location.CLOSE;
            selectionChanged = true; // The selection changed, so we must rebuild.
            startLocationChanged = true;
        }
        if (gamepad1.aWasPressed() && location != GGRobotConstants.Location.FAR) {
            location = GGRobotConstants.Location.FAR;
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
                    if (location == GGRobotConstants.Location.CLOSE) {
                        if (robot.launchSequence(true, currentAction.feederSide, GGRobotConstants.LauncherDistance.AUTO)) {
                            currentActionIndex++;
                        }
                    } else if (location == GGRobotConstants.Location.FAR){
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

        addTelemetry();
    }

    private void addTelemetry() {
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
        SharedState.alliance = this.alliance;
        if (vision != null) vision.stop();
        if (robot != null) robot.stopAll();
    }

    //================================================================================
    // AUTONOMOUS PATH PLANNING
    //================================================================================

    private void buildActionSequence() {
        // The OpMode's ONLY job is to ask the robot to build a plan based on the
        // current selections. All the complex if/else and switch logic is now
        // inside the GGRobot class.
        this.actionSequence = robot.buildAutonomousSequence(alliance, location, detectedMotif);

        // This part remains in the OpMode because it's about setting the *initial state*
        // of the odometry for THIS specific autonomous run.
        if (location == GGRobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_CLOSE : GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else { // Location.FAR
            robot.drive.pinpoint.setPosition((alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_FAR : GGRobotConstants.Waypoints.START_BLUE_FAR);
        }
    }



}

