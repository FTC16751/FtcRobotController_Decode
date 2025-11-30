package org.firstinspires.ftc.teamcode.Auto.GearGirls.earlyIdeas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.AutoAction;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="GG Pinpoint Action Auto", group="GGBot")
@Disabled
public class GGAutonomous002 extends OpMode {

    // --- Subsystems ---
    private GGRobot robot;
    private VisionUtil vision;

    // --- State Machine and Action Playlist ---
    // With this new logic, the state machine becomes incredibly simple!
    private enum AutonomousState {
        EXECUTE_SEQUENCE, // The one and only active state
        WAITING_FOR_START, DRIVE_TO_TARGET_1, DRIVE_TO_TARGET_2, DRIVE_TO_TARGET_3, DRIVE_TO_TARGET_4, DRIVE_TO_TARGET_5, AT_TARGET, COMPLETE
    }
    private AutonomousState autonomousState = AutonomousState.EXECUTE_SEQUENCE;

    private List<AutoAction> actionSequence = new ArrayList<>();
    private int currentActionIndex = 0;

    // --- Game-Specific Variables ---
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH,12,0, AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH,0,12, AngleUnit.DEGREES,0);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90);

    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot(hardwareMap, telemetry);
        vision = new VisionUtil(hardwareMap, telemetry);
        vision.setPipeline(0);

        telemetry.addData(">", "Robot Initialized. Detecting Motif...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        vision.update();
        detectedMotif = vision.getMotifPattern();
        buildActionSequence(); // Build the plan based on vision

        telemetry.addData("Detected Motif", detectedMotif);
        telemetry.addData("Planned Actions", actionSequence.size());
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset the sequence and set the starting state
        currentActionIndex = 0;
        autonomousState = AutonomousState.EXECUTE_SEQUENCE;
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
        AutoAction currentAction = actionSequence.get(currentActionIndex);
        switch (autonomousState) {
            case EXECUTE_SEQUENCE:
                if (currentAction.type == AutoAction.ActionType.DRIVE_TO_POINT) {
                    // This is a BLOCKING call. The loop will "freeze" here until the robot arrives.
                    if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), currentAction.targetPose, 0.5, 2)){
                    //if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,90), 0.5, 5)){
                        currentActionIndex++;
                        //the first step in the autonomous
                        autonomousState = AutonomousState.DRIVE_TO_TARGET_1;

                    };
                    // Because it's blocking, as soon as it finishes, the action is complete.
                    //isActionComplete = true;

                }
                break;
            case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the xxx.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), TARGET_1, 0.5, 2)){
                    currentActionIndex++;
                    autonomousState = AutonomousState.DRIVE_TO_TARGET_2;
                };

//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), TARGET_1, 0.5, 2)){
//                    telemetry.addLine("at position #1!");
//
//                    currentActionIndex++;
//                    autonomousState = AutonomousState.DRIVE_TO_TARGET_2;
//                }
                break;
            case DRIVE_TO_TARGET_2:
                //drive to the second target
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), TARGET_2, 0.5, 2)){
                    telemetry.addLine("at position #2!");
                    telemetry.addData("currentActionIndex", currentActionIndex);
                    telemetry.addData("ActionSequence Size ", actionSequence.size());
                    currentActionIndex++;
                    autonomousState = AutonomousState.DRIVE_TO_TARGET_3;
                }
                break;
            case DRIVE_TO_TARGET_3:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), TARGET_3, 0.5, 2)){
                    telemetry.addLine("at position #3");
                    telemetry.addData("currentActionIndex", currentActionIndex);
                    telemetry.addData("ActionSequence Size ", actionSequence.size());
                    currentActionIndex++;
                    autonomousState = AutonomousState.DRIVE_TO_TARGET_4;
                }
                break;
            case DRIVE_TO_TARGET_4:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(),TARGET_4,0.5,2)){
                    telemetry.addLine("at position #4");
                    telemetry.addData("currentActionIndex", currentActionIndex);
                    telemetry.addData("ActionSequence Size ", actionSequence.size());
                    currentActionIndex++;
                    autonomousState = AutonomousState.DRIVE_TO_TARGET_5;
                }
                break;
            case DRIVE_TO_TARGET_5:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(),TARGET_5,0.5,0)){
                    telemetry.addLine("There!");
                    telemetry.addData("currentActionIndex", currentActionIndex);
                    telemetry.addData("ActionSequence Size ", actionSequence.size());
                    currentActionIndex++;
                    autonomousState = AutonomousState.AT_TARGET;
                }
                break;

            case COMPLETE:
                robot.stopAll();
                requestOpModeStop(); // End the OpMode
                break;
        }

        telemetry.addData("Auto State", autonomousState);
        telemetry.addData("currentActionIndex ", currentActionIndex);
        telemetry.addData("ActionSequence Size ", actionSequence.size());
        telemetry.addData("current type: ",currentAction.type);
        telemetry.addData("current type: ",currentAction.description);
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

        // We use Pose2D for targets: new Pose2D(X_INCHES, Y_INCHES, Math.toRadians(HEADING_DEGREES))
        switch (detectedMotif) {
            case GPP21: // Left

                actionSequence.add(new AutoAction("Drive forward 12", new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,90)));
                // actionSequence.add(new AutoAction("Drop Pixel", ...)); // Future action
                //actionSequence.add(new AutoAction("Drive to Back", new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0)));
                //actionSequence.add(new AutoAction("Shoot Left", FeederSide.LEFT));
                //actionSequence.add(new AutoAction("Shoot Right", FeederSide.RIGHT));
                //actionSequence.add(new AutoAction("Park", new Pose2D(DistanceUnit.INCH,0, 12, AngleUnit.DEGREES,90)));
                break;

            case PGP22: // Center
                actionSequence.add(new AutoAction("Drive to Center Spike", new Pose2D(DistanceUnit.INCH,32, 0, AngleUnit.DEGREES,0)));
                // ...
                break;

            case PPG23: // Right
            case UNKNOWN: // Default path
            default:
                actionSequence.add(new AutoAction("seq 0", new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,-90)));
                actionSequence.add(new AutoAction("seq 1", new Pose2D(DistanceUnit.INCH,0, -0, AngleUnit.DEGREES,-180)));
                actionSequence.add(new AutoAction("seq 2", new Pose2D(DistanceUnit.INCH,0, -0, AngleUnit.DEGREES,-0)));
                actionSequence.add(new AutoAction("seq 3", new Pose2D(DistanceUnit.INCH,0, -0, AngleUnit.DEGREES,-90)));
                actionSequence.add(new AutoAction("seq 4", new Pose2D(DistanceUnit.INCH,0, -0, AngleUnit.DEGREES,-180)));
                actionSequence.add(new AutoAction("seq 5", new Pose2D(DistanceUnit.INCH,0, -0, AngleUnit.DEGREES,-0)));

                // ...
                break;
        }
    }
}

