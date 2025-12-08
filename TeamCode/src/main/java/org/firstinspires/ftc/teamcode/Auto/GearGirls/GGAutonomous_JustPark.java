package org.firstinspires.ftc.teamcode.Auto.GearGirls;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;


@Autonomous(name="GG AUTO: Just Parka", group="GGBot")
public class GGAutonomous_JustPark extends OpMode {

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
    private enum RedCloseState { START, PARK }
    private RedCloseState redCloseState = RedCloseState.START;

    private enum RedFarState { START,  PARK }
    private RedFarState redFarState = RedFarState.START;

    private enum BlueCloseState { START, PARK }
    private BlueCloseState blueCloseState = BlueCloseState.START;
    private enum BlueFarState {START, PARK }
    private BlueFarState blueFarState = BlueFarState.START;


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


        // --- Telemetry Feedback ---
        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addLine("Ready to Start!");
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


    private void runRedClosePath() {
        telemetry.addData("Current Path", "Red Close");

        switch (redCloseState) {
            case START:
                redCloseState = RedCloseState.PARK;
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

    private void runRedFarPath() {
        telemetry.addData("Current Path", "Red Far");
        switch (redFarState) {
            case START:
                blueCloseState = BlueCloseState.PARK;
                break;
            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_PARK, 0.5, 0.25)) {
                autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
    }

    private void runBlueClosePath() {
        telemetry.addData("Current Path", "Blue Close");

        switch (blueCloseState) {
            case START:
                blueCloseState = BlueCloseState.PARK;
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


    private void runBlueFarPath() {
        telemetry.addData("Current Path", "Blue Far");
        switch (blueFarState) {
            case START:
                blueCloseState = BlueCloseState.PARK;
                break;
            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
    }

}
