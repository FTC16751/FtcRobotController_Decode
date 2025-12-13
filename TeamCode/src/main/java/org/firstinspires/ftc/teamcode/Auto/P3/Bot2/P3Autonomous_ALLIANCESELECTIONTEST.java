package org.firstinspires.ftc.teamcode.Auto.P3.Bot2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3RobotConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.SharedState;
//import org.firstinspires.ftc.teamcode.utilities.P3Robot.SharedState;

@Autonomous(name="P3 AUTO: bot2 Alliance Selection Test", group="P3Bot2",preselectTeleOp = "P3: Teleop (Team Version)")
@Disabled
public class P3Autonomous_ALLIANCESELECTIONTEST extends OpMode {

    // --- Subsystems ---
    private P3_Robot robot;

    // --- OpMode State and Configuration ---
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    private P3RobotConstants.Location location = P3RobotConstants.Location.CLOSE;


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
        robot = new P3_Robot(hardwareMap, telemetry);
        telemetry.addData(">", "Robot Initialized. Ready for selections.");
    }

    @Override
    public void init_loop() {
        robot.update();
        // --- Driver Selections ---
        if (gamepad1.x) { alliance = CommonConstants.Alliance.BLUE; }
        if (gamepad1.b) { alliance = CommonConstants.Alliance.RED; }
        if (gamepad1.y) { location = P3RobotConstants.Location.CLOSE; }
        if (gamepad1.a) { location = P3RobotConstants.Location.FAR; }


        // --- Telemetry Feedback ---
        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addLine("Ready to Start!");
        telemetry.addData("robot location X: ", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
        telemetry.addData("robot location: Y ", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
        telemetry.addData("robot location: HEADING", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        // Set the robot's starting position based on the final selections
        if (location == P3RobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ? P3RobotConstants.Waypoints.START_RED_CLOSE : P3RobotConstants.Waypoints.START_BLUE_CLOSE);
        } else { // FAR
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ? P3RobotConstants.Waypoints.START_RED_FAR : P3RobotConstants.Waypoints.START_BLUE_FAR);
        }

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
                if (alliance == CommonConstants.Alliance.RED) {
                    if (location == P3RobotConstants.Location.CLOSE) {
                        runRedClosePath();
                    } else { // FAR
                        runRedFarPath();
                    }
                } else { // BLUE
                    if (location == P3RobotConstants.Location.CLOSE) {
                        runBlueClosePath();
                    } else { // FAR
                        runBlueFarPath();
                    }
                }
                break;
            case COMPLETE:
                robot.stopAll();
                requestOpModeStop();
                break;
        }
        robot.addTelemetry();
        telemetry.addData("imu heading: ", robot.drive.heading);
        telemetry.addData("Path State Complete", blueCloseState);
        telemetry.addData("robot location X: ", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
        telemetry.addData("robot location: Y ", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
        telemetry.addData("robot location: HEADING", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));
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
                autonomousState = AutonomousState.COMPLETE;


                break;
        }
        telemetry.addData("Path State", redCloseState);
    }

    private void runRedFarPath() {
        telemetry.addData("Current Path", "Red Far");
        switch (redFarState) {
            case START:
                redFarState = RedFarState.PARK;
                break;
            case PARK:
                autonomousState = AutonomousState.COMPLETE;

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
                autonomousState = AutonomousState.COMPLETE;
                break;
        }
        telemetry.addData("Path State", blueCloseState);
    }


    private void runBlueFarPath() {
        telemetry.addData("Current Path", "Blue Far");
        switch (blueFarState) {
            case START:
                blueFarState = BlueFarState.PARK;
                break;
            case PARK:
                autonomousState = AutonomousState.COMPLETE;
                break;
        }
    }

}
