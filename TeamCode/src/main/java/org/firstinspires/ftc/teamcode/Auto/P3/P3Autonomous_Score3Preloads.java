package org.firstinspires.ftc.teamcode.Auto.P3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3RobotConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot;
//import org.firstinspires.ftc.teamcode.utilities.P3Robot.SharedState;

@Autonomous(name="P3 AUTO: Score Preloads", group="P3Bot",preselectTeleOp = "P3: Teleop (Team Version)")
public class P3Autonomous_Score3Preloads extends OpMode {

    // --- Subsystems ---
    private P3_Robot robot;

    // --- OpMode State and Configuration ---
    private P3RobotConstants.Alliance alliance = P3RobotConstants.Alliance.RED;
    private P3RobotConstants.Location location = P3RobotConstants.Location.CLOSE;


    // --- Master State Machine ---
    private enum AutonomousState {PRE_START, RUNNING_PATH, COMPLETE}

    private AutonomousState autonomousState = AutonomousState.PRE_START;

    // --- Path-Specific State Machines ---
    private enum RedCloseState {
        START, MOVE_AWAY_FROM_GOAL, START_SHOOTING,
        SHOOTING, DRIVE_TO_SPIKEMARK1, PARK
    }

    private RedCloseState redCloseState = RedCloseState.START;

    private enum RedFarState {START, RED_FAR_DRIVE_TO_SCORE, START_SHOOTING, SHOOTING, PARK}

    private RedFarState redFarState = RedFarState.START;

    private enum BlueCloseState {
        START, MOVE_AWAY_FROM_GOAL, START_SHOOTING,
        SHOOTING, DRIVE_TO_SPIKEMARK1, PARK
    }

    private BlueCloseState blueCloseState = BlueCloseState.START;

    private enum BlueFarState {START, PARK}

    private BlueFarState blueFarState = BlueFarState.START;
    private ElapsedTime driveTimer = new ElapsedTime();
    final double DRIVE_TIME = 5.0;
    private int shotsFired = 0;
    private double autoTargetVelocity = 0;

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
        if (gamepad1.x) {
            alliance = P3RobotConstants.Alliance.BLUE;
        }
        if (gamepad1.b) {
            alliance = P3RobotConstants.Alliance.RED;
        }
        if (gamepad1.y) {
            location = P3RobotConstants.Location.CLOSE;
        }
        if (gamepad1.a) {
            location = P3RobotConstants.Location.FAR;
        }


        // --- Telemetry Feedback ---
        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addLine("Ready to Start!");
//        telemetry.addData("robot location X: ", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
//        telemetry.addData("robot location: Y ", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
//        telemetry.addData("robot location: HEADING", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        // Set the robot's starting position based on the final selections
        if (location == P3RobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == P3RobotConstants.Alliance.RED) ?
                    P3RobotConstants.Waypoints.START_RED_CLOSE
                    : P3RobotConstants.Waypoints.START_BLUE_CLOSE);

        } else { // FAR
            robot.drive.pinpoint.setPosition((alliance == P3RobotConstants.Alliance.RED) ? P3RobotConstants.Waypoints.START_RED_FAR : P3RobotConstants.Waypoints.START_BLUE_FAR);
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
                if (alliance == P3RobotConstants.Alliance.RED) {
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
        //robot.addTelemetry();
//        telemetry.addData("imu heading: ", robot.drive.heading);
//        telemetry.addData("Path State Complete", blueCloseState);
//        telemetry.addData("robot location X: ", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
//        telemetry.addData("robot location: Y ", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
//        telemetry.addData("robot location: HEADING", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));
//        telemetry.addData("shooter velocity: ", robot.launcher.getShooterMotorVelocity());
//        telemetry.update();
    }

    @Override
    public void stop() {
        // Save the selected alliance for TeleOp to use.
        //SharedState.alliance = this.alliance;
        if (robot != null) {
            robot.stopAll();
        }
    }

    private void runRedClosePath() {
        telemetry.addData("Current Path", "Red Close");
        switch (redCloseState) {
            case START:
                redCloseState = RedCloseState.MOVE_AWAY_FROM_GOAL;
                break;
            case MOVE_AWAY_FROM_GOAL:
                robot.intake.setIntakeMotorPower(1.0);
                robot.launcher.setShooterMotorVelocity(1000);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_CLOSE_DRIVE_AWAY, 0.35, 0.25)) {
                    shotsFired = 0;
                    redCloseState = RedCloseState.START_SHOOTING;
                }
                break;
            case START_SHOOTING:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_CLOSE_DRIVE_AWAY, 0.35, 0.25);
                autoTargetVelocity = 1300;//robot.getTargetVelocityForDistance(robot.vision.getDistanceToTagMeters()* 39.3701);
                robot.launcher.setShooterMotorVelocity(autoTargetVelocity);
                driveTimer.reset();
                redCloseState = RedCloseState.SHOOTING;
                break;
            case SHOOTING:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_CLOSE_DRIVE_AWAY, 0.35, 0.25);
                telemetry.addData("Path State", redCloseState);
                //if (!robot.isLaunchSequenceBusy()) {
                if (shotsFired < 3) {
                    // Start the sequence
                    telemetry.addData("Launch Sequence", "launch!");
                    if (robot.launchSequence(true, autoTargetVelocity)) {
                        shotsFired++;
                    }
                    ;

                } else {
                    // All 3 shots are done, move to the next auto state (e.g., PARK).
                    redCloseState = RedCloseState.PARK;
                }
                //}
                break;
            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_CLOSE_PARK, 0.5, 0.25)) {
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
                redFarState = RedFarState.RED_FAR_DRIVE_TO_SCORE;
                break;
            case RED_FAR_DRIVE_TO_SCORE:
                robot.intake.setIntakeMotorPower(1.0);
                robot.launcher.setShooterMotorVelocity(1000);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_FAR_DRIVE_AWAY, 0.35, 0.25)) {
                    shotsFired = 0;
                    redFarState = RedFarState.START_SHOOTING;
                }
                break;
            case START_SHOOTING:
                telemetry.addData("I'm here with olivia, chilling not getting to this code", "bruh number 1");

                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_FAR_DRIVE_AWAY, 0.35, 0.25);
                autoTargetVelocity = 1300;//robot.getTargetVelocityForDistance(robot.vision.getDistanceToTagMeters()* 39.3701);
                robot.launcher.setShooterMotorVelocity(autoTargetVelocity);
                driveTimer.reset();
                redFarState = RedFarState.SHOOTING;
                break;
            case SHOOTING:
                telemetry.addData("I'm here with olivia, chilling not getting to this code", "bruh");
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_FAR_DRIVE_AWAY, 0.35, 0.25);
                telemetry.addData("Path State", redFarState);
                if (shotsFired < 3) {
                    // Start the sequence
                    telemetry.addData("Launch Sequence", "launch!");
                    if (robot.launchSequence(true, autoTargetVelocity)) {
                        shotsFired++;
                    } else {
                        // All 3 shots are done, move to the next auto state (e.g., PARK).
                        redFarState = RedFarState.PARK;
                    }
                }
                break;
            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.RED_FAR_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", redFarState);
    }

        private void runBlueClosePath () {
            telemetry.addData("Current Path", "Blue Close");

            switch (blueCloseState) {
                case START:
                    blueCloseState = BlueCloseState.MOVE_AWAY_FROM_GOAL;
                    break;
                case MOVE_AWAY_FROM_GOAL:
                    robot.intake.setIntakeMotorPower(1.0);
                    robot.launcher.setShooterMotorVelocity(1000);
                    if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY, 0.35, 1)) {
                        shotsFired = 0;
                        blueCloseState = BlueCloseState.START_SHOOTING;
                    }
                    break;
                case START_SHOOTING:
                    robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY, 0.35, 0);
                    autoTargetVelocity = 1350;//robot.getTargetVelocityForDistance(robot.vision.getDistanceToTagMeters()* 39.3701);
                    robot.launcher.setShooterMotorVelocity(autoTargetVelocity);
                    driveTimer.reset();
                    blueCloseState = BlueCloseState.SHOOTING;
                    break;
                case SHOOTING:
                    robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY, 0.35, 0);
                    telemetry.addData("Path State", blueCloseState);
                    //if (!robot.isLaunchSequenceBusy()) {
                    if (shotsFired < 3) {
                        // Start the sequence
                        telemetry.addData("Launch Sequence", "launch!");
                        if (robot.launchSequence(true, autoTargetVelocity)) {
                            shotsFired++;
                        }
                        ;

                    } else {
                        // All 3 shots are done, move to the next auto state (e.g., PARK).
                        blueCloseState = BlueCloseState.PARK;
                    }
                    //}
                    break;
                case PARK:
                    if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.BLUE_CLOSE_PARK, 0.5, 0.25)) {
                        autonomousState = AutonomousState.COMPLETE;
                    }
                    break;
            }
            telemetry.addData("Path State", blueCloseState);
        }


        private void runBlueFarPath () {
            telemetry.addData("Current Path", "Blue Far");
            switch (blueFarState) {
                case START:
                    blueFarState = BlueFarState.PARK;
                    break;
                case PARK:
//                robot.drive.turnTo(45,0.5,.25);
                    if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), P3RobotConstants.Waypoints.BLUE_FAR_PARK, 0.5, 0.25)) {
                        autonomousState = AutonomousState.COMPLETE;
                    }
                    autonomousState = AutonomousState.COMPLETE;
                    break;
            }
        }

    }
