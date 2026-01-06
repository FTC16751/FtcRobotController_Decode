package org.firstinspires.ftc.teamcode.Auto.P3.Bot2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3RobotConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name="P3 AUTO Bot2: Queue Version", group="P3Bot2", preselectTeleOp = "P3: Teleop (Team Version)")
public class P3Autonomous_Queue extends OpMode {

    // --- Subsystems ---
    private P3_Robot robot;

    // --- OpMode State and Configuration ---
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    private P3RobotConstants.Location location = P3RobotConstants.Location.CLOSE;

    // --- Master State Machine ---
    private enum AutonomousState { PRE_START, RUNNING_PATH, COMPLETE }
    private AutonomousState autonomousState = AutonomousState.PRE_START;

    // === 1. A SINGLE, UNIFIED STATE MACHINE ===
    // These enums describe the ACTION the robot is taking, not the entire path.
    private enum PathState {
        START,
        DRIVE_TO_SHOOT_PRELOAD,
        AIM_AT_TARGET,
        SHOOT_PRELOAD,
        DRIVE_TO_SPIKE_1,
        COLLECT_FROM_SPIKE_1,
        DRIVE_TO_SPIKE_2,
        COLLECT_FROM_SPIKE_2,
        DRIVE_TO_SPIKE_3,
        COLLECT_FROM_SPIKE_3,
        DRIVE_TO_SHOOT_CYCLE_1,
        SHOOT_CYCLE_1,
        DRIVE_TO_SHOOT_CYCLE_2,
        SHOOT_CYCLE_2,
        PARK,
        INIT_WAIT, WAIT_FOR_TIMER, OPEN_GATE, ALIGN_GATE, IDLE // A final state for when a path is done
    }
    private PathState currentState = PathState.START;

    // === 2. A "SCRIPT" for each path ===
    // A Queue is a "First-In, First-Out" list, perfect for a sequence of steps.
    private final Queue<PathState> pathScript = new LinkedList<>();

    // === 3. Waypoint and Velocity Variables ===
    // These will be loaded in start() based on the selected path.
    private Pose2D shootingPosition;
    private Pose2D spike1Align;
    private Pose2D spike1Collect;
    private Pose2D spike2Align;
    private Pose2D spike2Collect;
    private Pose2D spike3Align;
    private Pose2D spike3Collect;
    private Pose2D parkPosition;
    private Pose2D alignOpenGate;
    private Pose2D openGate;
    private double shootingVelocity;
    
    // Path-specific parameters
    private double driveToShootSpeed = 0.5;
    private double driveToShootHoldTime;
    private double shootingDriveSpeed;
    private double shootingHoldTime;
    private double spike1AlignSpeed = 0.5;
    private double spike1CollectSpeed= 0.2;
    private double spike2AlignSpeed = 0.5;
    private double spike2CollectSpeed= 0.2;
    private double spike3AlignSpeed = 0.5;
    private double spike3CollectSpeed= 0.2;
    private double openGateAlignSpeed=0.5;
    private double openGateSpeed= 0.5;
    private boolean useFeederOnSpikeMarkCollection;

    // --- Action-Specific Variables ---
    private int shotsFired = 0;
    private static final double TX_ALIGN_KP = 0.02; // Proportional gain for turning
    private static final double TX_ALIGN_TOLERANCE_DEG = 1.0;
    private final ElapsedTime waitTimer = new ElapsedTime();
    private double waitDuration = 0.0;
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
        // Use else-if for exclusive choices
        if (gamepad1.x) { alliance = CommonConstants.Alliance.BLUE; }
        else if (gamepad1.b) { alliance = CommonConstants.Alliance.RED; }

        if (gamepad1.y) { location = P3RobotConstants.Location.CLOSE; }
        else if (gamepad1.a) { location = P3RobotConstants.Location.FAR; }

        if (gamepad1.dpadUpWasPressed()) { waitDuration += 1; }
        else if (gamepad1.dpadDownWasPressed()) { waitDuration -= 1; }


        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addData("Wait Duration (Dpad up +1 sec, Dpad down -1 sec)", waitDuration);
        telemetry.addLine("Ready to Start!");
        telemetry.addLine();
        telemetry.addData("robot location X: ", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
        telemetry.addData("robot location: Y ", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
        telemetry.addData("robot location: HEADING", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        // --- 4. BUILD THE SCRIPT AND LOAD THE WAYPOINTS ---
        // This block now defines the entire "plan" for the chosen path.

        if (alliance == CommonConstants.Alliance.RED) {
            if (location == P3RobotConstants.Location.CLOSE) {
                // Set the waypoints for this path
                shootingPosition = P3RobotConstants.Bot2_Waypoints.RED_CLOSE_SHOOTING_POSITION;
                spike1Align = P3RobotConstants.Bot2_Waypoints.RED_SPIKEMARK1_ALIGN;
                spike1Collect = P3RobotConstants.Bot2_Waypoints.RED_SPIKEMARK1_COLLECT;
                spike2Align = P3RobotConstants.Bot2_Waypoints.RED_CLOSE_SPIKEMARK2_ALIGN;
                spike2Collect = P3RobotConstants.Bot2_Waypoints.RED_CLOSE_SPIKEMARK2_COLLECT;
                parkPosition = P3RobotConstants.Bot2_Waypoints.RED_CLOSE_PARK;
                shootingVelocity = 1100;
                
                // Set path-specific parameters
                driveToShootSpeed = 0.5;
                driveToShootHoldTime = 0.25;
                shootingDriveSpeed = 0.35;
                shootingHoldTime = 0.25;
                spike1AlignSpeed = 0.5;
                spike1CollectSpeed = 0.2;
                useFeederOnSpikeMarkCollection = true;

                // Build the script for Red Close
                if (waitDuration > 0){
                    pathScript.add(PathState.INIT_WAIT);
                    pathScript.add(PathState.WAIT_FOR_TIMER);
                }
                pathScript.add(PathState.DRIVE_TO_SHOOT_PRELOAD);
                pathScript.add(PathState.SHOOT_PRELOAD);
                pathScript.add(PathState.DRIVE_TO_SPIKE_1);
                pathScript.add(PathState.COLLECT_FROM_SPIKE_1);
                pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_1);
                pathScript.add(PathState.SHOOT_CYCLE_1);
                pathScript.add(PathState.ALIGN_GATE);
                pathScript.add(PathState.OPEN_GATE);
                pathScript.add(PathState.DRIVE_TO_SPIKE_2);
                pathScript.add(PathState.COLLECT_FROM_SPIKE_2);
                pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_2);
                pathScript.add(PathState.SHOOT_CYCLE_2);
                pathScript.add(PathState.PARK);

            } else { // RED FAR
                shootingPosition = P3RobotConstants.Bot2_Waypoints.RED_FAR_SHOOTING_POSITION;
                spike3Align = P3RobotConstants.Bot2_Waypoints.RED_SPIKEMARK3_ALIGN;
                spike3Collect = P3RobotConstants.Bot2_Waypoints.RED_SPIKEMARK3_COLLECT;
                parkPosition = P3RobotConstants.Bot2_Waypoints.RED_FAR_PARK_POSITION;
                spike2Align = P3RobotConstants.Bot2_Waypoints.RED_FAR_SPIKEMARK2_ALIGN;
                spike2Collect = P3RobotConstants.Bot2_Waypoints.RED_FAR_SPIKEMARK2_COLLECT;
                shootingVelocity = 1400;
                
                // Set path-specific parameters
                driveToShootSpeed = 0.35;
                driveToShootHoldTime = 0.25;
                shootingDriveSpeed = 0.5;
                shootingHoldTime = 0.25;
                spike3AlignSpeed = 0.5;
                spike3CollectSpeed = 0.2;
                useFeederOnSpikeMarkCollection = false;
                
                // Initialize launcher position for Red Far
                robot.launcher.setStopPosition();

                // Build the script for Red Far
                if (waitDuration > 0){
                    pathScript.add(PathState.INIT_WAIT);
                    pathScript.add(PathState.WAIT_FOR_TIMER);
                }
                pathScript.add(PathState.DRIVE_TO_SHOOT_PRELOAD);
                pathScript.add(PathState.SHOOT_PRELOAD);
                pathScript.add(PathState.DRIVE_TO_SPIKE_3);
                pathScript.add(PathState.COLLECT_FROM_SPIKE_3);
                pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_1);
                pathScript.add(PathState.SHOOT_CYCLE_1);
                pathScript.add(PathState.DRIVE_TO_SPIKE_2);
                pathScript.add(PathState.COLLECT_FROM_SPIKE_2);
                pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_2);
                pathScript.add(PathState.SHOOT_CYCLE_2);
                pathScript.add(PathState.PARK);
            }
        } else { // BLUE
            if (location == P3RobotConstants.Location.CLOSE) {
                shootingPosition = P3RobotConstants.Bot2_Waypoints.BLUE_CLOSE_SHOOTING_POSITION;
                spike1Align = P3RobotConstants.Bot2_Waypoints.BLUE_SPIKEMARK1_ALIGN;
                spike1Collect = P3RobotConstants.Bot2_Waypoints.BLUE_SPIKEMARK1_COLLECT;
                spike2Align = P3RobotConstants.Bot2_Waypoints.BLUE_CLOSE_SPIKEMARK2_ALIGN;
                spike2Collect = P3RobotConstants.Bot2_Waypoints.BLUE_CLOSE_SPIKEMARK2_COLLECT;
                alignOpenGate = P3RobotConstants.Bot2_Waypoints.BLUE_ALIGN_GATE;
                openGate = P3RobotConstants.Bot2_Waypoints.BLUE_OPEN_GATE;
                parkPosition = P3RobotConstants.Bot2_Waypoints.BLUE_CLOSE_PARK;
                shootingVelocity = 1050;

                // Set path-specific parameters (Blue Close uses tighter heading control)
                driveToShootSpeed = 0.75;
                driveToShootHoldTime = 0.20;
                shootingDriveSpeed = 0.6;
                shootingHoldTime = 0.0; // Strict heading control during shooting
                spike1AlignSpeed = 0.75;
                spike1CollectSpeed = 0.75;
                useFeederOnSpikeMarkCollection = true;

                // Build script for Blue Close
                if (waitDuration > 0){
                    pathScript.add(PathState.INIT_WAIT);
                    pathScript.add(PathState.WAIT_FOR_TIMER);
                }
                    pathScript.add(PathState.DRIVE_TO_SHOOT_PRELOAD);
                    pathScript.add(PathState.AIM_AT_TARGET);
                    pathScript.add(PathState.SHOOT_PRELOAD);
                    pathScript.add(PathState.DRIVE_TO_SPIKE_1);
                    pathScript.add(PathState.COLLECT_FROM_SPIKE_1);
                    pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_1);
                    pathScript.add(PathState.AIM_AT_TARGET);
                    pathScript.add(PathState.SHOOT_CYCLE_1);
                    pathScript.add(PathState.ALIGN_GATE);
                    pathScript.add(PathState.OPEN_GATE);
                    pathScript.add(PathState.DRIVE_TO_SPIKE_2);
                    pathScript.add(PathState.COLLECT_FROM_SPIKE_2);
                    pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_2);
                    pathScript.add(PathState.AIM_AT_TARGET);
                    pathScript.add(PathState.SHOOT_CYCLE_2);
                    pathScript.add(PathState.PARK);

            } else { // BLUE FAR
                shootingPosition = P3RobotConstants.Bot2_Waypoints.BLUE_FAR_SHOOTING_POSITION;
                spike3Align = P3RobotConstants.Bot2_Waypoints.BLUE_SPIKEMARK3_ALIGN;
                spike3Collect = P3RobotConstants.Bot2_Waypoints.BLUE_SPIKEMARK3_COLLECT;
                spike2Align = P3RobotConstants.Bot2_Waypoints.BLUE_SPIKEMARK2_ALIGN;
                spike2Collect = P3RobotConstants.Bot2_Waypoints.BLUE_SPIKEMARK2_COLLECT;
                parkPosition = P3RobotConstants.Bot2_Waypoints.BLUE_FAR_PARK_POSITION;
                shootingVelocity = 1460;
                
                // Set path-specific parameters
                driveToShootSpeed = 0.5;
                driveToShootHoldTime = 0.0; // Strict heading control
                shootingDriveSpeed = 0.5;
                shootingHoldTime = 0.0;
                spike3AlignSpeed = 0.5;
                spike3CollectSpeed = 0.7; // Blue Far uses faster collection
                spike2AlignSpeed = 0.5;
                spike2CollectSpeed = 0.2;
                useFeederOnSpikeMarkCollection = true; // Blue Far uses feeder motor

                // Build the most complex script for Blue Far
                if (waitDuration > 0){
                    pathScript.add(PathState.INIT_WAIT);
                    pathScript.add(PathState.WAIT_FOR_TIMER);
                }
                pathScript.add(PathState.DRIVE_TO_SHOOT_PRELOAD);
                pathScript.add(PathState.SHOOT_PRELOAD);
                pathScript.add(PathState.DRIVE_TO_SPIKE_3);
                pathScript.add(PathState.COLLECT_FROM_SPIKE_3);
                pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_1);
                pathScript.add(PathState.SHOOT_CYCLE_1);
                pathScript.add(PathState.DRIVE_TO_SPIKE_2);
                pathScript.add(PathState.COLLECT_FROM_SPIKE_2);
                pathScript.add(PathState.DRIVE_TO_SHOOT_CYCLE_2);
                pathScript.add(PathState.SHOOT_CYCLE_2);
                pathScript.add(PathState.PARK);
            }
        }

        // Set the robot's physical starting position on the field
        if (location == P3RobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    P3RobotConstants.Bot2_Waypoints.START_RED_CLOSE : P3RobotConstants.Bot2_Waypoints.START_BLUE_CLOSE);
        } else { // FAR
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    P3RobotConstants.Bot2_Waypoints.RED_FAR_START_POSITION : P3RobotConstants.Bot2_Waypoints.START_BLUE_FAR);
        }
        robot.vision.setTargetingAlliance(alliance);
        telemetry.log().add("Vision pipeline set for: " + alliance);
        // Get the first state from the script
        currentState = getNextState();

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
                // --- 5. RUN THE SINGLE, UNIFIED STATE MACHINE ---
                runPath();
                break;
            case COMPLETE:
                robot.stopAll();
                requestOpModeStop();
                break;
        }

        telemetry.addData("Current Path", alliance + " " + location);
        telemetry.addData("Current State", currentState);
        telemetry.addData("imu heading: ", robot.drive.heading);
        telemetry.addData("robot location X: ", robot.drive.getOdoPosition().getX(DistanceUnit.INCH));
        telemetry.addData("robot location: Y ", robot.drive.getOdoPosition().getY(DistanceUnit.INCH));
        telemetry.addData("robot location: HEADING", robot.drive.getOdoPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("shooter velocity: ", robot.launcher.getShooterMotorVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopAll();
        }
    }

    /**
     * A helper method to get the next state from the script queue.
     * If the queue is empty, it returns the IDLE state.
     */
    private PathState getNextState() {
        if (!pathScript.isEmpty()) {
            return pathScript.poll(); // .poll() retrieves and removes the head of the queue
        }
        return PathState.IDLE;
    }

    /**
     * A reusable helper method for the shooting cycle.
     * @param totalShots The number of artifacts to shoot in this cycle.
     * @return true when all shots have been fired, false otherwise.
     */
    private boolean shootCycle(int totalShots) {
        // Hold position while shooting to counteract inertia
        robot.drive.driveTo(robot.drive.getOdoPosition(), robot.drive.getOdoPosition(), shootingDriveSpeed, shootingHoldTime);
        robot.launcher.setShooterMotorVelocity(shootingVelocity);
        robot.launcher.setShootingPosition();

        if (shotsFired < totalShots) {
            if (robot.launchSequence(true, shootingVelocity)) {
                shotsFired++;
            }
            return false; // Still shooting
        }
        return true; // Finished shooting all shots for this cycle
    }


    //================================================================================
    // THE NEW UNIFIED PATH METHOD
    //================================================================================

    private void runPath() {
        switch (currentState) {
            case START:
                currentState = getNextState();
                break;

            case DRIVE_TO_SHOOT_PRELOAD:
                robot.intake.startIntake();
                robot.launcher.setShooterMotorVelocity(shootingVelocity);
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), shootingPosition, driveToShootSpeed, driveToShootHoldTime)) {
                    shotsFired = 0;
                    currentState = getNextState();
                }
                break;

            case AIM_AT_TARGET:
                double turnPower;
                if (robot.vision.isTargetVisible()) {
                    double txError = robot.vision.getTargetAngleX();

                    // Check if our aim is within the tolerance.
                    if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
                        // Aim is good! Stop turning and move to the next state (SHOOTING).
                        turnPower = 0.0;
                        robot.drive.stopRobot(); // Explicitly stop the robot
                        currentState = getNextState(); // Move on to SHOOT_PRELOAD
                    } else {
                        // Aim is not good. Calculate a correction power.
                        // We reuse the exact same logic from TeleOp.
                        turnPower = txError * TX_ALIGN_KP;
                    }
                } else {
                    // We can't see the tag. For safety, stop turning.
                    // might try to turn based on odometry, but this is safest.
                    turnPower = 0.0;
                    currentState = getNextState();//test if we need this. cover up the april tag or turn down the lights in the room
                }

                // Command the robot to only turn
                robot.drive.moveRobot(0, 0, turnPower);
                telemetry.addData("Aiming", "Running... Error: %.1f", robot.vision.getTargetAngleX());
                break;
            // --- END OF NEW STATE LOGIC ---

            case SHOOT_PRELOAD:
                if (shootCycle(3)) {
                    currentState = getNextState();
                }
                break;

            case DRIVE_TO_SPIKE_1:
                robot.launcher.setStopPosition();
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), spike1Align, spike1AlignSpeed, 0.0)) {
                    currentState = getNextState();
                }
                break;

            case COLLECT_FROM_SPIKE_1:
                if (useFeederOnSpikeMarkCollection) {
                    robot.feeder.setFeederMotorPower(-0.450);
                }
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), spike1Collect, spike1CollectSpeed, 0.25)) {
                    if (useFeederOnSpikeMarkCollection) {
                        robot.feeder.setFeederMotorPower(0.0);
                    }
                    currentState = getNextState();
                }
                break;

            case ALIGN_GATE:
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), alignOpenGate, openGateAlignSpeed, 0.25)) {
                    currentState = getNextState();
                }
                break;

            case OPEN_GATE:
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), openGate, openGateSpeed, 0.25)) {
                    currentState = getNextState();
                }
                break;

            case DRIVE_TO_SPIKE_2:
                robot.launcher.setStopPosition();
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), spike2Align, spike2AlignSpeed, 0.25)) {
                    currentState = getNextState();
                }
                break;

            case COLLECT_FROM_SPIKE_2:
                if (useFeederOnSpikeMarkCollection) {
                    robot.feeder.setFeederMotorPower(-0.450);
                }
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), spike2Collect, spike2CollectSpeed, 0.25)) {
                    if (useFeederOnSpikeMarkCollection) {
                        robot.feeder.setFeederMotorPower(0.0);
                    }
                    currentState = getNextState();
                }
                break;

            case DRIVE_TO_SPIKE_3:
                robot.launcher.setStopPosition();
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), spike3Align, spike3AlignSpeed, 0.25)) {
                    currentState = getNextState();
                }
                break;

            case COLLECT_FROM_SPIKE_3:
                if (useFeederOnSpikeMarkCollection) {
                    robot.feeder.setFeederMotorPower(-0.45);
                }
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), spike3Collect, spike3CollectSpeed, 0.25)) {
                    if (useFeederOnSpikeMarkCollection) {
                        robot.feeder.setFeederMotorPower(0.0);
                    }
                    currentState = getNextState();
                }
                break;

            case DRIVE_TO_SHOOT_CYCLE_1:
            case DRIVE_TO_SHOOT_CYCLE_2: // Can combine logic if the action is the same
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), shootingPosition, driveToShootSpeed, driveToShootHoldTime)) {
                    shotsFired = 0;
                    currentState = getNextState();
                }
                break;

            case SHOOT_CYCLE_1:
            case SHOOT_CYCLE_2: // Can combine logic
                if (shootCycle(3)) {
                    currentState = getNextState();
                }
                break;

            case PARK:
                robot.launcher.setStopPosition();
                if (robot.drive.driveTo(robot.drive.getOdoPosition(), parkPosition, 0.3, 0.25)) {
                    currentState = getNextState();
                }
                break;

            case INIT_WAIT:
                waitTimer.reset();
                currentState = getNextState();
                break;

            case WAIT_FOR_TIMER:
                if (waitTimer.seconds() >= waitDuration) {
                    currentState = getNextState();
                }
                break;

            case IDLE:
                // The script is empty, so the entire path is done.
                autonomousState = AutonomousState.COMPLETE;
                break;
        }
    }
}
