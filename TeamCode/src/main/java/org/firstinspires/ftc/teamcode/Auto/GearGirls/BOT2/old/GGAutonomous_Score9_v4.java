package org.firstinspires.ftc.teamcode.Auto.GearGirls.BOT2.old;

import static org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants.Launcher.AUTO_MIN_VELOCITY;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot2;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeSensorFusion002;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;

/**
 * GearGirls Autonomous - Score 9 with ShotSequenceController
 *
 * This autonomous uses the ShotSequenceController for sensor-based shot sequencing.
 * The controller handles all spinner rotation, color detection, and firing logic.
 *
 * @author GearGirls Team
 * @version 4.0 - Clean ShotSequenceController Implementation
 */
@Autonomous(name="GG AUTO: Score 9 (v4 volley ctrl)", group="GGBot", preselectTeleOp = "GearGirlsBot2_test")
@Disabled
public class GGAutonomous_Score9_v4 extends OpMode {

    // --- Subsystems ---
    private GGRobot2 robot;

    // --- OpMode State and Configuration ---
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    private GGRobotConstants.Location location = GGRobotConstants.Location.CLOSE;
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;
    private int selectedCycles = 2; // Default: 1=Preload only, 2=Preload+Spike1, 3=Preload+Spike1+Spike2
    private boolean blindFireMode = false; // If true, ignore sensors and fire left-right-left

    // --- Master State Machine ---
    private enum AutonomousState { PRE_START, RUNNING_PATH, COMPLETE }
    private AutonomousState autonomousState = AutonomousState.PRE_START;

    // --- Path-Specific State Machines ---
    private enum RedCloseState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        RED_CLOSE_ALIGN_SPIKEMARK1, RED_CLOSE_COLLECT_SPIKEMARK1_BALL1,
        SET_DIVERTER_LEFT, WAIT_FOR_DIVERTER,
        RED_CLOSE_COLLECT_SPIKEMARK1_BALL2, RED_CLOSE_COLLECT_SPIKEMARK1_BALL3,
        DRIVE_TO_SCORE_1, SHOOT_SEQUENCE1,
        RED_CLOSE_ALIGN_SPIKEMARK2, RED_CLOSE_COLLECT_SPIKEMARK2_BALL1,
        SET_DIVERTER_LEFT2, WAIT_FOR_DIVERTER2, RED_CLOSE_COLLECT_SPIKEMARK2_BALL2,
        SET_DIVERTER_RIGHT2, WAIT_FOR_DIVERTER3, RED_CLOSE_COLLECT_SPIKEMARK2_BALL3,
        DRIVE_TO_SCORE3, SHOOT_SEQUENCE3, PARK
    }
    private RedCloseState redCloseState = RedCloseState.DRIVE_TO_SCORE;

    private enum RedFarState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        RED_FAR_ALIGN_SPIKEMARK3, RED_FAR_COLLECT_SPIKEMARK3_BALL1,
        RED_FAR_COLLECT_SPIKEMARK3_BALL2, RED_FAR_COLLECT_SPIKEMARK3_BALL3,
        DRIVE_TO_SCORE2, SET_DIVERTER_RIGHT, WAIT_FOR_DIVERTER, SHOOT_SEQUENCE2,
        RED_FAR_ALIGN_SPIKEMARK2, RED_FAR_COLLECT_SPIKEMARK2_BALL1,
        SET_DIVERTER_LEFT2, WAIT_FOR_DIVERTER2, RED_FAR_COLLECT_SPIKEMARK2_BALL2,
        RED_FAR_COLLECT_SPIKEMARK2_BALL3, DRIVE_TO_SCORE3, SHOOT_SEQUENCE3,
        WAIT_FOR_DIVERTER3, SET_DIVERTER_RIGHT2, PARK
    }
    private RedFarState redFarState = RedFarState.DRIVE_TO_SCORE;

    private enum BlueCloseState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1,
        GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3a,
        GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3, GO_TO_BLUE_CLOSE_SPIKEMARK1_END,
        DRIVE_TO_SCORE_AFTER_SPIKE_MARK1, SHOOT_SEQUENCE2,
        BLUE_CLOSE_ALIGN_SPIKEMARK2, BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL1,
        SET_DIVERTER_RIGHT2, WAIT_FOR_DIVERTER2,
        BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL2, BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL3,
        DRIVE_TO_SCORE3, SHOOT_SEQUENCE3, SET_DIVERTER_LEFT, WAIT_FOR_DIVERTER, PARK
    }
    private BlueCloseState blueCloseState = BlueCloseState.DRIVE_TO_SCORE;

    private enum BlueFarState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        BLUE_FAR_ALIGN_SPIKEMARK3, BLUE_FAR_COLLECT_SPIKEMARK3_BALL1,
        BLUE_FAR_COLLECT_SPIKEMARK3_BALL2, BLUE_FAR_COLLECT_SPIKEMARK3_BALL3,
        DRIVE_TO_SCORE2, SET_DIVERTER_RIGHT, WAIT_FOR_DIVERTER, SHOOT_SEQUENCE2,
        BLUE_FAR_ALIGN_SPIKEMARK2, BLUE_FAR_COLLECT_SPIKEMARK2_BALL1,
        SET_DIVERTER_LEFT2, WAIT_FOR_DIVERTER2, BLUE_FAR_COLLECT_SPIKEMARK2_BALL2,
        BLUE_FAR_COLLECT_SPIKEMARK2_BALL3, DRIVE_TO_SCORE3, SHOOT_SEQUENCE3,
        WAIT_FOR_DIVERTER3, SET_DIVERTER_RIGHT2, PARK
    }
    private BlueFarState blueFarState = BlueFarState.DRIVE_TO_SCORE;

    // --- Action-Specific Variables ---
    private ElapsedTime waitTimer = new ElapsedTime();
    private static final double WAIT_TIME = 0.250;

    // --- Shooting Constants ---
    private static final double MINIMUM_SAFE_LAUNCHER_VELOCITY = AUTO_MIN_VELOCITY;

    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot2(hardwareMap, telemetry);
        robot.vision.setPipeline(0);
        telemetry.addData(">", "Robot Initialized. Ready for selections.");
    }

    @Override
    public void init_loop() {
        robot.vision.setMotifDetectionMode();
        robot.vision.update();
        robot.update();

        // --- Driver Selections ---
        if (gamepad1.x) { alliance = CommonConstants.Alliance.BLUE; }
        if (gamepad1.b) { alliance = CommonConstants.Alliance.RED; }
        if (gamepad1.y) { location = GGRobotConstants.Location.CLOSE; }
        if (gamepad1.a) { location = GGRobotConstants.Location.FAR; }

        // --- Cycle Selection ---
        if (gamepad1.dpadLeftWasPressed() && selectedCycles > 1) { selectedCycles--; }
        if (gamepad1.dpadRightWasPressed() && selectedCycles < 3) { selectedCycles++; }

        // --- Shooting Mode Selection ---
        if (gamepad1.left_bumper) { blindFireMode = false; } // LB = Sensor mode
        if (gamepad1.right_bumper) { blindFireMode = true; }  // RB = Blind fire mode

        // --- Vision Detection ---
        detectedMotif = robot.vision.getMotifPattern();

        // --- Telemetry Feedback ---
        telemetry.addLine("=== SMART SELECTION ===");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addData("Cycles", "%d (DPad Left/Right)", selectedCycles);
        telemetry.addLine("  1 = Preload only");
        telemetry.addLine("  2 = Preload + Spike Mark 1");
        telemetry.addLine("  3 = Preload + Spike Mark 1 + Spike Mark 2");

        telemetry.addData("Shooting Mode", blindFireMode ? "BLIND FIRE" : "Sensor-Based");
        telemetry.addLine(blindFireMode ? "  (L, R, spin→L)" : "  (Match colors)");
        telemetry.addLine("LB = Sensor Mode | RB = Blind Fire");
        telemetry.addLine("\nReady to Start!");
        telemetry.addLine();
        telemetry.addLine("=== PINPOINT ODOMETRY DATA ===");
        telemetry.addLine();
        telemetry.addData("X: ", robot.drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y: ", robot.drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Heading: ", robot.drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addLine("=== COLOR SENSOR DATA ===");
        telemetry.addLine();
        telemetry.addData("Detected Motif", detectedMotif);
        telemetry.addLine();
        telemetry.addData("left Occupied ",robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.LEFT));
        telemetry.addData("right Occupied ",robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.RIGHT));
        telemetry.addData("leftColor ",robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT));
        telemetry.addData("rightColor ",robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT));
        telemetry.update();
    }

    @Override
    public void start() {
        // Set starting position
        if (location == GGRobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_CLOSE : GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_FAR : GGRobotConstants.Waypoints.START_BLUE_FAR);
        }

        // Configure vision for alliance
        robot.vision.setTargetingAlliance(alliance);

        // Start launcher
        robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY,
                GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY);

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
                robot.intake.setIntakeMotorPower(-1);
                if (alliance == CommonConstants.Alliance.RED) {
                    if (location == GGRobotConstants.Location.CLOSE) {
                        runRedClosePath();
                    } else {
                        runRedFarPath();
                    }
                } else {
                    if (location == GGRobotConstants.Location.CLOSE) {
                        runBlueClosePath();
                    } else {
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
        telemetry.update();
    }

    @Override
    public void stop() {
        SharedState.alliance = this.alliance;
        if (robot != null) {
            robot.stopAll();
        }
    }

    //================================================================================
    // RED CLOSE PATH
    //================================================================================

    private void runRedClosePath() {
        telemetry.addData("Current Path", "Red Close");

        switch (redCloseState) {
            case DRIVE_TO_SCORE:
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,
                        GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
                    redCloseState = RedCloseState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 2) {
                        redCloseState = RedCloseState.RED_CLOSE_ALIGN_SPIKEMARK1;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        redCloseState = RedCloseState.PARK;
                    }
                }
                break;

            case RED_CLOSE_ALIGN_SPIKEMARK1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_ALIGN, 0.5, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL3;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL1, 0.5, 0.20)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL2;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL2, 0.5, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL3;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL3, 0.5, 0.0)) {
                    redCloseState = RedCloseState.DRIVE_TO_SCORE_1;
                }
                break;

            case DRIVE_TO_SCORE_1:
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,
                        GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.7, 0.0)) {
                    redCloseState = RedCloseState.SHOOT_SEQUENCE1;
                }
                break;

            case SHOOT_SEQUENCE1:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 3) {
                        redCloseState = RedCloseState.RED_CLOSE_ALIGN_SPIKEMARK2;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        redCloseState = RedCloseState.PARK;
                    }
                }
                break;

            case RED_CLOSE_ALIGN_SPIKEMARK2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_ALIGN, 0.7, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL3;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL1_COLLECT, 0.5, 0.2)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL2;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL2_COLLECT, 0.50, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL3;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL3_COLLECT, 0.5, 0.0)) {
                    redCloseState = RedCloseState.DRIVE_TO_SCORE3;
                }
                break;

            case DRIVE_TO_SCORE3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,
                            GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    redCloseState = RedCloseState.SHOOT_SEQUENCE3;
                }
                break;

            case SHOOT_SEQUENCE3:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);


                if (runShootSequence(detectedMotif)) {
                    robot.launcher.setMotorVelocity(0, 0);
                    redCloseState = RedCloseState.PARK;
                }
                break;

            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_PARK, 0.8, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", redCloseState);
    }

    //================================================================================
    // RED FAR PATH
    //================================================================================

    private void runRedFarPath() {
        telemetry.addData("Current Path", "Red Far");

        switch (redFarState) {
            case DRIVE_TO_SCORE:
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,
                        GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    redFarState = RedFarState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 2) {
                        redFarState = RedFarState.RED_FAR_ALIGN_SPIKEMARK3;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        redFarState = RedFarState.PARK;
                    }
                }
                break;

            case RED_FAR_ALIGN_SPIKEMARK3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_ALIGN, 0.75, 0.0)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK3_BALL1;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK3_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL1, 0.5, 0.10)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK3_BALL2;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK3_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL2, 0.5, 0.0)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK3_BALL3;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK3_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL3, 0.5, 0.0)) {
                    redFarState = RedFarState.DRIVE_TO_SCORE2;
                }
                break;

            case DRIVE_TO_SCORE2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,
                            GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    redFarState = RedFarState.SHOOT_SEQUENCE2;
                }
                break;

            case SHOOT_SEQUENCE2:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 3) {
                        redFarState = RedFarState.RED_FAR_ALIGN_SPIKEMARK2;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        redFarState = RedFarState.PARK;
                    }
                }
                break;

            case RED_FAR_ALIGN_SPIKEMARK2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_ALIGN, 0.6, 0.0)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK2_BALL1;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK2_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL1, 0.5, 0.1)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK2_BALL2;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK2_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL2, 0.50, 0.0)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK2_BALL3;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK2_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL3, 0.5, 0.0)) {
                    redFarState = RedFarState.DRIVE_TO_SCORE3;
                }
                break;

            case DRIVE_TO_SCORE3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {

                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,
                            GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);

                    redFarState = RedFarState.SHOOT_SEQUENCE3;
                }
                break;

            case SHOOT_SEQUENCE3:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    robot.launcher.setMotorVelocity(0, 0);
                    redFarState = RedFarState.PARK;
                }
                break;

            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_PARK, 0.80, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", redFarState);
    }

    //================================================================================
    // BLUE CLOSE PATH
    //================================================================================

    private void runBlueClosePath() {
        telemetry.addData("Current Path", "Blue Close");

        switch (blueCloseState) {
            case DRIVE_TO_SCORE:
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,
                        GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 2) {
                        blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        blueCloseState = BlueCloseState.PARK;
                    }
                }
                break;

            case GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_ALIGN, 0.75, 0.0)) {
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1;
                }
                break;

            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL1, 0.5, 0.20)) {
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2;
                }
                break;

            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL2, 0.5, 0.0)) {
                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3;
                }
                break;

            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL3, 0.5, 0.0)) {
                    blueCloseState = BlueCloseState.DRIVE_TO_SCORE_AFTER_SPIKE_MARK1;
                }
                break;

            case DRIVE_TO_SCORE_AFTER_SPIKE_MARK1:
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,
                        GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE2;
                }
                break;

            case SHOOT_SEQUENCE2:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 3) {
                        blueCloseState = BlueCloseState.BLUE_CLOSE_ALIGN_SPIKEMARK2;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        blueCloseState = BlueCloseState.PARK;
                    }
                }
                break;

            case BLUE_CLOSE_ALIGN_SPIKEMARK2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_ALIGN, 0.6, 0.0)) {
                    blueCloseState = BlueCloseState.BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL1;
                }
                break;

            case BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL1_COLLECT, 0.5, 0.2)) {
                    blueCloseState = BlueCloseState.BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL2;
                }
                break;

            case BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL2_COLLECT, 0.50, 0.0)) {
                    blueCloseState = BlueCloseState.BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL3;
                }
                break;

            case BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL3_COLLECT, 0.5, 0.0)) {
                    blueCloseState = BlueCloseState.DRIVE_TO_SCORE3;
                }
                break;

            case DRIVE_TO_SCORE3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,
                            GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE3;
                }
                break;

            case SHOOT_SEQUENCE3:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    robot.launcher.setMotorVelocity(0, 0);
                    blueCloseState = BlueCloseState.PARK;
                }
                break;

            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", blueCloseState);
    }

    //================================================================================
    // BLUE FAR PATH
    //================================================================================

    private void runBlueFarPath() {
        telemetry.addData("Current Path", "Blue Far");

        switch (blueFarState) {
            case DRIVE_TO_SCORE:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.5, 0.25)) {
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,
                            GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    blueFarState = BlueFarState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 2) {
                        blueFarState = BlueFarState.BLUE_FAR_ALIGN_SPIKEMARK3;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        blueFarState = BlueFarState.PARK;
                    }
                }
                break;

            case BLUE_FAR_ALIGN_SPIKEMARK3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_ALIGN, 0.75, 0.0)) {
                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK3_BALL1;
                }
                break;

            case BLUE_FAR_COLLECT_SPIKEMARK3_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL1, 0.5, 0.10)) {
                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK3_BALL2;
                }
                break;

            case BLUE_FAR_COLLECT_SPIKEMARK3_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL2, 0.5, 0.0)) {
                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK3_BALL3;
                }
                break;

            case BLUE_FAR_COLLECT_SPIKEMARK3_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL3, 0.5, 0.0)) {
                    blueFarState = BlueFarState.DRIVE_TO_SCORE2;
                }
                break;

            case DRIVE_TO_SCORE2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,
                            GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    blueFarState = BlueFarState.SHOOT_SEQUENCE2;
                }
                break;

            case SHOOT_SEQUENCE2:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 3) {
                        blueFarState = BlueFarState.BLUE_FAR_ALIGN_SPIKEMARK2;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        blueFarState = BlueFarState.PARK;
                    }
                }
                break;

            case BLUE_FAR_ALIGN_SPIKEMARK2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_ALIGN, 0.6, 0.0)) {
                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK2_BALL1;
                }
                break;

            case BLUE_FAR_COLLECT_SPIKEMARK2_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL1, 0.5, 0.1)) {
                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK2_BALL2;
                }
                break;

            case BLUE_FAR_COLLECT_SPIKEMARK2_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL2, 0.50, 0.0)) {
                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK2_BALL3;
                }
                break;

            case BLUE_FAR_COLLECT_SPIKEMARK2_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL3, 0.5, 0.0)) {
                    blueFarState = BlueFarState.DRIVE_TO_SCORE3;
                }
                break;

            case DRIVE_TO_SCORE3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,
                            GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    blueFarState = BlueFarState.SHOOT_SEQUENCE3;
                }
                break;

            case SHOOT_SEQUENCE3:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    robot.launcher.setMotorVelocity(0, 0);
                    blueFarState = BlueFarState.PARK;
                }
                break;

            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", blueFarState);
    }

    //================================================================================
    // SHOT SEQUENCE CONTROLLER INTEGRATION
    //================================================================================

    /**
     * Executes a 3-shot sequence using the ShotSequenceController.
     * The controller handles all sensor reading, spinner rotation, and firing logic.
     *
     * @param motif The detected vision pattern (GPP/PGP/PPG)
     * @return true once all 3 shots have been fired or an error occurred
     */
    private boolean runShootSequence(VisionUtil.MotifPattern motif) {
        // Handle errors
        if (robot.shotSequence.isError()) {
            robot.triggerBothFlippers();
            telemetry.addLine("⚠ Shot sequence encountered an error, continuing...");
            robot.shotSequence.reset();  // Reset for next volley
            return true;
        }

        // Check if done
        if (robot.shotSequence.isDone()) {
            robot.shotSequence.reset();  // Reset for next volley
            return true;
        }

        // Start sequence if it's not currently busy (IDLE or ready to start)
        if (!robot.shotSequence.isBusy()) {
            robot.shotSequence.setLauncherReadyVelocity(1700);//MINIMUM_SAFE_LAUNCHER_VELOCITY
            robot.shotSequence.setRequireLauncherReady(false);
            robot.shotSequence.setBlindFireMode(true);//blindFireMode
            robot.shotSequence.start(motifToShortString(motif));
        }

        // Add telemetry
        robot.shotSequence.addTelemetry(telemetry);

        return false; // Still running
    }

    /**
     * Converts MotifPattern enum to 3-letter shorthand (GPP/PGP/PPG).
     */
    private String motifToShortString(VisionUtil.MotifPattern motif) {
        if (motif == null) return "PPG";
        switch (motif) {
            case GPP21: return "GPP";
            case PGP22: return "PGP";
            case PPG23: return "PPG";
            default: return "PPG";
        }
    }
}