package org.firstinspires.ftc.teamcode.Auto.GearGirls.BOT2.old;


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
 * GearGirls Autonomous - Sensor-Based Shot Sequencing
 *
 * This autonomous program uses color sensors to determine shot sequences instead of
 * relying on pre-programmed patterns based on spike mark pickup positions.
 *
 * Key Updates from Score9:
 * - Uses GGRobot2 class with spinner mechanism
 * - Uses IntakeSensorFusion002 for color detection
 * - Determines shot sequence dynamically based on sensor readings
 * - Preload: Purple in LEFT, Green in RIGHT
 *
 * The robot can score up to 9 game elements:
 * - 3 preloaded (2 purple, 1 green)
 * - 3 from spike mark 1
 * - 3 from spike mark 2
 *
 * @author GearGirls Team
 * @version 1.0 - Sensor-Based
 */
@Autonomous(name="GG AUTO: Score 9 (SENSOR)", group="GGBot", preselectTeleOp = "Gear Girls Bot 2 (RUN ME)")
@Disabled
public class GGAutonomous_Score9_Bot2 extends OpMode {

    // --- Subsystems ---
    private GGRobot2 robot;

    // --- OpMode State and Configuration ---
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    private GGRobotConstants.Location location = GGRobotConstants.Location.CLOSE;
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;
    private int selectedCycles = 2; // Default: 1=Preload only, 2=Preload+Spike1, 3=Preload+Spike1+Spike2

    // --- Master State Machine ---
    private enum AutonomousState { PRE_START, RUNNING_PATH, COMPLETE }
    private AutonomousState autonomousState = AutonomousState.PRE_START;

    // --- Path-Specific State Machines ---
    // Using same state machines as original, but replacing diverter logic with spinner logic
    private enum RedCloseState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        RED_CLOSE_ALIGN_SPIKEMARK1, RED_CLOSE_COLLECT_SPIKEMARK1_BALL1,
        ROTATE_SPINNER_1, WAIT_FOR_SPINNER_1,
        RED_CLOSE_COLLECT_SPIKEMARK1_BALL2, RED_CLOSE_COLLECT_SPIKEMARK1_BALL3,
        DRIVE_TO_SCORE_1, SHOOT_SEQUENCE1,
        RED_CLOSE_ALIGN_SPIKEMARK2, RED_CLOSE_COLLECT_SPIKEMARK2_BALL1,
        ROTATE_SPINNER_2, WAIT_FOR_SPINNER_2,
        RED_CLOSE_COLLECT_SPIKEMARK2_BALL2,
        ROTATE_SPINNER_3, WAIT_FOR_SPINNER_3,
        RED_CLOSE_COLLECT_SPIKEMARK2_BALL3,
        DRIVE_TO_SCORE3, SHOOT_SEQUENCE3, PARK
    }
    private RedCloseState redCloseState = RedCloseState.DRIVE_TO_SCORE;

    private enum RedFarState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        RED_FAR_ALIGN_SPIKEMARK3, RED_FAR_COLLECT_SPIKEMARK3_BALL1,
        RED_FAR_COLLECT_SPIKEMARK3_BALL2, RED_FAR_COLLECT_SPIKEMARK3_BALL3,
        DRIVE_TO_SCORE2,
        ROTATE_SPINNER_1, WAIT_FOR_SPINNER_1,
        SHOOT_SEQUENCE2,
        RED_FAR_ALIGN_SPIKEMARK2, RED_FAR_COLLECT_SPIKEMARK2_BALL1,
        ROTATE_SPINNER_2, WAIT_FOR_SPINNER_2,
        RED_FAR_COLLECT_SPIKEMARK2_BALL2, RED_FAR_COLLECT_SPIKEMARK2_BALL3,
        DRIVE_TO_SCORE3, SHOOT_SEQUENCE3,
        ROTATE_SPINNER_3, WAIT_FOR_SPINNER_3,
        PARK
    }
    private RedFarState redFarState = RedFarState.DRIVE_TO_SCORE;

    private enum BlueCloseState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1,
        GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3a,
        GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3, GO_TO_BLUE_CLOSE_SPIKEMARK1_END,
        DRIVE_TO_SCORE_AFTER_SPIKE_MARK1, SHOOT_SEQUENCE2,
        BLUE_CLOSE_ALIGN_SPIKEMARK2, BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL1,
        ROTATE_SPINNER_1, WAIT_FOR_SPINNER_1,
        BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL2, BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL3,
        DRIVE_TO_SCORE3, SHOOT_SEQUENCE3,
        ROTATE_SPINNER_2, WAIT_FOR_SPINNER_2,
        PARK
    }
    private BlueCloseState blueCloseState = BlueCloseState.DRIVE_TO_SCORE;

    private enum BlueFarState {
        DRIVE_TO_SCORE, SHOOT_SEQUENCE,
        BLUE_FAR_ALIGN_SPIKEMARK3, BLUE_FAR_COLLECT_SPIKEMARK3_BALL1,
        BLUE_FAR_COLLECT_SPIKEMARK3_BALL2, BLUE_FAR_COLLECT_SPIKEMARK3_BALL3,
        DRIVE_TO_SCORE2,
        ROTATE_SPINNER_1, WAIT_FOR_SPINNER_1,
        SHOOT_SEQUENCE2,
        BLUE_FAR_ALIGN_SPIKEMARK2, BLUE_FAR_COLLECT_SPIKEMARK2_BALL1,
        ROTATE_SPINNER_2, WAIT_FOR_SPINNER_2,
        BLUE_FAR_COLLECT_SPIKEMARK2_BALL2, BLUE_FAR_COLLECT_SPIKEMARK2_BALL3,
        DRIVE_TO_SCORE3, SHOOT_SEQUENCE3,
        ROTATE_SPINNER_3, WAIT_FOR_SPINNER_3,
        PARK
    }
    private BlueFarState blueFarState = BlueFarState.DRIVE_TO_SCORE;

    // --- Action-Specific Variables ---
    private int shotsFired = 0;
    private ElapsedTime waitTimer = new ElapsedTime();
    final double WAIT_TIME = 0.250;

    // No complex planning needed - we'll read sensors before each shot!

    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot2(hardwareMap, telemetry);
        robot.vision.setPipeline(0); // Ensure correct pipeline is active
        telemetry.addData(">", "Robot Initialized. Ready for selections.");
    }

    @Override
    public void init_loop() {
        robot.vision.setMotifDetectionMode();
        robot.vision.update(); // Continuously update vision to detect the motif
        robot.update();

        // --- Driver Selections ---
        if (gamepad1.x) { alliance = CommonConstants.Alliance.BLUE; }
        if (gamepad1.b) { alliance = CommonConstants.Alliance.RED; }
        if (gamepad1.y) { location = GGRobotConstants.Location.CLOSE; }
        if (gamepad1.a) { location = GGRobotConstants.Location.FAR; }

        // --- Cycle Selection ---
        if (gamepad1.dpadLeftWasPressed() && selectedCycles > 1) { selectedCycles--; }
        if (gamepad1.dpadRightWasPressed() && selectedCycles < 3) { selectedCycles++; }

        // --- Vision Detection ---
        detectedMotif = robot.vision.getMotifPattern();

        // --- Telemetry Feedback ---
        telemetry.addLine("=== SENSOR-BASED AUTONOMOUS ===");
        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addData("Cycles", "%d (DPad Left/Right)", selectedCycles);
        telemetry.addLine("  1 = Preload only");
        telemetry.addLine("  2 = Preload + Spike Mark 1");
        telemetry.addLine("  3 = Preload + Spike Mark 1 + Spike Mark 2");
        telemetry.addData("Detected Motif", detectedMotif);
        telemetry.addLine("\nPreload Config: LEFT=Purple, RIGHT=Green");
        telemetry.addLine("Shot sequence will be determined by sensors!");
        telemetry.addLine("\nReady to Start!");
        telemetry.addData("X: ", robot.drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y: ", robot.drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Heading: ", robot.drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }

    @Override
    public void start() {
        // Set the robot's starting position based on the final selections
        if (location == GGRobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_CLOSE : GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else { // FAR
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_FAR : GGRobotConstants.Waypoints.START_BLUE_FAR);
        }

        // Save alliance to SharedState for TeleOp
        SharedState.alliance = alliance;

        // Set the pipeline to ONLY look for our alliance's scoring tags
        robot.vision.setTargetingAlliance(alliance);

        // Start spinning up the launcher motor immediately
        robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY,GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY);

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
                robot.intake.intakeOn();
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
                // Path is done. Stop all motors and end the OpMode
                robot.stopAll();
                requestOpModeStop();
                break;
        }

        // Add telemetry from the robot object
        robot.addTelemetry();
        telemetry.update();
    }

    //================================================================================
    // SIMPLIFIED SENSOR-BASED SHOOTING
    //================================================================================

    /**
     * Executes shots one at a time based on the motif pattern.
     * Reads sensors before EACH shot to determine which side to fire.
     *
     * @param shotNumber The current shot (0, 1, or 2)
     * @param motif The detected vision pattern
     * @return true if shot was fired, false if waiting for flipper to complete
     */
    private boolean executeShotSimple(int shotNumber, VisionUtil.MotifPattern motif) {
        // Wait for previous shot to complete
        if (robot.areFlippersBusy()) {
            return false;
        }

        // Determine what color we need for this shot based on the motif
        IntakeSensorFusion002.ArtifactColor neededColor = getNeededColorForShot(shotNumber, motif);

        // Read the sensors RIGHT NOW
        IntakeSensorFusion002.ArtifactColor leftColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT);
        IntakeSensorFusion002.ArtifactColor rightColor = robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT);

        // Fire whichever side has the color we need
        if (leftColor == neededColor) {
            robot.triggerLeftFlipper();
            telemetry.addData("Shot", "%d: Firing LEFT (%s)", shotNumber + 1, neededColor);
        } else if (rightColor == neededColor) {
            robot.triggerRightFlipper();
            telemetry.addData("Shot", "%d: Firing RIGHT (%s)", shotNumber + 1, neededColor);
        } else {
            // Color not found in either sensor - default to left
            telemetry.addLine("⚠ WARNING: " + neededColor + " not found, defaulting to LEFT");
            robot.triggerLeftFlipper();
        }

        telemetry.addData("Sensors", "L=%s, R=%s | Need=%s", leftColor, rightColor, neededColor);

        return true;
    }

    /**
     * Simple helper: What color do we need for this shot number?
     *
     * @param shotNumber The shot we're taking (0, 1, or 2)
     * @param motif The detected vision pattern
     * @return The color needed for this shot
     */
    private IntakeSensorFusion002.ArtifactColor getNeededColorForShot(
            int shotNumber, VisionUtil.MotifPattern motif) {

        switch (motif) {
            case GPP21: // Green, Purple, Purple
                return (shotNumber == 0) ? IntakeSensorFusion002.ArtifactColor.GREEN
                        : IntakeSensorFusion002.ArtifactColor.PURPLE;

            case PGP22: // Purple, Green, Purple
                return (shotNumber == 1) ? IntakeSensorFusion002.ArtifactColor.GREEN
                        : IntakeSensorFusion002.ArtifactColor.PURPLE;

            case PPG23: // Purple, Purple, Green
            case UNKNOWN:
            default:
                return (shotNumber == 2) ? IntakeSensorFusion002.ArtifactColor.GREEN
                        : IntakeSensorFusion002.ArtifactColor.PURPLE;
        }
    }

    //================================================================================
    // RED CLOSE PATH
    //================================================================================

    private void runRedClosePath() {
        switch (redCloseState) {
            case DRIVE_TO_SCORE:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    // No planning needed - we read sensors before each shot!
                    redCloseState = RedCloseState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.25, 0.0);

                if (shotsFired < 3) {
                    if (executeShotSimple(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                    // All 3 shots fired - move to next state based on cycle selection
                    if (selectedCycles == 1) {
                        robot.launcher.stopMotors();
                        redCloseState = RedCloseState.PARK;
                    } else {
                        redCloseState = RedCloseState.RED_CLOSE_ALIGN_SPIKEMARK1;
                    }
                }
                break;

            case RED_CLOSE_ALIGN_SPIKEMARK1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL1, 0.5, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL1;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL2, 0.5, 0.0)) {
                    redCloseState = RedCloseState.ROTATE_SPINNER_1;
                }
                break;

            case ROTATE_SPINNER_1:
                robot.rotateSpinnerLeft();
                waitTimer.reset();
                redCloseState = RedCloseState.WAIT_FOR_SPINNER_1;
                break;

            case WAIT_FOR_SPINNER_1:
                if (waitTimer.seconds() >= WAIT_TIME) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL2;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL3, 0.5, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL3;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
                    redCloseState = RedCloseState.DRIVE_TO_SCORE_1;
                }
                break;

            case DRIVE_TO_SCORE_1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    // Sensors will be read before each shot
                    redCloseState = RedCloseState.SHOOT_SEQUENCE1;
                }
                break;

            case SHOOT_SEQUENCE1:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.25, 0.0);

                if (shotsFired < 3) {
                    if (executeShotSimple(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                    if (selectedCycles == 2) {
                        robot.launcher.stopMotors();
                        redCloseState = RedCloseState.PARK;
                    } else {
                        redCloseState = RedCloseState.RED_CLOSE_ALIGN_SPIKEMARK2;
                    }
                }
                break;

            // Continue with spike mark 2 collection...
            case RED_CLOSE_ALIGN_SPIKEMARK2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_ALIGN, 0.5, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL1;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL1_COLLECT, 0.5, 0.0)) {
                    redCloseState = RedCloseState.ROTATE_SPINNER_2;
                }
                break;

            case ROTATE_SPINNER_2:
                robot.rotateSpinnerLeft();
                waitTimer.reset();
                redCloseState = RedCloseState.WAIT_FOR_SPINNER_2;
                break;

            case WAIT_FOR_SPINNER_2:
                if (waitTimer.seconds() >= WAIT_TIME) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL2;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL2_COLLECT, 0.5, 0.0)) {
                    redCloseState = RedCloseState.ROTATE_SPINNER_3;
                }
                break;

            case ROTATE_SPINNER_3:
                robot.rotateSpinnerRight();
                waitTimer.reset();
                redCloseState = RedCloseState.WAIT_FOR_SPINNER_3;
                break;

            case WAIT_FOR_SPINNER_3:
                if (waitTimer.seconds() >= WAIT_TIME) {
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
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    // Sensors will be read before each shot
                    redCloseState = RedCloseState.SHOOT_SEQUENCE3;
                }
                break;

            case SHOOT_SEQUENCE3:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.25, 0.0);

                if (shotsFired < 3) {
                    if (executeShotSimple(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                    robot.launcher.stopMotors();
                    redCloseState = RedCloseState.PARK;
                }
                break;

            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_CLOSE_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", redCloseState);
    }

    //================================================================================
    // RED FAR PATH (Similar structure to Red Close, adapted for far waypoints)
    //================================================================================

    private void runRedFarPath() {
        // Implementation follows same pattern as Red Close but with different waypoints

        switch (redFarState) {
            case DRIVE_TO_SCORE:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    redFarState = RedFarState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.25, 0.0);

                if (shotsFired < 3) {
                    if (executeShotSimple(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                    if (selectedCycles == 1) {
                        robot.launcher.stopMotors();
                        redFarState = RedFarState.PARK;
                    } else {
                        redFarState = RedFarState.RED_FAR_ALIGN_SPIKEMARK3;
                    }
                }
                break;

            // TODO: Complete remaining states following Red Close pattern

            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.RED_FAR_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            default:
                redFarState = RedFarState.PARK;
                break;
        }
        telemetry.addData("Path State", redFarState);
    }

    //================================================================================
    // BLUE CLOSE PATH
    //================================================================================

    private void runBlueClosePath() {
        // Implementation follows same sensor-based pattern
        switch (blueCloseState) {
            case DRIVE_TO_SCORE:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY,GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.25, 0.0);

                if (shotsFired < 3) {
                    if (executeShotSimple(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                    if (selectedCycles == 1) {
                        robot.launcher.stopMotors();
                        blueCloseState = BlueCloseState.PARK;
                    } else {
                        blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN;
                    }
                }
                break;

            // Additional states following same pattern...
            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_CLOSE_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            // TODO: Complete remaining states
            default:
                telemetry.addLine("⚠ BLUE CLOSE: State not fully implemented yet");
                blueCloseState = BlueCloseState.PARK;
                break;
        }
        telemetry.addData("Path State", blueCloseState);
    }

    //================================================================================
    // BLUE FAR PATH
    //================================================================================

    private void runBlueFarPath() {
        switch (blueFarState) {
            case DRIVE_TO_SCORE:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY,GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    blueFarState = BlueFarState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.25, 0.0);

                if (shotsFired < 3) {
                    if (executeShotSimple(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                    if (selectedCycles == 1) {
                        robot.launcher.stopMotors();
                        blueFarState = BlueFarState.PARK;
                    } else {
                        blueFarState = BlueFarState.BLUE_FAR_ALIGN_SPIKEMARK3;
                    }
                }
                break;

            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        GGRobotConstants.Waypoints.BLUE_FAR_PARK, 0.5, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            // TODO: Complete remaining states
            default:
                telemetry.addLine("⚠ BLUE FAR: State not fully implemented yet");
                blueFarState = BlueFarState.PARK;
                break;
        }
        telemetry.addData("Path State", blueFarState);
    }
}