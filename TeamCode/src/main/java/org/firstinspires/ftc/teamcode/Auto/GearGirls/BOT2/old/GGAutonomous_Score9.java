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
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchIndexer;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;

/**
 * A simplified, explicit state-machine-based autonomous OpMode.
 * This structure is designed to be easy to read and follow for new programmers.
 * It uses dedicated helper methods for each of the four starting paths.
 *
 * @author Your Team Name
 * @version 004
 */
@Autonomous(name="bot 2 GG AUTO: Score 9", group="GGBot", preselectTeleOp = "Gear Girls Telop (RUN ME)")
@Disabled
public class GGAutonomous_Score9 extends OpMode {

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
    private enum RedCloseState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, RED_CLOSE_ALIGN_SPIKEMARK1, RED_CLOSE_COLLECT_SPIKEMARK1_BALL1, SET_DIVERTER_LEFT, WAIT_FOR_DIVERTER, RED_CLOSE_COLLECT_SPIKEMARK1_BALL2, RED_CLOSE_COLLECT_SPIKEMARK1_BALL3, DRIVE_TO_SCORE_1, SHOOT_SEQUENCE1, RED_CLOSE_ALIGN_SPIKEMARK2, RED_CLOSE_COLLECT_SPIKEMARK2_BALL1, SET_DIVERTER_LEFT2, WAIT_FOR_DIVERTER2, RED_CLOSE_COLLECT_SPIKEMARK2_BALL2, SET_DIVERTER_RIGHT2, WAIT_FOR_DIVERTER3, RED_CLOSE_COLLECT_SPIKEMARK2_BALL3, DRIVE_TO_SCORE3, SHOOT_SEQUENCE3, PARK } private RedCloseState redCloseState = RedCloseState.DRIVE_TO_SCORE;

    private enum RedFarState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, RED_FAR_ALIGN_SPIKEMARK3, RED_FAR_COLLECT_SPIKEMARK3_BALL1, RED_FAR_COLLECT_SPIKEMARK3_BALL2, RED_FAR_COLLECT_SPIKEMARK3_BALL3, DRIVE_TO_SCORE2, SET_DIVERTER_RIGHT, WAIT_FOR_DIVERTER, SHOOT_SEQUENCE2, RED_FAR_ALIGN_SPIKEMARK2, RED_FAR_COLLECT_SPIKEMARK2_BALL1, SET_DIVERTER_LEFT2, WAIT_FOR_DIVERTER2, RED_FAR_COLLECT_SPIKEMARK2_BALL2, RED_FAR_COLLECT_SPIKEMARK2_BALL3, DRIVE_TO_SCORE3, SHOOT_SEQUENCE3, WAIT_FOR_DIVERTER3, SET_DIVERTER_RIGHT2, PARK } private RedFarState redFarState = RedFarState.DRIVE_TO_SCORE;
    private enum BlueCloseState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3a, GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3, GO_TO_BLUE_CLOSE_SPIKEMARK1_END, DRIVE_TO_SCORE_AFTER_SPIKE_MARK1, SHOOT_SEQUENCE2, BLUE_CLOSE_ALIGN_SPIKEMARK2, BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL1, SET_DIVERTER_RIGHT2, WAIT_FOR_DIVERTER2, BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL2, BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL3, DRIVE_TO_SCORE3, SHOOT_SEQUENCE3, SET_DIVERTER_LEFT, WAIT_FOR_DIVERTER, PARK }  private BlueCloseState blueCloseState = BlueCloseState.DRIVE_TO_SCORE;
    private enum BlueFarState { DRIVE_TO_SCORE, SHOOT_SEQUENCE, BLUE_FAR_ALIGN_SPIKEMARK3, BLUE_FAR_COLLECT_SPIKEMARK3_BALL1, BLUE_FAR_COLLECT_SPIKEMARK3_BALL2, BLUE_FAR_COLLECT_SPIKEMARK3_BALL3, DRIVE_TO_SCORE2, SET_DIVERTER_RIGHT, WAIT_FOR_DIVERTER, SHOOT_SEQUENCE2, BLUE_FAR_ALIGN_SPIKEMARK2, BLUE_FAR_COLLECT_SPIKEMARK2_BALL1, SET_DIVERTER_LEFT2, WAIT_FOR_DIVERTER2, BLUE_FAR_COLLECT_SPIKEMARK2_BALL2, BLUE_FAR_COLLECT_SPIKEMARK2_BALL3, DRIVE_TO_SCORE3, SHOOT_SEQUENCE3, WAIT_FOR_DIVERTER3, SET_DIVERTER_RIGHT2, PARK } private BlueFarState blueFarState = BlueFarState.DRIVE_TO_SCORE;

    // --- Action-Specific Variables ---
    private int shotsFired = 0;
    private double shotVelocity = 0;
    private ElapsedTime waitTimer = new ElapsedTime();
    final double WAIT_TIME = .250;

    private boolean spinnerIsRotated = false;

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
        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", "%s (X=Blue, B=Red)", alliance);
        telemetry.addData("Location", "%s (Y=Close, A=Far)", location);
        telemetry.addData("Cycles", "%d (DPad Left/Right)", selectedCycles);
        telemetry.addLine("  1 = Preload only");
        telemetry.addLine("  2 = Preload + Spike Mark 1");
        telemetry.addLine("  3 = Preload + Spike Mark 1 + Spike Mark 2");
        telemetry.addData("Detected Motif", detectedMotif);
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
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_CLOSE : GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else { // FAR
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ? GGRobotConstants.Waypoints.START_RED_FAR : GGRobotConstants.Waypoints.START_BLUE_FAR);
        }

        // Set the pipeline to ONLY look for our alliance's scoring tags.
        robot.vision.setTargetingAlliance(alliance);

        // Start spinning up the launcher motor immediately.
        robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY, GGRobotConstants.Launcher.AUTO_TARGET_VELOCITY);

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
                robot.intake.setIntakeMotorPower(-1);
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

    //================================================================================
    // PATH-SPECIFIC HELPER METHODS
    //================================================================================
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
    private boolean executeShot(int shotNumber, VisionUtil.MotifPattern motif) {

        if (robot.launcher.getLeftMotorVelocity() < shotVelocity-125) return false;

        // Wait for previous shot to complete
        if (robot.areFlippersBusy() || robot.isSpinnerBusy()) {
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
    /**
     * Runs the state machine for the RED alliance, CLOSE starting position.
     * This method contains the "script" for this specific path.
     */
    private void runRedClosePath() {
        telemetry.addData("Current Path", "Red Close");

        switch (redCloseState) {
            case DRIVE_TO_SCORE:
                shotsFired = 0;
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);

                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    redCloseState = RedCloseState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);

                if (shotsFired < 2) {
                    if (executeShot(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else if (shotsFired == 2) {
                    //once we've shot two balls, we need to rotate the spinner to shoot the third ball
                    if(!robot.areFlippersBusy()) {
                        if(!robot.spinner.isRotated()) {
                            robot.rotateSpinnerLeft();
                            //spinnerIsRotated = true; //TODO: This should maybe be done in the spinner class.
                                                     // There needs to be a way for us to keep track of when it's in the rotated state,
                                                     // because the next rotation has to be rotateRight()
                            //done
                        } else {
                            robot.rotateSpinnerRight();
                        }
                    }
                    if (executeShot(shotsFired, detectedMotif)) {
                        shotsFired++;
                        //rotate back to starting position
                        robot.rotateSpinnerRight(); //TODO: Figure out why the spinner isn't spinning here. It might just be a timing issue, where the robot is still in the SPINNING state
                        spinnerIsRotated = false;
                    }

                }
                else {
                        // CYCLE SELECTION LOGIC: Check if we should do spike mark 1
                        //also, move to the next state if we've fired all three shots.
                        if (selectedCycles >= 2 && shotsFired == 3) {
                            redCloseState = RedCloseState.RED_CLOSE_ALIGN_SPIKEMARK1;
                        } else {
                            // Skip directly to park
                            robot.launcher.setMotorVelocity(0, 0);
                            redCloseState = RedCloseState.PARK;
                        }
                }
                break;

            case RED_CLOSE_ALIGN_SPIKEMARK1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_ALIGN, 0.75, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL1;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL1, 0.5, 0.20)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL2;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL2, 0.5, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK1_BALL3;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK1_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL3, 0.5, 0.0)) {
                    redCloseState = RedCloseState.DRIVE_TO_SCORE_1;
                }
                break;

            case DRIVE_TO_SCORE_1:
                shotsFired = 0;
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);

                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    redCloseState = RedCloseState.SHOOT_SEQUENCE1;
                }
                break;

            case SHOOT_SEQUENCE1:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);

                if (shotsFired < 3) {
                    if (executeShot(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                    // CYCLE SELECTION LOGIC: Check if we should do spike mark 2
                    if (selectedCycles >= 3) {
                        redCloseState = RedCloseState.RED_CLOSE_ALIGN_SPIKEMARK2;
                    } else {
                        // Skip directly to park
                        robot.launcher.setMotorVelocity(0, 0);
                        redCloseState = RedCloseState.PARK;
                    }
                }
                break;

            case RED_CLOSE_ALIGN_SPIKEMARK2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_ALIGN, 0.6, 0.0)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL1;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL1:
                //swap middle bar -op-or+ - after first bar - third spike mark , frwd 1, swp, fwd 1, p  , frwd 1
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL1_COLLECT, 0.5, 0.2)) {
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL2;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL2:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL2_COLLECT, 0.50, 0.0)){
                    redCloseState = RedCloseState.RED_CLOSE_COLLECT_SPIKEMARK2_BALL3;
                }
                break;

            case RED_CLOSE_COLLECT_SPIKEMARK2_BALL3:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL3_COLLECT, 0.5, 0.0)){
                    redCloseState = RedCloseState.DRIVE_TO_SCORE3;
                }
                break;

            case DRIVE_TO_SCORE3:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    redCloseState = RedCloseState.SHOOT_SEQUENCE3;
                }
                break;
            case SHOOT_SEQUENCE3:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE, 0.25, 0.0);
                if (shotsFired < 3) {
                    if (executeShot(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                } else {
                            // All 3 shots are fired. Turn off the launcher and move to the park state.
                            robot.launcher.setMotorVelocity(0, 0);
                            redCloseState = RedCloseState.PARK;
                }
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

    /**
     * Runs the state machine for the RED alliance, FAR starting position.
     */
    private void runRedFarPath() {
        telemetry.addData("Current Path", "Red Far");
        // x= 8.3816 y= 12.0461 h= -25.3986
        switch (redFarState) {
            case DRIVE_TO_SCORE:
                shotsFired = 0;
                shotVelocity = GGRobotConstants.Launcher.FAR_TARGET_VELOCITY;
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                redFarState = RedFarState.SHOOT_SEQUENCE;

                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    redFarState = RedFarState.SHOOT_SEQUENCE;
                }
                break;

            case SHOOT_SEQUENCE:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.5, 0.0);
                    if (shotsFired < 3) {
                        if (executeShot(shotsFired, detectedMotif)) {
                            shotsFired++;
                        }
                    } else {
                        // CYCLE SELECTION LOGIC: Check if we should do spike mark 3 (first one from far)
                        if (selectedCycles >= 2) {
                            redFarState = RedFarState.RED_FAR_ALIGN_SPIKEMARK3;
                        } else {
                            // Skip directly to park
                            robot.launcher.setMotorVelocity(0, 0);
                            redFarState = RedFarState.PARK;
                        }
                    }
                break;

            case RED_FAR_ALIGN_SPIKEMARK3:

                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_ALIGN, 0.75, 0.0)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK3_BALL1;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK3_BALL1:
                //swap middle bar -op-or+ - after first bar - third spike mark , frwd 1, swp, fwd 1, p  , frwd 1
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL1, 0.5, 0.10)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK3_BALL2;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK3_BALL2:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL2, 0.5, 0.0)){
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK3_BALL3;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK3_BALL3:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL3, 0.5, 0.0)){
                    redFarState = RedFarState.DRIVE_TO_SCORE2;
                }
                break;

            case DRIVE_TO_SCORE2:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    redFarState = RedFarState.SHOOT_SEQUENCE2;
                }
                break;

            case SHOOT_SEQUENCE2:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0);
                if (shotsFired < 3) {
                    if (executeShot(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                    } else {
                        // CYCLE SELECTION LOGIC: Check if we should do spike mark 2
                        if (selectedCycles >= 3) {
                            redFarState = RedFarState.RED_FAR_ALIGN_SPIKEMARK2;
                        } else {
                            // Skip directly to park
                            robot.launcher.setMotorVelocity(0, 0);
                            redFarState = RedFarState.PARK;
                        }
                    }
                break;

            case RED_FAR_ALIGN_SPIKEMARK2:

                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_ALIGN, 0.6, 0.0)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK2_BALL1;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK2_BALL1:
                //swap middle bar -op-or+ - after first bar - third spike mark , frwd 1, swp, fwd 1, p  , frwd 1
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL1, 0.5, 0.1)) {
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK2_BALL2;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK2_BALL2:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL2, 0.50, 0.0)){
                    //redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK2_BALL3;
                    redFarState = RedFarState.RED_FAR_COLLECT_SPIKEMARK2_BALL3;
                }
                break;

            case RED_FAR_COLLECT_SPIKEMARK2_BALL3:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL3, 0.5, 0.0)){
                    redFarState = RedFarState.DRIVE_TO_SCORE3;
                }
                break;

            case DRIVE_TO_SCORE3:
                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    redFarState = RedFarState.SHOOT_SEQUENCE3;
                }
                break;

            case SHOOT_SEQUENCE3:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE, 0.25, 0.0);
                if (shotsFired < 3) {
                    if (executeShot(shotsFired, detectedMotif)) {
                        shotsFired++;
                    }
                    } else {

                        // All 3 shots are fired. Turn off the launcher and move to the park state.
                        robot.launcher.setMotorVelocity(0, 0);
                        redFarState = RedFarState.PARK;
                    }
                break;

            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.RED_FAR_PARK, .80, 0.25)) {
                    // Once parking is complete, the entire autonomous routine is done.
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", redFarState);
    }


    private void runBlueClosePath() {
        telemetry.addData("Current Path", "Blue Close");

        switch (blueCloseState) {
            case DRIVE_TO_SCORE:
                shotsFired = 0;
                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);

                // Drive to the scoring position.
                // The driveTo() method is non-blocking and returns true when complete.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
                    // When the drive is done, reset the shot counter and move to the shooting state.
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
                    blueCloseState = BlueCloseState.PARK;
                }
                break;
//
//            case SHOOT_SEQUENCE:
//                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);
//
//                if (!robot.feeder.isBusy()) {
//                    if (shotsFired < 3) {
//                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
//                        if(robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.CLOSE)) {
//                            shotsFired++;
//                        }
//                    } else {
//                        // CYCLE SELECTION LOGIC: Check if we should do spike mark 1
//                        if (selectedCycles >= 2) {
//                            blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN;
//                        } else {
//                            // Skip directly to park
//                            robot.launcher.setMotorVelocity(0, 0);
//                            blueCloseState = BlueCloseState.PARK;
//                        }
//                    }
//                }
//                break;
//
//            case GO_TO_BLUE_CLOSE_SPIKEMARK1_ALIGN:
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_ALIGN, 0.75, 0.0)) {
//                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1;
//                } else
//                {
//                    robot.intake.setDiverterRight();
//                }
//                break;
//
//            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL1:
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL1, 0.5, 0.20)) {
//                    blueCloseState = BlueCloseState.SET_DIVERTER_LEFT;
//                }
//                break;
//
//            case SET_DIVERTER_LEFT:
//                robot.intake.setDiverterLeft();
//                waitTimer.reset();
//                blueCloseState = BlueCloseState.WAIT_FOR_DIVERTER;
//                break;
//
//            case WAIT_FOR_DIVERTER:
//                if (waitTimer.seconds() >= WAIT_TIME) {
//                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2;
//                }
//                break;
//
//            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL2:
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL2, 0.5, 0.0)) {
//                    blueCloseState = BlueCloseState.GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3;
//                }
//                break;
//
//            case GO_TO_BLUE_CLOSE_SPIKEMARK1_BALL3:
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL3, 0.5, 0.0)) {
//                    blueCloseState = BlueCloseState.DRIVE_TO_SCORE_AFTER_SPIKE_MARK1;
//                }
//                break;
//
//            case DRIVE_TO_SCORE_AFTER_SPIKE_MARK1:
//                shotsFired = 0;
//                robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
//
//                // Drive to the scoring position.
//                // The driveTo() method is non-blocking and returns true when complete.
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0)) {
//                    // When the drive is done, reset the shot counter and move to the shooting state.
//                    shotsFired = 0;
//                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
//                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE2;
//                }
//                break;
//
//            case SHOOT_SEQUENCE2:
//                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.5, 0.0);
//
//                if (!robot.feeder.isBusy()) {
//                    if (shotsFired < 3) {
//                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
//                        if(robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.CLOSE)) {
//                            shotsFired++;
//                        }
//                    } else {
//                        if(robot.launchSequence(true, LaunchIndexer.FeederSide.RIGHT, GGRobotConstants.LauncherDistance.CLOSE)) {
//                            // CYCLE SELECTION LOGIC: Check if we should do spike mark 2
//                            if (selectedCycles >= 3) {
//                                blueCloseState = BlueCloseState.BLUE_CLOSE_ALIGN_SPIKEMARK2;
//                            } else {
//                                // Skip directly to park
//                                robot.launcher.setMotorVelocity(0, 0);
//                                blueCloseState = BlueCloseState.PARK;
//                            }
//                        }
//                    }
//                }
//                break;
//
//            case BLUE_CLOSE_ALIGN_SPIKEMARK2:
//
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_ALIGN, 0.6, 0.0)) {
//                    blueCloseState = BlueCloseState.BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL1;
//                } else {
//                    robot.intake.setDiverterRight();
//                }
//                break;
//
//            case BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL1:
//                //swap middle bar -op-or+ - after first bar - third spike mark , frwd 1, swp, fwd 1, p  , frwd 1
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL1_COLLECT, 0.5, 0.2)) {
//                    blueCloseState = BlueCloseState.SET_DIVERTER_RIGHT2;
//                }
//                break;
//
//            case SET_DIVERTER_RIGHT2:
//                robot.intake.setDiverterLeft();
//                waitTimer.reset();
//                blueCloseState = BlueCloseState.WAIT_FOR_DIVERTER2;
//                break;
//
//            case WAIT_FOR_DIVERTER2:
//                if (waitTimer.seconds() >= WAIT_TIME) {
//                    blueCloseState = BlueCloseState.BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL2;
//                }
//                break;
//
//            case BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL2:
//                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL2_COLLECT, 0.50, 0.0)){
//                    blueCloseState = BlueCloseState.BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL3;
//                }
//                break;
//
//            case BLUE_CLOSE_COLLECT_SPIKEMARK2_BALL3:
//                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL3_COLLECT, 0.5, 0.0)){
//                    blueCloseState = BlueCloseState.DRIVE_TO_SCORE3;
//                }
//                break;
//
//            case DRIVE_TO_SCORE3:
//                // Drive to the scoring position.
//                // The driveTo() method is non-blocking and returns true when complete.
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.8, 0.0)) {
//                    // When the drive is done, reset the shot counter and move to the shooting state.
//                    shotsFired = 0;
//                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY, GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY);
//                    blueCloseState = BlueCloseState.SHOOT_SEQUENCE3;
//                }
//                break;
//            case SHOOT_SEQUENCE3:
//                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE, 0.25, 0.0);
//                if (!robot.feeder.isBusy()) {
//                    if (shotsFired < 3) {
//                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
//                        if (robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.CLOSE)) {
//                            shotsFired++;
//                        }
//                    } else {
//                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
//                        if (robot.launchSequence(true, LaunchIndexer.FeederSide.RIGHT, GGRobotConstants.LauncherDistance.CLOSE)) {
//                            // All 3 shots are fired. Turn off the launcher and move to the park state.
//                            robot.launcher.setMotorVelocity(0, 0);
//                            blueCloseState = BlueCloseState.PARK;
//                        }
//                    }
//                }
//                break;

            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_CLOSE_PARK, 0.5, 0.25)) {
                    // Once parking is complete, the entire autonomous routine is done.
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

        }
        telemetry.addData("Path State", blueCloseState);
    }

    /**
     * Runs the state machine for the BLUE alliance, FAR starting position.
     */
    private void runBlueFarPath() {
        telemetry.addData("Current Path", "Blue Far");
        // x= 8.3816 y= 12.0461 h= -25.3986
        switch (blueFarState) {
            case DRIVE_TO_SCORE:
                // Drive to the scoring position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.5, 0.25)) {
                    shotsFired = 0;
                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
                    blueFarState = BlueFarState.PARK;
                }
                break;
//
//            case SHOOT_SEQUENCE:
//                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.5, 0);
//                if (!robot.feeder.isBusy()) {
//                    if (shotsFired < 3) {
//                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
//                        if(robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.FAR)) {
//                            shotsFired++;
//                        }
//                    } else {
//                        // CYCLE SELECTION LOGIC: Check if we should do spike mark 1
//                        if (selectedCycles >= 2) {
//                            blueFarState = BlueFarState.BLUE_FAR_ALIGN_SPIKEMARK3;
//                        } else {
//                            // Skip directly to park
//                            robot.launcher.setMotorVelocity(0, 0);
//                            blueFarState = BlueFarState.PARK;
//                        }
//                    }
//                }
//                break;
//
//            case BLUE_FAR_ALIGN_SPIKEMARK3:
//
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_ALIGN, 0.75, 0.0)) {
//                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK3_BALL1;
//                } else {
//                    robot.intake.setDiverterLeft();
//                }
//                break;
//
//            case BLUE_FAR_COLLECT_SPIKEMARK3_BALL1:
//                //swap middle bar -op-or+ - after first bar - third spike mark , frwd 1, swp, fwd 1, p  , frwd 1
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL1, 0.5, 0.10)) {
//                    blueFarState = BlueFarState.SET_DIVERTER_RIGHT;
//                }
//                break;
//
//            case SET_DIVERTER_RIGHT:
//                robot.intake.setDiverterRight();
//                waitTimer.reset();
//                blueFarState = BlueFarState.WAIT_FOR_DIVERTER;
//                break;
//
//            case WAIT_FOR_DIVERTER:
//                if (waitTimer.seconds() >= WAIT_TIME) {
//                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK3_BALL2;
//                }
//                break;
//
//            case BLUE_FAR_COLLECT_SPIKEMARK3_BALL2:
//                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL2, 0.5, 0.0)){
//                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK3_BALL3;
//                }
//                break;
//
//            case BLUE_FAR_COLLECT_SPIKEMARK3_BALL3:
//                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL3, 0.5, 0.0)){
//                    blueFarState = BlueFarState.DRIVE_TO_SCORE2;
//                }
//                break;
//
//            case DRIVE_TO_SCORE2:
//                // Drive to the scoring position.
//                // The driveTo() method is non-blocking and returns true when complete.
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
//                    // When the drive is done, reset the shot counter and move to the shooting state.
//                    shotsFired = 0;
//                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
//                    blueFarState = BlueFarState.SHOOT_SEQUENCE2;
//                }
//                break;
//
//            case SHOOT_SEQUENCE2:
//                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.8, 0.0);
//                if (!robot.feeder.isBusy()) {
//                    if (shotsFired < 3) {
//                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
//                        if (robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.FAR)) {
//                            shotsFired++;
//                        }
//                    } else {
//                        // CYCLE SELECTION LOGIC: Check if we should do spike mark 2
//                        if (selectedCycles >= 3) {
//                            blueFarState = BlueFarState.BLUE_FAR_ALIGN_SPIKEMARK2;
//                        } else {
//                            // Skip directly to park
//                            robot.launcher.setMotorVelocity(0, 0);
//                            blueFarState = BlueFarState.PARK;
//                        }
//                    }
//                }
//                break;
//
//            case BLUE_FAR_ALIGN_SPIKEMARK2:
//
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_ALIGN, 0.6, 0.0)) {
//                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK2_BALL1;
//                } else {
//                    robot.intake.setDiverterRight();
//                }
//                break;
//
//            case BLUE_FAR_COLLECT_SPIKEMARK2_BALL1:
//                //swap middle bar -op-or+ - after first bar - third spike mark , frwd 1, swp, fwd 1, p  , frwd 1
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL1, 0.5, 0.1)) {
//                    blueFarState = BlueFarState.SET_DIVERTER_LEFT2;
//                }
//                break;
//
//            case SET_DIVERTER_LEFT2:
//                robot.intake.setDiverterLeft();
//                waitTimer.reset();
//                blueFarState = BlueFarState.WAIT_FOR_DIVERTER2;
//                break;
//
//            case WAIT_FOR_DIVERTER2:
//                if (waitTimer.seconds() >= WAIT_TIME) {
//                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK2_BALL2;
//                }
//                break;
//
//            case BLUE_FAR_COLLECT_SPIKEMARK2_BALL2:
//                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL2, 0.50, 0.0)){
//                    blueFarState = BlueFarState.SET_DIVERTER_RIGHT2;
//                }
//                break;
//
//            case SET_DIVERTER_RIGHT2:
//                robot.intake.setDiverterRight();
//                waitTimer.reset();
//                blueFarState = BlueFarState.WAIT_FOR_DIVERTER3;
//                break;
//
//            case WAIT_FOR_DIVERTER3:
//                if (waitTimer.seconds() >= WAIT_TIME) {
//                    blueFarState = BlueFarState.BLUE_FAR_COLLECT_SPIKEMARK2_BALL3;
//                }
//                break;
//
//            case BLUE_FAR_COLLECT_SPIKEMARK2_BALL3:
//                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL3, 0.5, 0.0)){
//                    blueFarState = BlueFarState.DRIVE_TO_SCORE3;
//                }
//                break;
//
//            case DRIVE_TO_SCORE3:
//                // Drive to the scoring position.
//                // The driveTo() method is non-blocking and returns true when complete.
//                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.8, 0.0)) {
//                    // When the drive is done, reset the shot counter and move to the shooting state.
//                    shotsFired = 0;
//                    robot.launcher.setMotorVelocity(GGRobotConstants.Launcher.FAR_TARGET_VELOCITY, GGRobotConstants.Launcher.FAR_TARGET_VELOCITY);
//                    blueFarState = BlueFarState.SHOOT_SEQUENCE3;
//                }
//                break;
//            case SHOOT_SEQUENCE3:
//                robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE, 0.25, 0.0);
//                if (!robot.feeder.isBusy()) {
//                    if (shotsFired < 3) {
//                        LaunchIndexer.FeederSide side = getSideForShot(shotsFired, detectedMotif);
//                        if (robot.launchSequence(true, side, GGRobotConstants.LauncherDistance.FAR)) {
//                            shotsFired++;
//                        }
//                    } else {
//
//                        // All 3 shots are fired. Turn off the launcher and move to the park state.
//                        robot.launcher.setMotorVelocity(0, 0);
//                        blueFarState = BlueFarState.PARK;
//                    }
//                }
//                break;

            case PARK:
                // Drive to the final parking position.
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), GGRobotConstants.Waypoints.BLUE_FAR_PARK, 0.5, 0.25)) {
                    // Once parking is complete, the entire autonomous routine is done.
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }
        telemetry.addData("Path State", blueFarState);
    }



    /**
     * A helper method to determine which side to shoot from based on the shot number and motif.
     * @param shotNumber The shot we are on (0, 1, or 2).
     * @param motif The detected vision pattern.
     * @return The FeederSide to use for the shot.
     */
    private LaunchIndexer.FeederSide getSideForShot(int shotNumber, VisionUtil.MotifPattern motif) {
        // This logic determines the shot order based on the pre-load and vision result.
        // Assuming Green is pre-loaded on the RIGHT, Purple on the LEFT.
        switch (motif) {
            case GPP21: // Green, Purple, Purple
                if (shotNumber == 0) return LaunchIndexer.FeederSide.RIGHT;
                else return LaunchIndexer.FeederSide.LEFT;
            case PGP22: // Purple, Green, Purple
                if (shotNumber == 1) return LaunchIndexer.FeederSide.RIGHT;
                else return LaunchIndexer.FeederSide.LEFT;
            case PPG23: // Purple, Purple, Green
            case UNKNOWN: // Default to a safe order if vision fails
            default:
                if (shotNumber == 2) return LaunchIndexer.FeederSide.RIGHT;
                else return LaunchIndexer.FeederSide.LEFT;
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
