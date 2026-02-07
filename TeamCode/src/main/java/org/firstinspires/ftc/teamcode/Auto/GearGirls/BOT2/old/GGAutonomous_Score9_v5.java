package org.firstinspires.ftc.teamcode.Auto.GearGirls.BOT2.old;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot2;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeSensorFusion002;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;

/**
 * GearGirls Autonomous - Score 9 with ShotSequenceController
 *
 * All four paths (Red Close, Red Far, Blue Close, Blue Far) share one state
 * machine and one runPath() method. The only thing that changes between paths
 * is the waypoints, which get assigned in setPathWaypoints() during start().
 *
 * @author GearGirls Team
 * @version 5.1 - Flat variables, no PathConfig class
 */
@Autonomous(name="GG AUTO: Score 9 (Bot2)", group="GGBot", preselectTeleOp = "GearGirlsBot2_test")
public class GGAutonomous_Score9_v5 extends OpMode {

    // --- Subsystems ---
    private GGRobot2 robot;

    //================================================================================
    // DRIVER SELECTIONS (set during init_loop before the match starts)
    //================================================================================
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    private GGRobotConstants.Location location = GGRobotConstants.Location.CLOSE;
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;
    private int selectedCycles = 2; // 1=Preload only, 2=+Spike A, 3=+Spike A & B
    private boolean blindFireMode = false;

    //================================================================================
    // THE ONE STATE MACHINE (used for ALL paths)
    //================================================================================
    private enum PathState {
        DRIVE_TO_SCORE,         // 1. Drive to the scoring position
        SHOOT_PRELOAD,          // 2. Fire the 3 preloaded balls

        // --- Cycle 1: collect Spike Mark A ---
        ALIGN_SPIKE_A,          // 3. Drive to the align waypoint near spike mark A
        COLLECT_A_BALL1,        // 4. Drive onto ball 1
        COLLECT_A_BALL2,        // 5. Drive onto ball 2
        COLLECT_A_BALL3,        // 6. Drive onto ball 3
        DRIVE_TO_SCORE_2,       // 7. Drive back to scoring position
        SHOOT_CYCLE1,           // 8. Fire those 3 balls

        // --- Cycle 2: collect Spike Mark B ---
        ALIGN_SPIKE_B,          // 9.  Drive to the align waypoint near spike mark B
        COLLECT_B_BALL1,        // 10. Drive onto ball 1
        COLLECT_B_BALL2,        // 11. Drive onto ball 2
        COLLECT_B_BALL3,        // 12. Drive onto ball 3
        DRIVE_TO_SCORE_3,       // 13. Drive back to scoring position
        SHOOT_CYCLE2,           // 14. Fire those 3 balls

        PARK                    // 15. Drive to the park zone
    }
    private PathState currentState = PathState.DRIVE_TO_SCORE;

    // Once PARK finishes, this flips to true and loop() stops the opmode.
    private boolean pathComplete = false;

    //================================================================================
    // WAYPOINT VARIABLES — set once in start(), then read by runPath() the whole match.
    // These are the only thing that differs between Red Close, Blue Far, etc.
    //================================================================================
    private String   pathName;              // Just for telemetry (e.g. "Red Close")
    private double   launcherVelocity;      // CLOSE or FAR launcher speed

    private Pose2D   scorePose;             // Where to drive to shoot
    private Pose2D   parkPose;              // Where to end up at the end

    // Spike Mark A — the first spike mark collected after the preload shot
    private Pose2D   spikeA_align;
    private Pose2D   spikeA_ball1;
    private Pose2D   spikeA_ball2;
    private Pose2D   spikeA_ball3;

    // Spike Mark B — the second spike mark, only used if cycles >= 3
    private Pose2D   spikeB_align;
    private Pose2D   spikeB_ball1;
    private Pose2D   spikeB_ball2;
    private Pose2D   spikeB_ball3;

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

        if (gamepad1.dpadLeftWasPressed()  && selectedCycles > 1) { selectedCycles--; }
        if (gamepad1.dpadRightWasPressed() && selectedCycles < 3) { selectedCycles++; }

        if (gamepad1.left_bumper)  { blindFireMode = false; }
        if (gamepad1.right_bumper) { blindFireMode = true;  }

        detectedMotif = robot.vision.getMotifPattern();

        // --- Telemetry ---
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("     🎮 AUTONOMOUS SELECTION 🎮");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine();

// Current selections - BIG and CLEAR
        if (alliance == CommonConstants.Alliance.RED) {
            telemetry.addLine("🔴 ALLIANCE: RED 🔴");

        } else {
            telemetry.addLine("🔵 ALLIANCE: BLUE 🔵");
        }
        telemetry.addData("   Change", "Press X (Blue) or B (Red)");
        telemetry.addLine();

        if (location == GGRobotConstants.Location.CLOSE) {
            telemetry.addLine("📍 LOCATION: CLOSE (GOAL SIDE");
        } else {
            telemetry.addLine("📍 LOCATION: FAR");
        }
        telemetry.addData("   Change", "Press Y (Close) or A (Far)");
        telemetry.addLine();

        telemetry.addLine(String.format("🔄 CYCLES: %d", selectedCycles));
        telemetry.addData("   1 Cycle", "Preload only");
        telemetry.addData("   2 Cycles", "Preload + Spike Mark A");
        telemetry.addData("   3 Cycles", "Preload + Two Spike Marks");
        telemetry.addData("   Change", "DPad Left/Right");
        telemetry.addLine();

        if (blindFireMode) {
            telemetry.addLine("🎯 MODE: BLIND FIRE (Fast)");
            telemetry.addData("   Pattern", "L+R together, spin→L");
        } else {
            telemetry.addLine("🎯 MODE: SENSOR (Accurate)");
            telemetry.addData("   Pattern", "Match detected colors");
        }
        telemetry.addData("   Change", "LB (Sensor) or RB (Blind)");

        telemetry.addLine();
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("✅ READY TO START - Press Play! ✅");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine();

        // Color sensor data - visual representation
        telemetry.addLine("🎨 PRELOAD DETECTION");
        telemetry.addLine("────────────────────────────────────");
        telemetry.addData("Detected Motif", detectedMotif);

        // Visual ball representation
        String leftBall = robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.LEFT)
                ? getColorEmoji(robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT))
                : "⚪"; // Empty
        String centerBall = robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.CENTER)
                ? getColorEmoji(robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.CENTER))
                : "⚪"; // Empty
        String rightBall = robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.RIGHT)
                ? getColorEmoji(robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT))
                : "⚪"; // Empty

        telemetry.addLine(String.format("   [%s] [%s] [%s]", leftBall, centerBall, rightBall));
        telemetry.addLine("    L    C    R");

        // Technical data - collapsed and secondary
        telemetry.addLine("📊 ROBOT STATUS");
        telemetry.addLine("────────────────────────────────────");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°",
                robot.drive.pinpoint.getPosition().getX(DistanceUnit.INCH),
                robot.drive.pinpoint.getPosition().getY(DistanceUnit.INCH),
                robot.drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addLine();

        telemetry.update();
    }

    @Override
    public void start() {
        // --- 1. Set odometry starting position ---
        if (location == GGRobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_CLOSE : GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_FAR : GGRobotConstants.Waypoints.START_BLUE_FAR);
        }

        // --- 2. Tell the vision system which alliance we are ---
        robot.vision.setTargetingAlliance(alliance);

        // --- 3. Assign all the waypoints for whichever path we picked ---
        setPathWaypoints(alliance, location);

        // --- 4. Spin up the launcher ---
        robot.launcher.setMotorVelocity(launcherVelocity, launcherVelocity);
    }

    //================================================================================
    // MAIN LOOP
    //================================================================================

    @Override
    public void loop() {
        robot.update();
        robot.intake.setIntakeMotorPower(-1);

        if (!pathComplete) {
            runPath();
        } else {
            robot.stopAll();
            requestOpModeStop();
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
    // THE UNIFIED PATH
    //================================================================================
    // Every driveTo just uses the waypoint variables directly.
    // The state transitions are the same no matter which path you picked.
    //================================================================================

    private void runPath() {
        telemetry.addData("Current Path", pathName);
        telemetry.addData("State",        currentState);

        switch (currentState) {

            // ------------------------------------------------------------------
            // PRELOAD SHOT
            // ------------------------------------------------------------------
            case DRIVE_TO_SCORE:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.7, 0.0)) {
                    currentState = PathState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 2) {
                        currentState = PathState.ALIGN_SPIKE_A;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        currentState = PathState.PARK;
                    }
                }
                break;

            // ------------------------------------------------------------------
            // CYCLE 1 — collect Spike Mark A
            // ------------------------------------------------------------------
            case ALIGN_SPIKE_A:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeA_align, 0.7, 0.0)) {
                    currentState = PathState.COLLECT_A_BALL3;
                }
                break;

            case COLLECT_A_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeA_ball1, 0.5, 0.20)) {
                    currentState = PathState.COLLECT_A_BALL2;
                }
                break;

            case COLLECT_A_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeA_ball2, 0.5, 0.0)) {
                    currentState = PathState.COLLECT_A_BALL3;
                }
                break;

            case COLLECT_A_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeA_ball3, 0.5, 0.0)) {
                    currentState = PathState.DRIVE_TO_SCORE_2;
                }
                break;

            case DRIVE_TO_SCORE_2:
                robot.launcher.setMotorVelocity(    launcherVelocity, launcherVelocity);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.7, 0.0)) {
                    currentState = PathState.SHOOT_CYCLE1;
                }
                break;

            case SHOOT_CYCLE1:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 3) {
                        currentState = PathState.ALIGN_SPIKE_B;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        currentState = PathState.PARK;
                    }
                }
                break;

            // ------------------------------------------------------------------
            // CYCLE 2 — collect Spike Mark B
            // ------------------------------------------------------------------
            case ALIGN_SPIKE_B:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeB_align, 0.7, 0.0)) {
                    currentState = PathState.COLLECT_B_BALL3;
                }
                break;

            case COLLECT_B_BALL1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeB_ball1, 0.5, 0.1)) {
                    currentState = PathState.COLLECT_B_BALL2;
                }
                break;

            case COLLECT_B_BALL2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeB_ball2, 0.5, 0.0)) {
                    currentState = PathState.COLLECT_B_BALL3;
                }
                break;

            case COLLECT_B_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeB_ball3, 0.5, 0.0)) {
                    currentState = PathState.DRIVE_TO_SCORE_3;
                }
                break;

            case DRIVE_TO_SCORE_3:
                robot.launcher.setMotorVelocity(launcherVelocity, launcherVelocity);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.7, 0.0)) {
                    currentState = PathState.SHOOT_CYCLE2;
                }
                break;

            case SHOOT_CYCLE2:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.5, 0.0);

                if (runShootSequence(detectedMotif)) {
                    robot.launcher.setMotorVelocity(0, 0);
                    currentState = PathState.PARK;
                }
                break;

            // ------------------------------------------------------------------
            // PARK
            // ------------------------------------------------------------------
            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        parkPose, 0.8, 0.25)) {
                    pathComplete = true;
                }
                break;
        }
    }

    //================================================================================
    // SET PATH WAYPOINTS — the only place the 4 paths are defined
    //================================================================================
    // Just assigns the waypoint variables above. No driving, no shooting, no logic.
    // If you need to tweak where the robot goes for one path, change it here.
    //================================================================================

    private void setPathWaypoints(CommonConstants.Alliance alliance, GGRobotConstants.Location location) {

        if (alliance == CommonConstants.Alliance.RED && location == GGRobotConstants.Location.CLOSE) {
            pathName            = "Red Close";
            launcherVelocity    = GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
            scorePose           = GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE;
            parkPose            = GGRobotConstants.Waypoints.RED_CLOSE_PARK;

            spikeA_align        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_ALIGN;
            spikeA_ball1        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL1;
            spikeA_ball2        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL2;
            spikeA_ball3        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL3;

            spikeB_align        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_ALIGN;
            spikeB_ball1        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL1_COLLECT;
            spikeB_ball2        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL2_COLLECT;
            spikeB_ball3        = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL3_COLLECT;

        } else if (alliance == CommonConstants.Alliance.RED && location == GGRobotConstants.Location.FAR) {
            pathName            = "Red Far";
            launcherVelocity    = GGRobotConstants.Launcher.FAR_TARGET_VELOCITY;
            scorePose           = GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE;
            parkPose            = GGRobotConstants.Waypoints.RED_FAR_PARK;

            // Far paths collect spike mark 3 first, then spike mark 2
            spikeA_align        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_ALIGN;
            spikeA_ball1        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL1;
            spikeA_ball2        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL2;
            spikeA_ball3        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL3;

            spikeB_align        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_ALIGN;
            spikeB_ball1        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL1;
            spikeB_ball2        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL2;
            spikeB_ball3        = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL3;

        } else if (alliance == CommonConstants.Alliance.BLUE && location == GGRobotConstants.Location.CLOSE) {
            pathName            = "Blue Close";
            launcherVelocity    = GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
            scorePose           = GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE;
            parkPose            = GGRobotConstants.Waypoints.BLUE_CLOSE_PARK;

            spikeA_align        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_ALIGN;
            spikeA_ball1        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL1;
            spikeA_ball2        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL2;
            spikeA_ball3        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL3;

            spikeB_align        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_ALIGN;
            spikeB_ball1        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL1_COLLECT;
            spikeB_ball2        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL2_COLLECT;
            spikeB_ball3        = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL3_COLLECT;

        } else {
            // Blue Far (default)
            pathName            = "Blue Far";
            launcherVelocity    = GGRobotConstants.Launcher.FAR_TARGET_VELOCITY;
            scorePose           = GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE;
            parkPose            = GGRobotConstants.Waypoints.BLUE_FAR_PARK;

            // Far paths collect spike mark 3 first, then spike mark 2
            spikeA_align        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_ALIGN;
            spikeA_ball1        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL1;
            spikeA_ball2        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL2;
            spikeA_ball3        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL3;

            spikeB_align        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_ALIGN;
            spikeB_ball1        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL1;
            spikeB_ball2        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL2;
            spikeB_ball3        = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL3;
        }
    }

    //================================================================================
    // SHOT SEQUENCE CONTROLLER INTEGRATION (unchanged from v4)
    //================================================================================

    /**
     * Executes a 3-shot sequence using the ShotSequenceController.
     * Returns true once all 3 shots have been fired (or an error occurred).
     */
    private boolean runShootSequence(VisionUtil.MotifPattern motif) {
        if (robot.shotSequence.isError()) {
            robot.triggerBothFlippers();
            telemetry.addLine("! Shot sequence error, continuing...");
            robot.shotSequence.reset();
            return true;
        }

        if (robot.shotSequence.isDone()) {
            robot.shotSequence.reset();
            return true;
        }

        if (!robot.shotSequence.isBusy()) {
            robot.shotSequence.setLauncherReadyVelocity(launcherVelocity);
            robot.shotSequence.setRequireLauncherReady(true);
            robot.shotSequence.setBlindFireMode(blindFireMode); //
            robot.shotSequence.start(motifToShortString(motif));
        }

        robot.shotSequence.addTelemetry(telemetry);
        return false;
    }

    /** Converts MotifPattern enum to 3-letter shorthand (GPP/PGP/PPG). */
    private String motifToShortString(VisionUtil.MotifPattern motif) {
        if (motif == null) return "PPG";
        switch (motif) {
            case GPP21: return "GPP";
            case PGP22: return "PGP";
            case PPG23: return "PPG";
            default:    return "PPG";
        }
    }

    /**
     * Get emoji representation of ball color for visual feedback
     */
    private String getColorEmoji(IntakeSensorFusion002.ArtifactColor color) {
        switch (color) {
            case PURPLE:
                return "🟣"; // Purple circle
            case GREEN:
                return "🟢"; // Green circle
            case UNKNOWN:
                return "❓"; // Question mark
            default:
                return "⚪"; // White circle (empty)
        }
    }
}