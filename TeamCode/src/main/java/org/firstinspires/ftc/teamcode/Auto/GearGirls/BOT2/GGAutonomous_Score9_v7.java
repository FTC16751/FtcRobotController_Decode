package org.firstinspires.ftc.teamcode.Auto.GearGirls.BOT2;

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
 * GearGirls Autonomous - Score 9 with Simplified ShotSequenceController
 *
 * @version 7.1 - Added purge mode selection
 */
@Autonomous(name="GG AUTO: Bot 2 (RUN ME)", group="GGBot", preselectTeleOp = "Gear Girls Bot 2 (RUN ME)")
public class GGAutonomous_Score9_v7 extends OpMode {

    private GGRobot2 robot;

    //================================================================================
    // DRIVER SELECTIONS
    //================================================================================
    private CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    private GGRobotConstants.Location location = GGRobotConstants.Location.CLOSE;
    private VisionUtil.MotifPattern detectedMotif = VisionUtil.MotifPattern.UNKNOWN;
    private int selectedCycles = 2;
    private boolean usePurgeMode = true;  // NEW: Purge mode toggle

    //================================================================================
    // STATE MACHINE
    //================================================================================
    private enum PathState {
        DRIVE_TO_SCORE,
        SHOOT_PRELOAD,

        // Cycle 1
        ALIGN_SPIKE_A,
        COLLECT_A_BALL3,
        DRIVE_TO_SCORE_2,
        SHOOT_CYCLE1,

        // Cycle 2
        ALIGN_SPIKE_B,
        COLLECT_B_BALL3,
        DRIVE_TO_SCORE_3,
        SHOOT_CYCLE2,

        PARK
    }
    private PathState currentState = PathState.DRIVE_TO_SCORE;
    private boolean pathComplete = false;

    //================================================================================
    // WAYPOINTS
    //================================================================================
    private String pathName;
    private double launcherVelocity;
    private Pose2D scorePose;
    private Pose2D parkPose;
    private Pose2D spikeA_align;
    private Pose2D spikeA_ball3;
    private Pose2D spikeB_align;
    private Pose2D spikeB_ball3;

    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot2(hardwareMap, telemetry);
        robot.vision.setPipeline(0);
        telemetry.addData(">", "Robot Initialized");
    }

    @Override
    public void init_loop() {
        robot.vision.setMotifDetectionMode();
        robot.vision.update();
        robot.update();

        // Driver selections
        if (gamepad1.x) alliance = CommonConstants.Alliance.BLUE;
        if (gamepad1.b) alliance = CommonConstants.Alliance.RED;
        if (gamepad1.y) location = GGRobotConstants.Location.CLOSE;
        if (gamepad1.a) location = GGRobotConstants.Location.FAR;
        if (gamepad1.dpad_left && selectedCycles > 1) selectedCycles--;
        if (gamepad1.dpad_right && selectedCycles < 3) selectedCycles++;

        // NEW: Purge mode toggle with bumpers
        if (gamepad1.left_bumper) usePurgeMode = false;
        if (gamepad1.right_bumper) usePurgeMode = true;

        detectedMotif = robot.vision.getMotifPattern();

        // Telemetry
        telemetry.addLine("🎮 AUTONOMOUS SELECTION 🎮");
        telemetry.addLine("━━━━━━━━━━━━━━━━━");
        telemetry.addLine();

        telemetry.addLine(alliance == CommonConstants.Alliance.RED ?
                "🔴 ALLIANCE: RED 🔴" : "🔵 ALLIANCE: BLUE 🔵");
        telemetry.addData("   Change", "X (Blue) or B (Red)");
        telemetry.addLine();

        telemetry.addLine(location == GGRobotConstants.Location.CLOSE ?
                "🤏 LOCATION: CLOSE" : "🔭 LOCATION: FAR");
        telemetry.addData("   Change", "Y (Close) or A (Far)");
        telemetry.addLine();

        telemetry.addLine(String.format("🔄 CYCLES: %d", selectedCycles));
        telemetry.addData("   1 Cycle", "Preload only");
        telemetry.addData("   2 Cycles", "Preload + Spike Mark A");
        telemetry.addData("   3 Cycles", "Preload + Two Spike Marks");
        telemetry.addData("   Change", "DPad Left/Right");
        telemetry.addLine();

        // NEW: Shooting mode display
        if (usePurgeMode) {
            telemetry.addLine("⚡ MODE: PURGE (FAST) ⚡");
            telemetry.addData("   Pattern", "Both flippers → Rotate → Left");
            telemetry.addData("   Speed", "~0.5s faster per volley");
        } else {
            telemetry.addLine("🎯 MODE: COLOR MATCH (ACCURATE) 🎯");
            telemetry.addData("   Pattern", "Match detected motif");
            telemetry.addData("   Accuracy", "Fires correct sequence");
        }
        telemetry.addData("   Change", "LB (Color) or RB (Purge)");
        telemetry.addLine();

        telemetry.addLine("🎨 PRELOAD DETECTION");
        telemetry.addData("Detected Motif", detectedMotif);
        telemetry.addLine("────────────────────────────────────");

        String leftBall = robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.LEFT)
                ? getColorEmoji(robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.LEFT)) : "⚪";
        String centerBall = robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.CENTER)
                ? getColorEmoji(robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.CENTER)) : "⚪";
        String rightBall = robot.isIntakeSlotOccupied(IntakeSensorFusion002.IntakeSlot.RIGHT)
                ? getColorEmoji(robot.getIntakeSlotColor(IntakeSensorFusion002.IntakeSlot.RIGHT)) : "⚪";

        telemetry.addLine(String.format("   [%s] [%s] [%s]", leftBall, centerBall, rightBall));
        telemetry.addLine("    L    C    R");

        // Note about purge mode ignoring colors
        if (usePurgeMode) {
            telemetry.addData("   Note", "Purge mode ignores colors");
        }

        telemetry.addLine();
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("✅ READY TO START - Press Play! ✅");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.update();
    }

    @Override
    public void start() {
        // Set odometry starting position
        if (location == GGRobotConstants.Location.CLOSE) {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_CLOSE :
                    GGRobotConstants.Waypoints.START_BLUE_CLOSE);
        } else {
            robot.drive.pinpoint.setPosition((alliance == CommonConstants.Alliance.RED) ?
                    GGRobotConstants.Waypoints.START_RED_FAR :
                    GGRobotConstants.Waypoints.START_BLUE_FAR);
        }

        robot.vision.setTargetingAlliance(alliance);
        setPathWaypoints(alliance, location);

        if (location == GGRobotConstants.Location.CLOSE) {
            launcherVelocity = GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
            robot.shotSequenceV2.setLauncherReadyVelocity(GGRobotConstants.Launcher.CLOSE_MIN_VELOCITY);

        } else {
            launcherVelocity = GGRobotConstants.Launcher.FAR_TARGET_VELOCITY;
            robot.shotSequenceV2.setLauncherReadyVelocity(GGRobotConstants.Launcher.FAR_MIN_VELOCITY);

        }

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

    private void runPath() {
        telemetry.addData("Path", pathName);
        telemetry.addData("State", currentState);
        telemetry.addData("Shoot Mode", usePurgeMode ? "PURGE" : "COLOR MATCH");

        switch (currentState) {

            // PRELOAD
            case DRIVE_TO_SCORE:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.75, 0.0)) {
                    currentState = PathState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.7, 0.0);

                if (runShootSequence(detectedMotif)) {
                    if (selectedCycles >= 2) {
                        currentState = PathState.ALIGN_SPIKE_A;
                    } else {
                        robot.launcher.setMotorVelocity(0, 0);
                        currentState = PathState.PARK;
                    }
                }
                break;

            // CYCLE 1
            case ALIGN_SPIKE_A:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeA_align, 0.75, 0.0)) {
                    currentState = PathState.COLLECT_A_BALL3;
                }
                break;

            case COLLECT_A_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeA_ball3, 0.7, 0.0)) {
                    currentState = PathState.DRIVE_TO_SCORE_2;
                }
                break;

            case DRIVE_TO_SCORE_2:
                robot.launcher.setMotorVelocity(launcherVelocity, launcherVelocity);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.75, 0.0)) {
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

            // CYCLE 2
            case ALIGN_SPIKE_B:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeB_align, 0.75, 0.0)) {
                    currentState = PathState.COLLECT_B_BALL3;
                }
                break;

            case COLLECT_B_BALL3:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        spikeB_ball3, 0.7, 0.0)) {
                    currentState = PathState.DRIVE_TO_SCORE_3;
                }
                break;

            case DRIVE_TO_SCORE_3:
                robot.launcher.setMotorVelocity(launcherVelocity, launcherVelocity);
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        scorePose, 0.77, 0.0)) {
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

            // PARK
            case PARK:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(),
                        parkPose, 1.0, 0.25)) {
                    pathComplete = true;
                }
                break;
        }
    }

    //================================================================================
    // WAYPOINT SETUP
    //================================================================================

    private void setPathWaypoints(CommonConstants.Alliance alliance, GGRobotConstants.Location location) {
        if (alliance == CommonConstants.Alliance.RED && location == GGRobotConstants.Location.CLOSE) {
            pathName = "Red Close";
            launcherVelocity = GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
            scorePose = GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_TO_SCORE;
            parkPose = GGRobotConstants.Waypoints.RED_CLOSE_PARK;
            spikeA_align = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_ALIGN;
            spikeA_ball3 = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK1_BALL3;
            spikeB_align = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_ALIGN;
            spikeB_ball3 = GGRobotConstants.Waypoints.RED_CLOSE_SPIKEMARK2_BALL3_COLLECT;

        } else if (alliance == CommonConstants.Alliance.RED && location == GGRobotConstants.Location.FAR) {
            pathName = "Red Far";
            launcherVelocity = GGRobotConstants.Launcher.FAR_TARGET_VELOCITY;
            scorePose = GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE;
            parkPose = GGRobotConstants.Waypoints.RED_FAR_PARK;
            spikeA_align = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_ALIGN;
            spikeA_ball3 = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK3_BALL3;
            spikeB_align = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_ALIGN;
            spikeB_ball3 = GGRobotConstants.Waypoints.RED_FAR_SPIKEMARK2_BALL3;

        } else if (alliance == CommonConstants.Alliance.BLUE && location == GGRobotConstants.Location.CLOSE) {
            pathName = "Blue Close";
            launcherVelocity = GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
            scorePose = GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_TO_SCORE;
            parkPose = GGRobotConstants.Waypoints.BLUE_CLOSE_PARK;
            spikeA_align = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_ALIGN;
            spikeA_ball3 = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK1_BALL3;
            spikeB_align = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_ALIGN;
            spikeB_ball3 = GGRobotConstants.Waypoints.BLUE_CLOSE_SPIKEMARK2_BALL3_COLLECT;

        } else {
            // Blue Far
            pathName = "Blue Far";
            launcherVelocity = GGRobotConstants.Launcher.FAR_TARGET_VELOCITY;
            scorePose = GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE;
            parkPose = GGRobotConstants.Waypoints.BLUE_FAR_PARK;
            spikeA_align = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_ALIGN;
            spikeA_ball3 = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK3_BALL3;
            spikeB_align = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_ALIGN;
            spikeB_ball3 = GGRobotConstants.Waypoints.BLUE_FAR_SPIKEMARK2_BALL3;
        }
    }

    //================================================================================
    // SHOT SEQUENCE
    //================================================================================

    /**
     * Simplified shot sequence - handles both color match and purge modes
     */
    private boolean runShootSequence(VisionUtil.MotifPattern motif) {
        // Check if done FIRST
        if (robot.shotSequenceV2.isDone()) {
            robot.shotSequenceV2.reset();
            return true;
        }

        // Start sequence if not already started (IDLE state)
        if (!robot.shotSequenceV2.isBusy() && !robot.shotSequenceV2.isDone()) {
            if (usePurgeMode) {
                // Use purge mode - ignores motif, fires all balls quickly
                robot.shotSequenceV2.purge();
            } else {
                // Use color matching mode
                robot.shotSequenceV2.start(motifToShortString(motif));
            }
        }

        // Show telemetry
        robot.shotSequenceV2.addTelemetry(telemetry);

        return false;
    }

    /** Convert MotifPattern to 3-letter string */
    private String motifToShortString(VisionUtil.MotifPattern motif) {
        if (motif == null) return "PPG";
        switch (motif) {
            case GPP21: return "GPP";
            case PGP22: return "PGP";
            case PPG23: return "PPG";
            default: return "PPG";
        }
    }

    /** Get emoji for ball color */
    private String getColorEmoji(IntakeSensorFusion002.ArtifactColor color) {
        switch (color) {
            case PURPLE: return "🟣";
            case GREEN: return "🟢";
            case UNKNOWN: return "❓";
            default: return "⚪";
        }
    }
}