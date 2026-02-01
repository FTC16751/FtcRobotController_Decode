package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import static org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil.RED_GOAL_X_COORDINATE_METERS;
import static org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil.RED_GOAL_Y_COORDINATE_METERS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.utilities.Common.InterpolatingLookupTable;
import org.firstinspires.ftc.teamcode.utilities.Common.LedUtil;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants.LauncherSystemState;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants.LauncherTargetingMode;

/**
 * The Robot class is the central hub that orchestrates all of the robot's subsystems.
 * It owns all the hardware and utility classes, providing a clean and high-level
 * interface for OpModes (TeleOp and Autonomous) to use. This promotes code reuse and
 * simplifies the OpMode logic significantly.
 *
 * VERSION 2.1 - Updated to use:
 * - IntakeUtilV2 (2-motor intake)
 * - LaunchFlippers (servo-based launch mechanism)
 * - Spinner (rotation mechanism)
 * - RPM-based velocity control
 */
public class GGRobot2 {

    // PUBLIC SUBSYSTEMS
    public final DriveUtil2026b drive;
    public final LauncherMotors launcher;
    public final LaunchFlippers flippers;     // Updated from LaunchIndexer
    public final IntakeUtilV2 intake;         // Updated from IntakeUtil
    public final Spinner_FORTEST spinner;             // Added
    public final IntakeSensorFusion001 intakeSensors; // Added - Sensor fusion system
    public final Telemetry telemetry;
    public final VisionUtil vision;
    public final LedUtil led;

    // PRIVATE STATE AND TIMERS FOR ROBOT-LEVEL ACTIONS
    private enum LaunchState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        COOLDOWN
    }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime launchCycleTimer = new ElapsedTime();

    private enum DriveAndIntakeState {
        IDLE,
        STARTING,
        INTAKING_FIRST_ARTIFACT,  // Intake with diverter to the RIGHT
        SWITCHING_DIVERTER,       // Move diverter to the LEFT
        INTAKING_REMAINING,       // Intake with diverter to the LEFT
        DRIVING_FORWARD           // The slow forward drive happens in parallel
    }
    private DriveAndIntakeState driveAndIntakeState = DriveAndIntakeState.IDLE;
    private ElapsedTime intakeSubActionTimer = new ElapsedTime();

    // === LOOKUP TABLE ===
    // NOTE: These values may need to be updated for the new 72mm wheels
    // Original values were for 96mm wheels
    private InterpolatingLookupTable flywheelTable;
    private double lastKnownGoodVelocity = 0.0;

    /**
     * The constructor for the Robot class.
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry The telemetry object from the OpMode.
     */
    public GGRobot2(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Get the robot configuration from the config file
        RobotConfig ggConfig = RobotConfig.createDefaultGearGirlsConfig();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, ggConfig);
        launcher = new LauncherMotors(hardwareMap);
        flippers = new LaunchFlippers(hardwareMap);    // Updated
        intake = new IntakeUtilV2(hardwareMap);        // Updated
        spinner = new Spinner_FORTEST(hardwareMap);            // Added
        intakeSensors = new IntakeSensorFusion001(hardwareMap, telemetry); // Added - Initialize sensor fusion
        vision = new VisionUtil(hardwareMap, telemetry);
        led = new LedUtil(hardwareMap, "led_servo");

        // Flywheel lookup table
        // TODO: These values need to be re-tuned for the 72mm wheels!
        // Current values are from the 96mm wheel configuration
        // Expected: multiply all velocities by approximately 1.33x or more
        flywheelTable = new InterpolatingLookupTable();
        flywheelTable.add(30.0, 960.0*1.1);
        flywheelTable.add(40.0, 1010.0*1.05);
        flywheelTable.add(50.0, 1070.0*1.05);
        flywheelTable.add(60.0, 1130.0);
        flywheelTable.add(70.0, 1200.0);
        flywheelTable.add(80.0, 1280.0);
        flywheelTable.add(100.0, 1390.0);
        flywheelTable.add(120.0, 1420.0);
        flywheelTable.add(130.0, 1470.0);
        flywheelTable.add(140.0, 1500.0);

        //drive.resetHeading();
        //stopAll();
    }

    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     */
    public void update() {
        if (led != null) updateLedStatus();
        if (drive != null) drive.update();
        if (flippers != null) flippers.update();    // CRITICAL: Must be called every loop
        if (spinner != null) spinner.update();       // Added
        if (intakeSensors != null) intakeSensors.update(); // CRITICAL: Update sensor fusion state machines
        if (vision != null) {
            vision.update();
            vision.updateRobotOrientation(drive.getHeading());
        }
    }

    /**
     * Stops all motors and mechanisms on the robot.
     */
    public void stopAll() {
        drive.stopRobot();
        launcher.setMotorVelocityRPM(0, 0);          // Updated to use RPM
        intake.stop();                                // Updated method name
        flippers.emergencyStop();                     // Updated from feeder
        vision.stop();
        led.setColor(LedUtil.Color.OFF);
    }

    /**
     * A consolidated method for displaying common robot telemetry.
     */
    public void addTelemetry() {
        telemetry.addData("--- Robot Status ---", "");
        telemetry.addData("Drive Busy", drive.isBusy());
        telemetry.addData("--- Launcher ---", "");
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Launcher Velocity RPM", launcher.getLeftMotorVelocityRPM());  // Updated to RPM
        telemetry.addData("Flippers Busy", flippers.isBusy());                           // Updated from feeder
        telemetry.addData("Spinner State", spinner.getSpinnerState());                   // Added
        telemetry.addData("Pinpoint Positions (x, y, heading): ",
                drive.pinpoint.getPosition().getX(DistanceUnit.INCH) + ", " +
                        drive.pinpoint.getPosition().getY(DistanceUnit.INCH) + ", " +
                        drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
    }

    /**
     * Configures the vision system for TeleOp based on alliance color.
     * @param alliance The alliance color (RED or BLUE)
     */
    public void configureVisionForTeleOp(CommonConstants.Alliance alliance) {
        if (vision != null) {
            vision.setTargetingAlliance(alliance);
            telemetry.addData("Vision", "Configured for %s Alliance", alliance);
        }
    }

    /**
     * Updates LED color based on vision targeting status.
     */
    private void updateLedStatus() {
        if (!vision.isTargetVisible()) {
            led.setColor(LedUtil.Color.OFF);
            return;
        }

        double headingError = vision.getTargetAngleX();
        final double AIMING_TOLERANCE_DEG = 2.0;
        if (Math.abs(headingError) <= AIMING_TOLERANCE_DEG) {
            led.setColor(LedUtil.Color.GREEN);
        } else if (headingError > AIMING_TOLERANCE_DEG) {
            led.setColor(LedUtil.Color.YELLOW);
        } else if (headingError < -AIMING_TOLERANCE_DEG) {
            led.setColor(LedUtil.Color.BLUE);
        }
    }

    /**
     * Sets the LED to blink with a specified color.
     * @param color The color value to blink
     */
    public void setBlinkingColor(double color) {
        final int BLINK_INTERVAL_MS = 250;
        boolean isLedOn = (System.currentTimeMillis() / BLINK_INTERVAL_MS) % 2 == 0;
        led.setColor(isLedOn ? color : LedUtil.Color.OFF);
    }

    /**
     * Resets the robot's odometry position based on the vision system's
     * field-centric pose.
     */
    public void resetOdometryToVision() {
        Pose3D visionPose3D = vision.getRobotPoseFieldSpace();
        if (visionPose3D != null) {
            double x_in = visionPose3D.getPosition().x * 39.3701;
            double y_in = visionPose3D.getPosition().y * 39.3701;
            double heading_deg = Math.toDegrees(visionPose3D.getOrientation().getYaw());

            Pose2D newPose = new Pose2D(DistanceUnit.INCH, x_in, y_in, AngleUnit.DEGREES, heading_deg);
            drive.pinpoint.setPosition(newPose);

            telemetry.addData("Odometry Reset", "X: %.1f, Y: %.1f, H: %.1f", x_in, y_in, heading_deg);
        } else {
            telemetry.addData("Odometry Reset", "FAILED - No vision pose");
        }
    }

    /**
     * Master launcher control method that handles both AUTO and PRESET targeting modes.
     *
     * @param systemState The desired state (IDLE or ACTIVE)
     * @param targetingMode The targeting mode (AUTO or PRESET)
     * @param presetDistance The preset distance if using PRESET mode
     * @return The final commanded velocity in RPM
     */
    public double updateLauncher(LauncherSystemState systemState,
                                 LauncherTargetingMode targetingMode,
                                 GGRobotConstants.LauncherDistance presetDistance) {
        double finalTargetVelocity = 0.0;

        if (systemState == LauncherSystemState.IDLE) {
            finalTargetVelocity = 0.0;
        } else if (systemState == LauncherSystemState.ACTIVE) {
            if (targetingMode == LauncherTargetingMode.AUTO) {
                finalTargetVelocity = updateAndGetTargetVelocity();
            } else {
                if (presetDistance == GGRobotConstants.LauncherDistance.CLOSE) {
                    finalTargetVelocity = GGRobotConstants.Launcher.CLOSE_TARGET_VELOCITY;
                } else {
                    finalTargetVelocity = GGRobotConstants.Launcher.FAR_TARGET_VELOCITY;
                }
            }
        }

        // Command the motors using RPM (updated)
        launcher.setMotorVelocity(finalTargetVelocity, finalTargetVelocity);

        return finalTargetVelocity;
    }

    /**
     * Calculates the target launcher velocity using a hierarchical "waterfall" of data sources.
     * Prefers Vision (Limelight), falls back to Odometry, and finally uses last known good value.
     *
     * @return The calculated target velocity in RPM
     */
    public double updateAndGetTargetVelocity() {
        final double METERS_TO_INCHES = 39.3701;
        String dataSource;
        double newVelocity;

        // --- The Waterfall Logic ---
        if (vision.isTargetVisible()) {
            // PRIMARY: Vision is available and is our most trusted source
            dataSource = "VISION";
            double distanceInches = vision.getDistanceToTagInches();
            newVelocity = getTargetVelocityForDistance(distanceInches);

            // Update fallback state with high-confidence value
            this.lastKnownGoodVelocity = newVelocity;
        }
        // Odometry fallback commented out - uncomment if needed
        // else if (drive.pinpoint.getLoopTime() > 0) {
        //     dataSource = "ODOMETRY";
        //     double distanceInches = getDistanceToGoal();
        //     newVelocity = getTargetVelocityForDistance(distanceInches);
        //     this.lastKnownGoodVelocity = newVelocity;
        // }
        else {
            // TERTIARY: Both vision and odometry failed, use last known good
            dataSource = "LAST KNOWN";
            newVelocity = this.lastKnownGoodVelocity;
        }

        telemetry.addData("Aiming Data Source", dataSource);
        return newVelocity;
    }

    /**
     * Checks if any of the robot's major subsystems are currently busy with an action.
     * @return true if the drive or launch sequence is active, false otherwise.
     */
    public boolean isBusy() {
        return drive.isBusy() || isLaunchSequenceBusy() || (driveAndIntakeState != DriveAndIntakeState.IDLE);
    }

    /**
     * Checks if the internal launch state machine is currently active.
     * @return true if the robot is in the middle of a launch sequence, false if IDLE.
     */
    public boolean isLaunchSequenceBusy() {
        return launchState != LaunchState.IDLE;
    }

    /**
     * Calculates the required flywheel velocity for a given distance using the lookup table.
     * NOTE: Lookup table values may need re-tuning for 72mm wheels!
     *
     * @param distanceInches The distance to the target
     * @return The calculated target velocity in RPM
     */
    public double getTargetVelocityForDistance(double distanceInches) {
        return flywheelTable.get(distanceInches);
    }

    /**
     * Calculates and returns the distance from the robot to the fixed goal.
     * @return The distance to the goal in INCHES
     */
    public double getDistanceToGoal() {
        Pose2D trgtPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45);
        Pose2D currPose = drive.pinpoint.getPosition();
        return drive.distanceTo(currPose, trgtPose, DistanceUnit.INCH);
    }

    /**
     * Calculates distance to goal using field-relative coordinates based on alliance.
     * @return The distance to the goal in INCHES
     */
    public double getDistanceToGoalFieldRelative() {
        CommonConstants.Alliance currentAlliance = SharedState.alliance;

        Pose2D trgtPose;
        if (currentAlliance == CommonConstants.Alliance.RED) {
            trgtPose = new Pose2D(DistanceUnit.METER,
                    GGRobotConstants.GoalLocation.RED_TAG24_X_M,
                    GGRobotConstants.GoalLocation.RED_TAG24_Y_M,
                    AngleUnit.DEGREES, 54);
        } else { // BLUE
            trgtPose = new Pose2D(DistanceUnit.METER,
                    GGRobotConstants.GoalLocation.BLUE_TAG20_X_M,
                    GGRobotConstants.GoalLocation.BLUE_TAG20_Y_M,
                    AngleUnit.DEGREES, 45);
        }

        Pose2D currPose = drive.pinpoint.getPosition();
        return drive.distanceTo(currPose, trgtPose, DistanceUnit.INCH);
    }

    /**
     * Calculates auto-aim turn power during TeleOp based on vision targeting.
     *
     * NOTE: This method may be inconsistent if odometry is not field-relative.
     *
     * @return The calculated turn power (-1.0 to 1.0) to send to the drivetrain
     */
    public double calculateAutoAimTurnPower() {
        telemetry.addLine("--- Auto-Aim Calculation ---");

        // Check if vision system can provide an aim point
        if (!vision.hasFieldPose()) {
            telemetry.addData("AutoAim", "OFF (No Field Pose)");
            return 0.0;
        }

        // Get Robot's Current Position from pinpoint odometry
        Pose2D robotPose = drive.pinpoint.getPosition();
        double robotX_in = robotPose.getX(DistanceUnit.INCH);
        double robotY_in = robotPose.getY(DistanceUnit.INCH);
        double robotHeading_deg = robotPose.getHeading(AngleUnit.DEGREES);

        telemetry.addData("Robot Field Pos (X, Y, H)", "%.1f in, %.1f in, %.1f deg",
                robotX_in, robotY_in, robotHeading_deg);

        // Get the current alliance
        CommonConstants.Alliance currentAlliance = SharedState.alliance;
        telemetry.addData("AutoAim Alliance", currentAlliance);

        // Determine Target Goal Coordinates based on alliance
        double goalX_m;
        double goalY_m;

        if (currentAlliance == CommonConstants.Alliance.RED) {
            goalX_m = GGRobotConstants.GoalLocation.RED_TAG24_X_M;
            goalY_m = GGRobotConstants.GoalLocation.RED_TAG24_Y_M;
            telemetry.addData("Target", "RED Goal (Tag 24)");
        } else { // BLUE
            goalX_m = GGRobotConstants.GoalLocation.BLUE_TAG20_X_M;
            goalY_m = GGRobotConstants.GoalLocation.BLUE_TAG20_Y_M;
            telemetry.addData("Target", "BLUE Goal (Tag 20)");
        }

        telemetry.addData("Target Field Pos (X, Y)", "%.1f in, %.1f in",
                goalX_m * 39.3701, goalY_m * 39.3701);

        // Get the heading error from the vision system
        double headingErrorDeg;
        if (currentAlliance == CommonConstants.Alliance.RED) {
            headingErrorDeg = vision.calculateHeadingErrorToRedGoalDegrees();
        } else { // BLUE
            headingErrorDeg = vision.calculateHeadingErrorToBlueGoalDegrees();
        }

        // Pass the heading error to DriveUtil for turn calculation
        double turnCmd = drive.calculateAutoAimTurn(headingErrorDeg);

        // Telemetry for debugging
        telemetry.addData("AutoAim", "ON | Error: %.1f deg | Cmd: %.2f", headingErrorDeg, turnCmd);
        telemetry.addData("FINAL Heading Error", "%.1f deg", headingErrorDeg);
        telemetry.addData("FINAL Turn Command", "%.2f", turnCmd);
        telemetry.addLine("--------------------------");

        return turnCmd;
    }

    // ========== SPINNER CONTROL METHODS ==========

    /**
     * Rotates the spinner to the left position.
     */
    public void rotateSpinnerLeft() {
        if (spinner != null) {
            spinner.rotateLeft();
        }
    }

    /**
     * Rotates the spinner to the right position.
     */
    public void rotateSpinnerRight() {
        if (spinner != null) {
            spinner.rotateRight();
        }
    }

    /**
     * Checks if the spinner is currently busy.
     * @return true if spinner is moving, false if idle
     */
    public boolean isSpinnerBusy() {
        return spinner != null && spinner.isSpinnerBusy();
    }

    // ========== FLIPPER CONTROL METHODS ==========

    /**
     * Triggers the left flipper to launch a game element.
     * The flipper will automatically retract after the hold time.
     */
    public void triggerLeftFlipper() {
        if (flippers != null) {
            flippers.trigger(LaunchFlippers.FlipperSide.LEFT);
        }
    }

    /**
     * Triggers the right flipper to launch a game element.
     * The flipper will automatically retract after the hold time.
     */
    public void triggerRightFlipper() {
        if (flippers != null) {
            flippers.trigger(LaunchFlippers.FlipperSide.RIGHT);
        }
    }

    /**
     * Triggers both flippers simultaneously.
     */
    public void triggerBothFlippers() {
        if (flippers != null) {
            flippers.trigger(LaunchFlippers.FlipperSide.BOTH);
        }
    }

    /**
     * Emergency stops all flippers and retracts them immediately.
     */
    public void emergencyStopFlippers() {
        if (flippers != null) {
            flippers.emergencyStop();
        }
    }

    /**
     * Checks if any flipper is currently busy with a flip cycle.
     * @return true if a flipper is operating, false otherwise
     */
    public boolean areFlippersBusy() {
        return flippers != null && flippers.isBusy();
    }
    // ========== INTAKE SENSOR FUSION METHODS ==========

    /**
     * Gets the determined color of the artifact in a specific intake slot.
     * @param slot The IntakeSlot to check (LEFT or RIGHT)
     * @return The ArtifactColor (PURPLE, GREEN, or UNKNOWN)
     */
    public IntakeSensorFusion001.ArtifactColor getIntakeSlotColor(IntakeSensorFusion001.IntakeSlot slot) {
        if (intakeSensors != null) {
            return intakeSensors.getColorOfSlot(slot);
        }
        return IntakeSensorFusion001.ArtifactColor.UNKNOWN;
    }

    /**
     * Checks if a specific intake slot is currently occupied by an artifact.
     * @param slot The IntakeSlot to check
     * @return true if an artifact is detected, false otherwise
     */
    public boolean isIntakeSlotOccupied(IntakeSensorFusion001.IntakeSlot slot) {
        if (intakeSensors != null) {
            return intakeSensors.isSlotOccupied(slot);
        }
        return false;
    }

    /**
     * Gets a list of all currently held artifacts.
     * @return A List of ArtifactColor representing the current inventory
     */
    public java.util.List<IntakeSensorFusion001.ArtifactColor> getIntakeInventory() {
        if (intakeSensors != null) {
            return intakeSensors.getInventory();
        }
        return new java.util.ArrayList<>();
    }

    /**
     * Gets the total count of artifacts currently in the intake.
     * @return The number of artifacts detected
     */
    public int getIntakeInventoryCount() {
        if (intakeSensors != null) {
            return intakeSensors.getInventory().size();
        }
        return 0;
    }
}