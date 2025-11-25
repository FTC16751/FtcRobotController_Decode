package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import static org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil.RED_TAG24_X_M;
import static org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil.RED_TAG24_Y_M;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;
import org.firstinspires.ftc.teamcode.utilities.Common.LedUtil;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.Common.InterpolatingLookupTable;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants.LauncherSystemState;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants.LauncherTargetingMode;

import java.util.List;
import java.util.ArrayList; // Add these imports

/**
 * The Robot class is the central hub that orchestrates all of the robot's subsystems.
 * It owns all the hardware and utility classes, providing a clean and high-level
 * interface for OpModes (TeleOp and Autonomous) to use. This promotes code reuse and
 * simplifies the OpMode logic significantly.
 */
public class GGRobot {

    // 1. PUBLIC SUBSYSTEMS
    public final DriveUtil2026b drive;
    public final LauncherMotors launcher;
    public final LaunchIndexer feeder;
    public final IntakeUtil intake;
    public final Telemetry telemetry;
    public final VisionUtil vision;
    public final IntakeSensorFusion intakeSensors;
    public final LedUtil led;

    // 2. PRIVATE STATE AND TIMERS FOR ROBOT-LEVEL ACTIONS
    private enum LaunchState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        COOLDOWN
    }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime launchCycleTimer = new ElapsedTime();
    // ---1. NEW STATE MACHINE FOR THE DRIVE_AND_INTAKE ACTION ---
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
    // === 1. DEFINE YOUR LOOKUP TABLE ===
    private InterpolatingLookupTable flywheelTable;
    private double lastKnownGoodVelocity = 0.0;

    /**
     * The constructor for the Robot class.
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry The telemetry object from the OpMode.
     */
    public GGRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        RobotConfig ggConfig = RobotConfig.createDefaultGearGirlsConfig();
        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, ggConfig);
        launcher = new LauncherMotors(hardwareMap);
        feeder = new LaunchIndexer(hardwareMap);
        intake = new IntakeUtil(hardwareMap);
        intakeSensors = new IntakeSensorFusion(hardwareMap, telemetry);
        vision = new VisionUtil(hardwareMap, telemetry);
        led = new LedUtil(hardwareMap, "led_servo");

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

        drive.resetHeading();
        //stopAll();
    }

    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     */
    public void update() {
        updateLedStatus();
        if (drive != null) drive.update();
        if (feeder != null) feeder.update();
        // We also need to update the internal launchSequence state machine
        if (intakeSensors != null) intakeSensors.update();
        if (vision != null) {
            vision.update();
            vision.updateRobotOrientation(drive.getHeading());

        }
        if (drive != null) drive.pinpoint.update();

    }


    /**
     * Stops all motors and mechanisms on the robot.
     */
    public void stopAll() {
        drive.stopRobot();
        launcher.setMotorVelocity(0, 0);
        intake.setIntakeMotorPower(0);
        vision.stop();
        led.setColor(LedUtil.Color.OFF);
    }

    /**
     * A consolidated method for displaying common robot telemetry.
     */
    public void addTelemetry() {
        telemetry.addData("--- Robot Status ---", "");
        //telemetry.addData("Drive State", drive.getDriveState()); // Assuming a getter in DriveUtil
        telemetry.addData("Drive Busy", drive.isBusy());
        telemetry.addData("--- Launcher ---", "");
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Launcher Velocity", launcher.getLeftMotorVelocity());
        telemetry.addData("Feeder Busy", feeder.isBusy());
        telemetry.addData("Pinpoint Positions (x, y, heading): ", drive.pinpoint.getPosition().getX(DistanceUnit.INCH) + ", " + drive.pinpoint.getPosition().getY(DistanceUnit.INCH) + ", " + drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
    }

    public void configureVisionForTeleOp(GGRobotConstants.Alliance alliance) {
        if (vision != null) {
            vision.setTargetingAlliance(alliance);
            telemetry.addData("Vision", "Configured for %s Alliance", alliance);
        }
    }

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


    public void setBlinkingColor(double color) {
        final int BLINK_INTERVAL_MS = 250;
        boolean isLedOn = (System.currentTimeMillis() / BLINK_INTERVAL_MS) % 2 == 0;
        led.setColor(isLedOn ? color : LedUtil.Color.OFF);
    }

    /**
     * Resets the robot's odometry position based on the vision system's
     * field-centric pose. This is the core of vision-based localization
     * and should be called whenever a reliable vision pose is available.
     */
    public void resetOdometryToVision() {
        Pose3D visionPose3D = vision.getRobotPoseFieldSpace();
        // Only perform the reset if the vision system has a valid field pose.
        if (visionPose3D != null) {
            // Get the X, Y, and Heading from VisionUtil.
            // Get the X, Y, and Heading from the Pose3D object.
            double visionX_meters = visionPose3D.getPosition().x;
            double visionY_meters = visionPose3D.getPosition().y;
            double visionHeading_rad = Math.toDegrees(visionPose3D.getOrientation().getYaw());

            // Create a new Pose2D object with the vision data.
            Pose2D visionPose2D = new Pose2D(DistanceUnit.METER,
                    visionX_meters,
                    visionY_meters,
                    AngleUnit.RADIANS,
                    visionHeading_rad
            );

            // Tell the Pinpoint odometry system to set its current position to this pose.
            drive.pinpoint.setPosition(visionPose2D);

            telemetry.log().add("ODOMETRY RESET to Vision Pose: ", visionX_meters * 39.3701, visionY_meters * 39.3701,
                    Math.toDegrees(visionHeading_rad));
        }
    }

    // =================================================================================
    // METHODS FOR ACTION-BASED AUTONOMOUS
    // =================================================================================

    /**
     * Executes a single AutoAction from a sequence.
     * This method acts as a router, starting the appropriate non-blocking action
     * on the correct subsystem based on the action's type.
     *
     * @param action The AutoAction to be executed.
     */
    public void executeAutoAction(AutoAction action) {
        // Use a switch statement on the action's type for clarity and extensibility.
        switch (action.type) {
            case DRIVE_TO_POINT:
                // This is a Pinpoint navigation action.
                // Call the non-blocking driveToPointAsync method from DriveUtil2026.
                // We pass the targetPose from the AutoAction object.
                drive.driveTo(drive.pinpoint.getPosition(),action.targetPose, 0.6, 0.25); // Using default speed and hold time
                break;

            case SHOOT:
                // This action is ONLY a shot. Call the non-blocking launch sequence.
                // We pass 'true' to initiate the shot and use the feeder side from the action.
                launchSequence(true, action.feederSide, GGRobotConstants.LauncherDistance.FAR);
                break;

            case CUSTOM:
                // This is a placeholder for future actions, like running an intake or moving an arm.
                // For example:
                // if (action.description.equals("Run Intake")) {
                //     intake.runIntakeForTime(2.0); // Assuming such a method exists
                // }
                break;
            // --- 3. ADD THE CASE FOR OUR NEW ACTION ---
            case DRIVE_AND_INTAKE:
                driveAndIntakeSequence(true,12);
                break;
        }
    }

    /**
     * A non-blocking state machine to handle the launch sequence for one shot.
     * It spins up the launcher, feeds a ring, and then becomes idle.
     * @param shootCommand When true, initiates the launch sequence.
     * @param sideToFire The feeder side (LEFT or RIGHT) to use.
     * @param distanceProfile The launcher distance profile (CLOSE or FAR) to determine speed.
     * @return true for one cycle when a shot has been fully completed.
     */
    public boolean launchSequence(boolean shootCommand, LaunchIndexer.FeederSide sideToFire, GGRobotConstants.LauncherDistance distanceProfile) {
        switch (launchState) {
            case IDLE:
                if (shootCommand) {
                    launcher.setMotorVelocity(distanceProfile.targetVelocity, distanceProfile.targetVelocity);
                    intake.setIntakeMotorPower(1);
                    launchState = LaunchState.SPINNING_UP;
                }
                break;

            case SPINNING_UP:
                launcher.setMotorVelocity(distanceProfile.targetVelocity, distanceProfile.targetVelocity);
                if (launcher.getLeftMotorVelocity() > distanceProfile.minVelocity &&
                        launcher.getRightMotorVelocity() > distanceProfile.minVelocity) {
                    if (sideToFire == LaunchIndexer.FeederSide.LEFT) {
                        feeder.triggerLeftFeeder();
                    } else {
                        feeder.triggerRightFeeder();
                    }
                    launchState = LaunchState.FEEDING;
                }
                break;

            case FEEDING:
                if (!feeder.isBusy()) {
                    launchCycleTimer.reset();
                    launchState = LaunchState.COOLDOWN;
                }
                break;

            case COOLDOWN:
                if (launchCycleTimer.seconds() > GGRobotConstants.Launcher.TIME_BETWEEN_SHOTS) {
                    launchState = LaunchState.IDLE;
                    return true;
                }
                break;
        }
        return false;
    }

    /**
     * A non-blocking state machine for the complex "Drive and Intake" action.
     * This method should be called repeatedly from the OpMode's loop.
     *
     * @param startSequence If true, begins the sequence. If false, just updates the current state.
     * @param driveDistance The distance to drive forward slowly while intaking.
     * @return              `true` for one cycle when the entire action is complete.
     */
    public boolean driveAndIntakeSequence(boolean startSequence, double driveDistance) {
        // If we are idle and receive the start command, begin the sequence.
        if (driveAndIntakeState == DriveAndIntakeState.IDLE && startSequence) {
            driveAndIntakeState = DriveAndIntakeState.STARTING;
        }

        // Run the state machine logic.
        switch (driveAndIntakeState) {
            case STARTING:
                // Start the intake and set the diverter for the first artifact.
                intake.setIntakeMotorPower(GGRobotConstants.Intake.INTAKE_SPEED);
                intake.setDiverterRight(); // First artifact goes to the right.
                intakeSubActionTimer.reset();
                driveAndIntakeState = DriveAndIntakeState.INTAKING_FIRST_ARTIFACT;

                // Also, start the slow forward drive using a non-blocking call.
               if (drive.driveRelative(drive.pinpoint.getPosition(),driveDistance, 0, 0, .25,2)){
                     // Drive at 20% speed.
                }
                break;

            case INTAKING_FIRST_ARTIFACT:
                // Wait until the right-side sensor detects an artifact.
                if (intakeSensors.isSlotOccupied(IntakeSensorFusion.IntakeSlot.RIGHT_2)) {
                    intake.setDiverterLeft(); // Switch diverter for the next artifacts.
                    intakeSubActionTimer.reset();
                    driveAndIntakeState = DriveAndIntakeState.SWITCHING_DIVERTER;
                }
                // Add a timeout here for robustness in a real match!
                break;

            case SWITCHING_DIVERTER:
                // Give the servo time to move.
                if (intakeSubActionTimer.seconds() > 0.5) {
                    driveAndIntakeState = DriveAndIntakeState.INTAKING_REMAINING;
                }
                break;

            case INTAKING_REMAINING:
                // Wait until both left-side sensors detect artifacts.
                if (intakeSensors.isSlotOccupied(IntakeSensorFusion.IntakeSlot.LEFT_1) &&
                        intakeSensors.isSlotOccupied(IntakeSensorFusion.IntakeSlot.LEFT_2)) {
                    intake.setIntakeMotorPower(0); // We have all three, stop the intake.
                    // The slow forward drive is still running. Now we just wait for it to finish.
                    driveAndIntakeState = DriveAndIntakeState.DRIVING_FORWARD;
                }
                // Add a timeout here for robustness in a real match!
                break;

            case DRIVING_FORWARD:
                // The intake part is done. Now we just wait for the driveAsync to finish.
                if (!drive.isBusy()) {
                    // The drive is done. The entire complex action is complete.
                    driveAndIntakeState = DriveAndIntakeState.IDLE; // Reset the state.
                    return true; // Signal completion for this one loop cycle.
                }
                break;

            case IDLE:
                // Do nothing if we are idle.
                break;
        }

        // If we have not returned true yet, it means the sequence is still in progress.
        return false;
    }

    /**
     * The master method for setting launcher velocity based on the OpMode's intent.
     * It contains all the complex orchestration logic.
     *
     * @param systemState The desired power state (IDLE or ACTIVE).
     * @param targetingMode The desired targeting mode (AUTO or PRESET).
     * @param presetDistance The fallback preset to use if in PRESET mode.
     * @return The final target velocity that was commanded to the motors.
     */
    public double updateLauncher(
            LauncherSystemState systemState,
            LauncherTargetingMode targetingMode,
            GGRobotConstants.LauncherDistance presetDistance
    )  {
        double finalTargetVelocity = 0;

        if (systemState == LauncherSystemState.IDLE) {
            // If the system is idle, always command zero velocity.
            launcher.setMotorVelocity(0, 0);
            return 0;
        }

        // If we reach here, the system is ACTIVE.
        // Now, decide which velocity to use based on the targeting mode.
        if (targetingMode == LauncherTargetingMode.AUTO) {
            // In AUTO mode, use the full waterfall calculation.
            finalTargetVelocity = updateAndGetTargetVelocity();
        } else { // PRESET mode
            // In PRESET mode, just use the provided preset.
            finalTargetVelocity = presetDistance.targetVelocity;
        }

        // Command the motors to the final determined velocity.
        launcher.setMotorVelocity(finalTargetVelocity, finalTargetVelocity);

        // Return the velocity that was just commanded, for telemetry purposes.
        return finalTargetVelocity;
    }
    /**
     * Calculates the target launcher velocity using a hierarchical "waterfall" of data sources.
     * It prefers Limelight, falls back to Odometry, and finally uses its last known good value.
     *
     * @return The calculated target velocity in ticks/sec.
     */
    public double updateAndGetTargetVelocity() {
        final double METERS_TO_INCHES = 39.3701;
        String dataSource; // For telemetry
        double newVelocity; // A temporary variable for the new calculation

        // --- The Waterfall Logic ---
        if (vision.isTargetVisible()) {
            // 1. PRIMARY: Limelight Vision is available and is our most trusted source.
            dataSource = "VISION";
            double distanceInches = vision.getDistanceToTagInches();
            newVelocity = getTargetVelocityForDistance(distanceInches);

            // We have a high-confidence value, so we update our fallback state.
            this.lastKnownGoodVelocity = newVelocity;

       }
//        else
//            if (drive.pinpoint.getLoopTime() > 0) { // A simpler check: is pinpoint sending any data?
//            // 2. SECONDARY: Limelight failed, fall back to Odometry.
//            dataSource = "ODOMETRY";
//            double distanceInches = getDistanceToGoal();
//            newVelocity = getTargetVelocityForDistance(distanceInches);
//
//            // We have a medium-confidence value, so we also update our fallback state.
//            this.lastKnownGoodVelocity = newVelocity;
//
//        }
        else {
            // 3. TERTIARY: Both vision and odometry have failed.
            dataSource = "LAST KNOWN";
            // DO NOT calculate a new value. Use the last one we successfully stored.
            newVelocity = this.lastKnownGoodVelocity;
        }

        telemetry.addData("Aiming Data Source", dataSource);
        return newVelocity; // Return the result of this loop's calculation.
    }


    /**
     * Checks if any of the robot's major subsystems are currently busy with an action.
     * @return true if the drive or launch sequence is active, false otherwise.
     */
    public boolean isBusy() {

        return drive.isBusy() || isLaunchSequenceBusy()|| (driveAndIntakeState != DriveAndIntakeState.IDLE);
    }

    /**
     * Checks if the internal launch state machine is currently active.
     * @return true if the robot is in the middle of a launch sequence, false if it is IDLE.
     */
    public boolean isLaunchSequenceBusy() {
        return launchState != LaunchState.IDLE;
    }

    public void setLauncherVelocityFromDistance() {
        Pose2D currentPos = drive.pinpoint.getPosition();

    }
    /**
     * Calculates the required flywheel velocity for a given distance using the lookup table.
     * @param distanceInches The distance to the target.
     * @return The calculated target velocity in ticks per second.
     */
    public double getTargetVelocityForDistance(double distanceInches) {
        // This method safely accesses the private flywheelTable.
        return flywheelTable.get(distanceInches);
    }

    /**
     * Calculates and returns the distance from the robot to the fixed goal.
     * This method is a wrapper around the DriveUtil's distance calculation.
     * @return The distance to the goal in INCHES.
     */
    public double getDistanceToGoal() {
        // Define the goal's location. This should be a constant.
        Pose2D trgtPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,45);
        Pose2D currPose = drive.pinpoint.getPosition();
        return drive.distanceTo(currPose, trgtPose, DistanceUnit.INCH);
    }
    public double getDistanceToGoalFieldRelative() {
        // Define the goal's location. This should be a constant.
        //if alliance is red
        // 2. Get the current alliance from the shared static variable.
        GGRobotConstants.Alliance currentAlliance = SharedState.alliance;

        // 3. Get the correct heading error from the vision system based on that alliance.
        Pose2D trgtPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,45);;
        if (currentAlliance == GGRobotConstants.Alliance.RED) {
               trgtPose = new Pose2D(DistanceUnit.METER,GGRobotConstants.GoalLocation.RED_TAG24_X_M, GGRobotConstants.GoalLocation.RED_TAG24_Y_M, AngleUnit.DEGREES,54);
        } else { // BLUE
                trgtPose = new Pose2D(DistanceUnit.METER,GGRobotConstants.GoalLocation.BLUE_TAG20_X_M, GGRobotConstants.GoalLocation.BLUE_TAG20_Y_M, AngleUnit.DEGREES,45);
        }
        Pose2D currPose = drive.pinpoint.getPosition();
        return drive.distanceTo(currPose, trgtPose, DistanceUnit.INCH);
    }

    /**
     * The master method for calculating auto-aim turn power during TeleOp.
     * It orchestrates the SharedState, Vision, and Drive subsystems to produce a single command.
     * This is the ONLY auto-aim method the OpMode should ever call.
     *
     * @return The calculated turn power (-1.0 to 1.0) to send to the drivetrain.
     */
    public double calculateAutoAimTurnPower() {
        // --- Start of Telemetry Section ---
        telemetry.addLine("--- Auto-Aim Calculation ---");
        // 1. Check if vision system can provide an aim point.
        if (!vision.hasFieldPose()) {
            telemetry.addData("AutoAim", "OFF (No Field Pose)");
            return 0.0; // Cannot aim without a field pose.
        }

        // --- Get Robot's Current Position ---
        // We get the current pose from the pinpoint odometry.
        Pose2D robotPose = drive.pinpoint.getPosition();
        double robotX_in = robotPose.getX(DistanceUnit.INCH);
        double robotY_in = robotPose.getY(DistanceUnit.INCH);
        double robotHeading_deg = robotPose.getHeading(AngleUnit.DEGREES);

        // Display the robot's current position
        telemetry.addData("Robot Field Pos (X, Y, H)", "%.1f in, %.1f in, %.1f deg",
                robotX_in, robotY_in, robotHeading_deg);

        // 2. Get the current alliance from the shared static variable.
        GGRobotConstants.Alliance currentAlliance = SharedState.alliance;
        telemetry.addData("AutoAim Alliance", currentAlliance);


        // --- Determine Target Goal Coordinates ---
        double goalX_m;
        double goalY_m;

        if (currentAlliance == GGRobotConstants.Alliance.RED) {
            goalX_m = GGRobotConstants.GoalLocation.RED_TAG24_X_M;
            goalY_m = GGRobotConstants.GoalLocation.RED_TAG24_Y_M;
            telemetry.addData("Target", "RED Goal (Tag 24)");
        } else { // BLUE
            goalX_m = GGRobotConstants.GoalLocation.BLUE_TAG20_X_M;
            goalY_m = GGRobotConstants.GoalLocation.BLUE_TAG20_Y_M;
            telemetry.addData("Target", "BLUE Goal (Tag 20)");
        }

        // Display the target coordinates (converted to inches for consistency)
        telemetry.addData("Target Field Pos (X, Y)", "%.1f in, %.1f in",
                goalX_m * 39.3701, goalY_m * 39.3701);


        // 3. Get the correct heading error from the vision system based on that alliance.
        double headingErrorDeg;
        if (currentAlliance == GGRobotConstants.Alliance.RED) {
            headingErrorDeg = vision.getHeadingErrorToRedTag24Deg();
        } else { // BLUE
            headingErrorDeg = vision.getHeadingErrorToBlueTag20Deg();
        }

        // 4. Pass the simple heading error to the DriveUtil specialist to get the motor command.
        //    The drive utility doesn't need to know anything about alliances or tags.
        double turnCmd = drive.calculateAutoAimTurn(headingErrorDeg);

        // 5. Add telemetry for debugging and return the final command.
        telemetry.addData("AutoAim Alliance", currentAlliance);
        telemetry.addData("AutoAim", "ON | Error: %.1f deg | Cmd: %.2f", headingErrorDeg, turnCmd);
        telemetry.addData("FINAL Heading Error", "%.1f deg", headingErrorDeg);
        telemetry.addData("FINAL Turn Command", "%.2f", turnCmd);
        telemetry.addLine("--------------------------");
        return turnCmd;
    }
    // =================================================================================
    // AUTONOMOUS PATH PLANNING
    // =================================================================================

    /**
     * The master planner for building an entire autonomous sequence.
     * This method contains all the logic for choosing waypoints and actions based
     * on the starting position and vision results.
     *
     * @param alliance  The selected Alliance (RED or BLUE).
     * @param location  The selected starting Location (CLOSE or FAR).
     * @param detectedMotif The MotifPattern detected by the vision system.
     * @return A complete List<AutoAction> ready for the OpMode to execute.
     */
    public List<AutoAction> buildAutonomousSequence(
            GGRobotConstants.Alliance alliance,
            GGRobotConstants.Location location,
            VisionUtil.MotifPattern detectedMotif
    ) {
        List<AutoAction> sequence = new ArrayList<>();
        telemetry.log().add("GGRobot: Building new sequence for " + alliance + "/" + location);

        // --- Step 1: Determine all necessary waypoints based on selections ---
        final Pose2D driveToScorePose;
        final Pose2D parkPose;

        if (location == GGRobotConstants.Location.CLOSE) {
            driveToScorePose = (alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.RED_CLOSE_DRIVE_AWAY : GGRobotConstants.Waypoints.BLUE_CLOSE_DRIVE_AWAY;
            parkPose = (alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.RED_CLOSE_PARK : GGRobotConstants.Waypoints.BLUE_CLOSE_PARK;
        } else { // Location.FAR
            driveToScorePose = (alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.RED_FAR_DRIVE_TO_SCORE : GGRobotConstants.Waypoints.BLUE_FAR_DRIVE_TO_SCORE;
            parkPose = (alliance == GGRobotConstants.Alliance.RED) ? GGRobotConstants.Waypoints.RED_FAR_PARK : GGRobotConstants.Waypoints.BLUE_FAR_PARK;
        }


        // --- Step 2: Build the Action "Script" ---

        // 1. Initial Drive
        sequence.add(AutoAction.createDriveAction("Drive to Scoring Pos", driveToScorePose));

        // 2. Add Shooting Sequence based on the detected motif
        // Assuming Green is on the Right, Purple is on the Left for pre-loads
        switch (detectedMotif) {
            case GPP21: // green purple purple
                sequence.add(AutoAction.createShootAction("Shoot Green", LaunchIndexer.FeederSide.RIGHT));
                sequence.add(AutoAction.createShootAction("Shoot Purple 1", LaunchIndexer.FeederSide.LEFT));
                sequence.add(AutoAction.createShootAction("Shoot Purple 2", LaunchIndexer.FeederSide.LEFT));
                break;
            case PGP22: // Purple Green Purple
                sequence.add(AutoAction.createShootAction("Shoot Purple 1", LaunchIndexer.FeederSide.LEFT));
                sequence.add(AutoAction.createShootAction("Shoot Green", LaunchIndexer.FeederSide.RIGHT));
                sequence.add(AutoAction.createShootAction("Shoot Purple 2", LaunchIndexer.FeederSide.LEFT));
                break;
            case PPG23: // Purple Purple Green
            case UNKNOWN:
            default:
                sequence.add(AutoAction.createShootAction("Shoot Purple 1", LaunchIndexer.FeederSide.LEFT));
                sequence.add(AutoAction.createShootAction("Shoot Purple 2", LaunchIndexer.FeederSide.LEFT));
                sequence.add(AutoAction.createShootAction("Shoot Green", LaunchIndexer.FeederSide.RIGHT));
                break;
        }

        // 3. Drive to Intake Area and run intake
        // We can add a placeholder drive action before the intake.
        sequence.add(AutoAction.createDriveAndIntakeAction("Intake from Floor", 12.0));

        // 4. Drive to Parking Position
        sequence.add(AutoAction.createDriveAction("Park", parkPose));


        telemetry.log().add("GGRobot: Sequence built with " + sequence.size() + " actions.");
        return sequence; // Return the completed plan
    }
}
