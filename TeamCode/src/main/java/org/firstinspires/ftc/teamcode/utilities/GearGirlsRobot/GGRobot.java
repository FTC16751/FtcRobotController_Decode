package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import static org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil.RED_GOAL_X_COORDINATE_METERS;
import static org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil.RED_GOAL_Y_COORDINATE_METERS;

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
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;
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

    //PUBLIC SUBSYSTEMS
    public final DriveUtil2026b drive;
    public final LauncherMotors launcher;
    public final LaunchIndexer feeder;
    public final IntakeUtil intake;
    public final Telemetry telemetry;
    public final VisionUtil vision;
    public final IntakeSensorFusion intakeSensors;
    public final LedUtil led;

    //PRIVATE STATE AND TIMERS FOR ROBOT-LEVEL ACTIONS
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
    private InterpolatingLookupTable flywheelTable;
    private double lastKnownGoodVelocity = 0.0;

    /**
     * The constructor for the Robot class.
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry The telemetry object from the OpMode.
     */
    public GGRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Get the robot configuration from the config file
        RobotConfig ggConfig = RobotConfig.createDefaultGearGirlsConfig();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, ggConfig);
        launcher = new LauncherMotors(hardwareMap);
        feeder = new LaunchIndexer(hardwareMap);
        intake = new IntakeUtil(hardwareMap);
        intakeSensors = new IntakeSensorFusion(hardwareMap, telemetry);
        vision = new VisionUtil(hardwareMap, telemetry);
        led = new LedUtil(hardwareMap, "led_servo");

        //this is our flywheel lookup table. there's probably a better place to put this, but it's what we have now
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
        if (feeder != null) feeder.update();
        if (intakeSensors != null) intakeSensors.update();
        if (vision != null) {
            vision.update();
            vision.updateRobotOrientation(drive.getHeading());

        }
        //i don't think we need to do this, it should be covered in drive.update()
        //if (drive != null) drive.pinpoint.update();

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
     * field-centric pose.
     * WORK IN PROGRESS. THE IDEA HERE IS TO USE THE APRIL TAG TO
     * RESET THE TELOP ODOMETRY FOR FIELD-RELAATIVE DRIVING.
     */
    public void resetOdometryToVision() {
        Pose3D visionPose3D = vision.getRobotPoseFieldSpace();
        // Only perform the reset if the vision system has a valid field pose.
        if (visionPose3D != null) {
            // Get the X, Y, and Heading from VisionUtil.
            double visionX_meters = visionPose3D.getPosition().x;
            double visionY_meters = visionPose3D.getPosition().y;
            double visionHeading_rad = Math.toDegrees(visionPose3D.getOrientation().getYaw());
            double tx = vision.getTargetAngleX();

            // Get the current alliance from the shared state.
            GGRobotConstants.Alliance currentAlliance = SharedState.alliance;

            double targetX_m;
            double targetY_m;

            if (currentAlliance == GGRobotConstants.Alliance.RED) {
                targetX_m = GGRobotConstants.GoalLocation.RED_TAG24_X_M;
                targetY_m = GGRobotConstants.GoalLocation.RED_TAG24_Y_M;
            } else { // BLUE alliance
                targetX_m = GGRobotConstants.GoalLocation.BLUE_TAG20_X_M;
                targetY_m = GGRobotConstants.GoalLocation.BLUE_TAG20_Y_M;
            }

            double tagY = RED_GOAL_Y_COORDINATE_METERS;
            double tagX = RED_GOAL_X_COORDINATE_METERS;

            // This offset defines "forward" from the driver's perspective.
            // 90 degrees makes "away from the driver station" the new 0-degree heading.
            double DRIVER_ROT_OFFSET = 90;

            //COMMENT THIS OUT FOR NOW. FIRST TEST TO JUST SET HEADING SO FIELD RELATIVE DRIVNG
            // works accoding to where the drive team is standing (red or blue)
            // Create a new Pose2D object with the vision data.
            Pose2D visionPose2D = new Pose2D(DistanceUnit.METER,
                    visionX_meters,
                    visionY_meters,
                    AngleUnit.RADIANS,
                    visionHeading_rad
            );
            // Tell the Pinpoint odometry system to set its current position to this pose.
            //drive.pinpoint.setPosition(visionPose2D);


            //Calculate the absolute angle on the field from the robot to the target tag.
            double angleFieldToTagDeg = Math.toDegrees(Math.atan2(targetY_m - visionY_meters, targetX_m - visionX_meters));

            //Calculate the true field heading by correcting the angle with the Limelight's 'tx' error.
            double fieldHeadingDeg = angleFieldToTagDeg - tx;
            fieldHeadingDeg = AngleUnit.normalizeDegrees(fieldHeadingDeg);

            //Create the "driver-relative" heading by applying the offset.
            double driverHeadingDeg = AngleUnit.normalizeDegrees(fieldHeadingDeg - DRIVER_ROT_OFFSET);

            // set the New Heading in pinpoint
            drive.pinpoint.setHeading(driverHeadingDeg, AngleUnit.DEGREES);

            telemetry.log().add("ODOMETRY RESET to Vision Pose: ", visionX_meters * 39.3701, visionY_meters * 39.3701,
                    Math.toDegrees(visionHeading_rad));
        }
    }

    // =================================================================================
    // METHODS FOR ACTION-BASED AUTONOMOUS
    // =================================================================================


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
     * method for setting launcher velocity based on the OpMode's intent.
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
        return flywheelTable.get(distanceInches);
    }

    /**
     * Calculates and returns the distance from the robot to the fixed goal.
     * This method is a wrapper around the DriveUtil's distance calculation.
     * @return The distance to the goal in INCHES.
     */
    public double getDistanceToGoal() {
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
     * Tmethod for calculating auto-aim turn power during TeleOp.
     *
     * This is one of 2 trials for auto-aim.
     * the other takes in the tx value from the limelight and uses that to calculate the turn power.
     * THIS METHOD IS CURRENTLY INCONSISTENT BECAUSE OUR ODOMETRY IN TELEOP IS NOT BASED ON THE FIELD COORDINATES.
     * @return The calculated turn power (-1.0 to 1.0) to send to the drivetrain.
     */
    public double calculateAutoAimTurnPower() {
        telemetry.addLine("--- Auto-Aim Calculation ---");
        //  Check if vision system can provide an aim point.
        if (!vision.hasFieldPose()) {
            telemetry.addData("AutoAim", "OFF (No Field Pose)");
            return 0.0; // Cannot aim without a field pose.
        }

        // --- Get Robot's Current Position from the pinpoint odometry.
        Pose2D robotPose = drive.pinpoint.getPosition();

        // get the robot's position in inches from
        double robotX_in = robotPose.getX(DistanceUnit.INCH);
        double robotY_in = robotPose.getY(DistanceUnit.INCH);
        double robotHeading_deg = robotPose.getHeading(AngleUnit.DEGREES);

        // I'm not sure if this is the right pose to get from limelight, dsplay it in telemetry
        telemetry.addData("Robot Field Pos (X, Y, H)", "%.1f in, %.1f in, %.1f deg",
                robotX_in, robotY_in, robotHeading_deg);

        //Get the current alliance from the shared static variable.
        GGRobotConstants.Alliance currentAlliance = SharedState.alliance;
        telemetry.addData("AutoAim Alliance", currentAlliance);


        // Determine Target Goal Coordinates. These are constants based on ftc field coordinates
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

        // Display the target coordinates
        telemetry.addData("Target Field Pos (X, Y)", "%.1f in, %.1f in",
                goalX_m * 39.3701, goalY_m * 39.3701);


        // Get the correct heading error from the vision system based on that alliance.
        double headingErrorDeg;
        if (currentAlliance == GGRobotConstants.Alliance.RED) {
            headingErrorDeg = vision.calculateHeadingErrorToRedGoalDegrees();
        } else { // BLUE
            headingErrorDeg = vision.calculateHeadingErrorToBlueGoalDegrees();
        }

        //Pass the simple heading error to the DriveUtil
        double turnCmd = drive.calculateAutoAimTurn(headingErrorDeg);

        // telemetry for debugging and return the final command.
        telemetry.addData("AutoAim Alliance", currentAlliance);
        telemetry.addData("AutoAim", "ON | Error: %.1f deg | Cmd: %.2f", headingErrorDeg, turnCmd);
        telemetry.addData("FINAL Heading Error", "%.1f deg", headingErrorDeg);
        telemetry.addData("FINAL Turn Command", "%.2f", turnCmd);
        telemetry.addLine("--------------------------");
        return turnCmd;
    }

}
