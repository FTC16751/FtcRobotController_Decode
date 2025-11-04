package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import android.graphics.PorterDuff;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026;
import org.firstinspires.ftc.teamcode.utilities.Common.InterpolatingLookupTable;

/**
 * The Robot class is the central hub that orchestrates all of the robot's subsystems.
 * It owns all the hardware and utility classes, providing a clean and high-level
 * interface for OpModes (TeleOp and Autonomous) to use. This promotes code reuse and
 * simplifies the OpMode logic significantly.
 */
public class GGRobot {

    // 1. PUBLIC SUBSYSTEMS
    public final DriveUtil2026 drive;
    public final LauncherMotors launcher;
    public final LaunchIndexer feeder;
    public final IntakeUtil intake;
    public final Telemetry telemetry;
    public final IntakeSensorFusion intakeSensors;


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

    /**
     * The constructor for the Robot class.
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry The telemetry object from the OpMode.
     */
    public GGRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize all subsystems
        drive = new DriveUtil2026(hardwareMap, telemetry, null);
        launcher = new LauncherMotors(hardwareMap);
        feeder = new LaunchIndexer(hardwareMap);
        intake = new IntakeUtil(hardwareMap);
        intakeSensors = new IntakeSensorFusion(hardwareMap, telemetry);

        flywheelTable = new InterpolatingLookupTable();
        flywheelTable = new InterpolatingLookupTable();
        flywheelTable.add(30.0, 960.0);
        flywheelTable.add(40.0, 1010.0);
        flywheelTable.add(50.0, 1070.0);
        flywheelTable.add(60.0, 1130.0);
        flywheelTable.add(70.0, 1200.0);
        flywheelTable.add(80.0, 1280.0);
        flywheelTable.add(100.0, 1390.0);
        flywheelTable.add(120.0, 1420.0);
        flywheelTable.add(130.0, 1470.0);
        flywheelTable.add(140.0, 1500.0);
        // Set initial robot hardware states
        drive.resetHeading();
        stopAll();
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
    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     */
    public void update() {
        if (drive != null) drive.update();
        if (feeder != null) feeder.update();
        // We also need to update the internal launchSequence state machine
        if (intakeSensors != null) intakeSensors.update();

        drive.pinpoint.update();
        telemetry.addData("from ggrobot X coordinate (IN)", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("from gg robot Y coordinate (IN)", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("from gg robot Heading angle (DEGREES)", drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
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
    }

    /**
     * Stops all motors and mechanisms on the robot.
     */
    public void stopAll() {
        drive.stopRobot();
        launcher.setMotorVelocity(0, 0);
        intake.setIntakeMotorPower(0);
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
    public void execute(AutoAction action) {
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
                if (intakeSensors.isSlotOccupied(IntakeSensorFusion.IntakeSlot.RIGHT_1)) {
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
}
