package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026;

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

    // 2. PRIVATE STATE AND TIMERS FOR ROBOT-LEVEL ACTIONS
    private enum LaunchState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        COOLDOWN
    }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime launchCycleTimer = new ElapsedTime();

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

        // Set initial robot hardware states
        drive.resetHeading();
        stopAll();
    }

    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     */
    public void update() {
        if (drive != null) drive.update();
        if (feeder != null) feeder.update();
        // We also need to update the internal launchSequence state machine
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
     * Checks if any of the robot's major subsystems are currently busy with an action.
     * @return true if the drive or launch sequence is active, false otherwise.
     */
    public boolean isBusy() {
        return drive.isBusy() || isLaunchSequenceBusy();
    }

    /**
     * Checks if the internal launch state machine is currently active.
     * @return true if the robot is in the middle of a launch sequence, false if it is IDLE.
     */
    public boolean isLaunchSequenceBusy() {
        return launchState != LaunchState.IDLE;
    }
}
