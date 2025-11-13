package org.firstinspires.ftc.teamcode.utilities.Skyline;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;

/**
 * Skyline_Robot is the central hub that orchestrates all of the Skyline robot's subsystems.
 * It owns all the hardware and utility classes, providing a clean interface for OpModes.
 */
public class Skyline_Robot {

    // Public subsystems so the OpMode can access them for direct driver control
    public final DriveUtil2026b drive;
    public final Skyline_LauncherUtil launcher;
    public final Skyline_FeederUtil feeder;
    public final Telemetry telemetry;

    // --- State Machine for the Launch Sequence ---
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime feederTimer = new ElapsedTime();

    public Skyline_Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        RobotConfig config = RobotConfig.createDefaultSkyLineConfig();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, config); // Pass opMode context
        launcher = new Skyline_LauncherUtil(hardwareMap);
        feeder = new Skyline_FeederUtil(hardwareMap);
    }

    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     * For now, it updates the launch sequence state machine.
     */
    public void update() {
        // This is where you would call update() on any subsystems that need it
        // e.g., vision.update();
        // The launch state machine is managed by the launch() method itself.
    }

    /**
     * The non-blocking state machine for launching a game element.
     * This orchestrates the launcher and feeder subsystems.
     * @param shotRequested True if the driver has requested a shot on this loop cycle.
     */
    public boolean launchSequence(boolean shotRequested, double targetVelocity, double minVelocity, double feedTime) {
        switch (launchState) {
            case IDLE:
                // If a shot is requested, start spinning up the launcher
                if (shotRequested) {
                    launcher.setVelocity(targetVelocity);
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                // Continuously command the velocity to ensure it gets there
                launcher.setVelocity(targetVelocity);
                // If the flywheel is at speed, move to the launch state
                if (launcher.getVelocity() > minVelocity) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                // Start the feeders and a timer
                feeder.setPower(1.0);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                // If the feed time has elapsed, stop the feeders and reset
                if (feederTimer.seconds() > feedTime) {
                    feeder.stop();
                    // Optionally, stop the launcher motor too, or let it coast
                    // launcher.setVelocity(0);
                    launchState = LaunchState.IDLE;
                    return true;
                }
                break;
        }
        telemetry.addData("Launch State", launchState); // Add state to telemetry
        return false;
    }

    public void stopAll() {
        drive.stopRobot();
        launcher.setVelocity(0);
        feeder.stop();
    }
}
