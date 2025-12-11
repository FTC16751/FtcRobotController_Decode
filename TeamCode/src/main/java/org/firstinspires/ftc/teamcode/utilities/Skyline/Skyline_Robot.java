package org.firstinspires.ftc.teamcode.utilities.Skyline;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.utilities.Common.InterpolatingLookupTable;
import org.firstinspires.ftc.teamcode.utilities.Common.LedUtil;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;

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
    public final VisionUtil vision;
    public final LedUtil led;

    // --- State Machine for the Launch Sequence ---
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime feederTimer = new ElapsedTime();
    private InterpolatingLookupTable flywheelTable;
    private double lastKnownGoodVelocity = 0.0;
    public Skyline_Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        RobotConfig config = RobotConfig.createDefaultSkyLineConfig();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, config); // Pass opMode context
        launcher = new Skyline_LauncherUtil(hardwareMap);
        feeder = new Skyline_FeederUtil(hardwareMap);
        vision = new VisionUtil(hardwareMap, telemetry);
        led = new LedUtil(hardwareMap, "led_servo");

        flywheelTable = new InterpolatingLookupTable();
        flywheelTable.add(30.0, 1200.0*1.045);
        flywheelTable.add(40.0, 1200.0*1.045);
        flywheelTable.add(50.0, 1230.0*1.045);
        flywheelTable.add(60.0, 1260.0*1.05);
        flywheelTable.add(70.0, 1285.0*1.05);
        flywheelTable.add(80.0, 1340.0*1.045);
        flywheelTable.add(90.0, 1420.0*1.04);
        flywheelTable.add(100.0, 1460.0*1.04);
        flywheelTable.add(110.0, 1480.0*1.04);
        flywheelTable.add(120.0, 1560.0*1.04);
        flywheelTable.add(130.0, 1640.0*1.04);
        flywheelTable.add(140.0, 1720.0*1.04);
        flywheelTable.add(150.0, 1760.0);
    }

    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     * For now, it updates the launch sequence state machine.
     */
    public void update() {
        if (drive != null) drive.update();
        if (vision != null) vision.update();
        if (led != null) updateLedStatus();

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
        vision.stop();
    }

    public void configureVisionForTeleOp(CommonConstants.Alliance alliance) {
        if (vision != null) {
            vision.setTargetingAlliance(alliance);
            telemetry.addData("Vision", "Configured for %s Alliance", alliance);
        }
    }

    public double updateAndGetTargetVelocity() {
        final double METERS_TO_INCHES = 39.3701;
        String dataSource; // For telemetry
        double newVelocity; // A temporary variable for the new calculation

        if (vision.isTargetVisible()) {
            // Limelight Vision
            dataSource = "VISION";
            double distanceInches = vision.getDistanceToTagInches();
            newVelocity = getTargetVelocityForDistance(distanceInches);

            // We have a high-confidence value, so we update our fallback state.
            this.lastKnownGoodVelocity = newVelocity;

        }
        else {
            dataSource = "LAST KNOWN";
            // DO NOT calculate a new value. Use the last one we successfully stored.
            newVelocity = this.lastKnownGoodVelocity;
        }

        telemetry.addData("Aiming Data Source", dataSource);
        return newVelocity; // Return the result of this loop's calculation.
    }
    public double getTargetVelocityForDistance(double distanceInches) {
        return flywheelTable.get(distanceInches);
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
            led.setColor(LedUtil.Color.ORANGE);
        } else if (headingError < -AIMING_TOLERANCE_DEG) {
            led.setColor(LedUtil.Color.BLUE);
        }
    }
}
