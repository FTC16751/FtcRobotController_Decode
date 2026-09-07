package org.firstinspires.ftc.teamcode.teams.skyline;

import org.firstinspires.ftc.teamcode.teams.skyline.subsystems.Skyline_FeederUtil;
import org.firstinspires.ftc.teamcode.teams.skyline.subsystems.Skyline_LauncherUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.CommonConstants;
import org.firstinspires.ftc.teamcode.common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.common.Feeder;
import org.firstinspires.ftc.teamcode.common.Flywheel;
import org.firstinspires.ftc.teamcode.common.InterpolatingLookupTable;
import org.firstinspires.ftc.teamcode.common.LaunchController;
import org.firstinspires.ftc.teamcode.common.LedUtil;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.VisionUtil;

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

    // --- Launch sequence: shared common.LaunchController with Skyline's launcher and feeder plugged in ---
    public final LaunchController launchController;
    private InterpolatingLookupTable flywheelTable;
    private double lastKnownGoodVelocity = 0.0;
    public Skyline_Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        RobotConfig config = SkylineBotConfig.create();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, config); // Pass opMode context
        launcher = new Skyline_LauncherUtil(hardwareMap);
        feeder = new Skyline_FeederUtil(hardwareMap);
        vision = new VisionUtil(hardwareMap, telemetry, config.hardware.limelight);
        led = new LedUtil(hardwareMap, config.hardware.led);

        // Shared launch sequence. Skyline's autos pass an absolute minimum velocity and a feed time
        // per call, so readyFraction is unused and feedTimeSec is set on each call (see launchSequence).
        // No cooldown and no stall check, matching how this robot shot before 2026-09-07. The 2 s
        // spin-up timeout is new: the old code waited forever if the flywheel never reached speed.
        Flywheel skylineFlywheel = new Flywheel() {
            @Override public void setVelocity(double v) { launcher.setVelocity(v); }
            @Override public double getVelocity()       { return launcher.getVelocity(); }
        };
        Feeder skylineFeeder = new Feeder() {
            @Override public void start() { feeder.setPower(1.0); }
            @Override public void stop()  { feeder.stop(); }
        };
        launchController = new LaunchController(skylineFlywheel, skylineFeeder,
                new LaunchController.Settings()
                        .cooldownSec(0)
                        .spinUpTimeoutSec(2.0)
                        .stallFraction(0)
                        .keepSpinning(true),
                telemetry);

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
        launchController.settings.feedTimeSec = feedTime;
        boolean done = launchController.update(shotRequested, targetVelocity, minVelocity);
        telemetry.addData("Launch State", launchController.getState());
        return done;
    }

    public void stopAll() {
        drive.stopRobot();
        launchController.stop();   // stops flywheel and feeder, resets the sequence to IDLE
        vision.stop();
    }

    public void configureVisionForTeleOp(CommonConstants.Alliance alliance) {
        if (vision != null) {
            vision.setTargetingAlliance(alliance);
            telemetry.addData("Vision", "Configured for %s Alliance", alliance);
        }
    }

    public double updateAndGetTargetVelocity() {
        final double METERS_TO_INCHES = CommonConstants.METERS_TO_INCHES;
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
