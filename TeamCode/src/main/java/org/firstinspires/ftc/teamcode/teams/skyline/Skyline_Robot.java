package org.firstinspires.ftc.teamcode.teams.skyline;

import org.firstinspires.ftc.teamcode.teams.skyline.subsystems.Skyline_FeederUtil;
import org.firstinspires.ftc.teamcode.teams.skyline.subsystems.Skyline_LauncherUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.AimLed;
import org.firstinspires.ftc.teamcode.common.CommonConstants;
import org.firstinspires.ftc.teamcode.common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.common.Feeder;
import org.firstinspires.ftc.teamcode.common.FlywheelVelocityModel;
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
    private final FlywheelVelocityModel flywheelModel;   // distance -> velocity, remembers last good
    private final AimLed aimLed;                         // LED shows lined up / left / right / none
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

        // Aiming helpers: the table and LED settings live in SkylineConstants
        flywheelModel = new FlywheelVelocityModel(
                SkylineConstants.Launcher.FLYWHEEL_TABLE,
                SkylineConstants.Launcher.FLYWHEEL_INITIAL_FALLBACK);
        aimLed = new AimLed(led, vision, SkylineConstants.Aim.LED_TOLERANCE_DEG,
                new AimLed.Colors()
                        .goalToRight(SkylineConstants.Aim.LED_GOAL_RIGHT)
                        .goalToLeft(SkylineConstants.Aim.LED_GOAL_LEFT));
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
        double velocity = flywheelModel.update(vision);
        telemetry.addData("Aiming Data Source", flywheelModel.getLastSource());
        return velocity;
    }
    public double getTargetVelocityForDistance(double distanceInches) {
        return flywheelModel.velocityForDistance(distanceInches);
    }

    private void updateLedStatus() {
        aimLed.update();
    }
}
