package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.utilities.Common.InterpolatingLookupTable;
import org.firstinspires.ftc.teamcode.utilities.Common.LedUtil;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * P3_Robot is the central hub that orchestrates all of the P3 robot's subsystems.
 * It owns all the hardware and utility classes, providing a clean interface for OpModes.
 */
public class P3_Robot {

    // --- PUBLIC SUBSYSTEMS ---
    // These are public so the OpMode can access them directly (e.g., robot.drive.arcadeDrive(...))
    public final DriveUtil2026b drive;
    public final P3_IntakeUtil intake;
    public final P3_LauncherUtil launcher;
    public final VisionUtil vision;
    public final Telemetry telemetry;
    public final IMU imu;
    public final LedUtil led;
    private InterpolatingLookupTable flywheelTable;
    public final P3_IndexerUtil feeder;

    private enum LaunchState {
        IDLE,         // The sequence is not running.
        SPIN_UP,      // Flywheels are accelerating to target speed.
        FEEDING,      // Feeder/indexer is running to push artifact into flywheels.
        COOLDOWN      // A brief pause after a shot before the next one can start.
    }

    private LaunchState launchState = LaunchState.IDLE;
    private final ElapsedTime launchTimer = new ElapsedTime();

    // These constants define the timing of the launch sequence. They belong here
    // as they define a robot-level behavior.
    private static final double FEED_TIME_SECONDS = 2.5; // How long to run the indexer for each shot.
    private static final double COOLDOWN_TIME_SECONDS = 0.25; // Brief pause between shots.
    // ---------------------------------------------------


    /**
     * Constructor for the P3_Robot class.
     */
    public P3_Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        RobotConfig config = RobotConfig.createP3Robot2Config();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, config);
        intake = new P3_IntakeUtil(hardwareMap);
        launcher = new P3_LauncherUtil(hardwareMap);
        vision = new VisionUtil(hardwareMap, telemetry);
        imu = hardwareMap.get(IMU.class, "imu");
        led = new LedUtil(hardwareMap, "light");
        feeder = new P3_IndexerUtil(hardwareMap);


        flywheelTable = new InterpolatingLookupTable();
        flywheelTable.add(30.0, 1000.0);
        flywheelTable.add(40.0, 1050.0);
        flywheelTable.add(45.0, 1100.0);
        flywheelTable.add(50.0, 1030.0);
        flywheelTable.add(55.0, 1070.0);
        flywheelTable.add(60.0, 1110.0);
        flywheelTable.add(65.0, 1150.0);
        flywheelTable.add(70.0, 1180.0);
        flywheelTable.add(75.0, 1220.0);
        flywheelTable.add(80.0, 1260.0);
        flywheelTable.add(90.0, 1320.0);
        flywheelTable.add(100.0, 1390.0);
        flywheelTable.add(110.0, 1440.0);
        flywheelTable.add(120.0, 1500.0);
        flywheelTable.add(125.0, 1530.0);
        flywheelTable.add(130.0, 1560.0);
        flywheelTable.add(135.0, 1580.0);
        flywheelTable.add(140.0, 1600.0);
    }

    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     */
    public void update() {
        // In the future, any subsystems that need continuous updates would be called here.
        // For now, it's a placeholder.
        // e.g., drive.update();
        vision.update();
        drive.update();
        if (led != null) updateLedStatus();


    }

    /**
     * Stops all motors and mechanisms on the robot. Call this in the OpMode's stop() method.
     */
    public void stopAll() {
        drive.stopRobot(); // Assuming a method like this exists in DriveUtil2026
        intake.setIntakePower(0);
        launcher.setShooterMotorVelocity(0);
        vision.stop();
    }

    public double getTargetVelocityForDistance(double distanceInches) {
        // This method safely accesses the private flywheelTable.
        return flywheelTable.get(distanceInches);
    }

    public double getFlywheelRpmForDistance(double distanceInches) {
        // Quadratic fit: RPM = a*d^2 + b*d + c
        double a = -0.022687;   // distance^2 coefficient
        double b = 12.2017;     // distance coefficient
        double c = 717.276;     // constant term

        double rpm = a * distanceInches * distanceInches
                + b * distanceInches
                + c;

        // Optional: clamp to your tested range
        if (distanceInches < 40.0) distanceInches = 40.0;
        if (distanceInches > 140.0) distanceInches = 140.0;

        return rpm;
    }

    /**
     * A consolidated method for displaying common robot telemetry.
     */
    public void addTelemetry() {
        telemetry.addLine("--- P3 Robot Telemetry ---");
        // Add telemetry from subsystems
        vision.addTelemetry();
        // You can add more telemetry from other subsystems here
        // e.g., telemetry.addData("Shooter Velocity", launcher.getShooterMotorVelocity());
    }

    public boolean launchSequence(boolean shootCommand, double targetVelocity) {
        telemetry.addData("launch sequence: ", shootCommand);
        switch (launchState) {
            case IDLE:
                telemetry.addData("launch sequence state: ", launchState);

                if (shootCommand) {
                    // A shot is requested. Start spinning up the motors.
                    launcher.setShooterMotorVelocity(targetVelocity);
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                telemetry.addData("launch sequence state: ", launchState);

                // Continuously command the velocity to ensure it gets there.
                launcher.setShooterMotorVelocity(targetVelocity);
                if (launcher.getShooterMotorVelocity() >= targetVelocity * 0.98) {
                    launcher.setIndexerServoPower(-1.0);
                    launcher.setShootingPosition();

                    launchTimer.reset(); // Start the timer for the feeding duration.
                    launchState = LaunchState.FEEDING;
                }
                break;

            case FEEDING:
                telemetry.addData("launch sequence state: ", launchState);

                // The feeder runs for a specific amount of time.
                if (launchTimer.seconds() > FEED_TIME_SECONDS) {
                    launcher.setIndexerServoPower(0.0);
                    //launcher.setStopPosition();

                    launchTimer.reset(); // Start the timer for the cooldown period.
                    launchState = LaunchState.COOLDOWN;
                } else {
                    launcher.setIndexerServoPower(-1.0);
                    launcher.setShootingPosition();
                }
                break;

            case COOLDOWN:
                // A brief pause to allow systems to settle before the next shot.
                if (launchTimer.seconds() > COOLDOWN_TIME_SECONDS) {
                    launchState = LaunchState.IDLE; // The sequence is complete.
                    return true; // Signal completion for one loop cycle.
                }
                break;
        }
        // Add telemetry to see what the state machine is doing
        telemetry.addData("Launch State", launchState);

        // If we haven't returned true yet, the sequence is still busy.
        return false;
    }
    /**
     * A helper method to let the OpMode know if the launch sequence is currently running.
     * @return true if the state is not IDLE, otherwise false.
     */
    public boolean isLaunchSequenceBusy() {
        return launchState != LaunchState.IDLE;
    }

    /**
     * A helper method to manually turn off the launcher and reset the sequence.
     * Useful for an emergency stop or when the driver deactivates the system.
     */
    public void stopLaunchSequence() {
        launcher.setShooterMotorVelocity(0);
        launcher.setIndexerServoPower(0.0);
        launcher.setStopPosition();
        launchState = LaunchState.IDLE;
    }
    public void configureVisionForTeleOp(CommonConstants.Alliance alliancecolor) {
        if (vision != null) {
            vision.setTargetingAlliance(alliancecolor);
            telemetry.addData("Vision", "Configured for %s Alliance", alliancecolor);
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

}
