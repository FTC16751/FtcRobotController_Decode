package org.firstinspires.ftc.teamcode.teams.p3;

import org.firstinspires.ftc.teamcode.teams.p3.subsystems.P3_IntakeUtil;
import org.firstinspires.ftc.teamcode.teams.p3.subsystems.P3_LauncherUtil;
import org.firstinspires.ftc.teamcode.teams.p3.subsystems.P3_RubberBandIndexerUtil;
import org.firstinspires.ftc.teamcode.teams.p3.subsystems.Turret;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
 * P3_Robot3 is the central hub that orchestrates all of the P3 robot's subsystems.
 * It owns all the hardware and utility classes, providing a clean interface for OpModes.
 *
 * This class manages the robot-level launch sequence state machine, coordinating
 * flywheel spin-up, indexer feeding, and timing between shots.
 */
public class P3_Robot3 {

    // ========================================
    // PUBLIC SUBSYSTEMS
    // ========================================
    // These are public so the OpMode can access them directly (e.g., robot.drive.arcadeDrive(...))
    public final DriveUtil2026b drive;
    public final P3_IntakeUtil intake;
    public final P3_LauncherUtil launcher;
    public final VisionUtil vision;
    public final Telemetry telemetry;
    public final IMU imu;
    public final LedUtil led;
    public final Turret turret;  // NEW: Turret subsystem
    public final P3_RubberBandIndexerUtil indexer;

    // ========================================
    // LAUNCH SEQUENCE
    // ========================================
    /**
     * The spin-up / ready / feed / cooldown sequence lives in common.LaunchController and is shared
     * by every robot. This class only supplies the two P3-specific pieces: how to drive the
     * flywheel (P3_LauncherUtil) and what "feed" means (the rubber-band indexer).
     * Timings and thresholds are set in the constructor.
     */
    public final LaunchController launchController;

    // ========================================
    // FLYWHEEL VELOCITY MANAGEMENT
    // ========================================
    private final InterpolatingLookupTable flywheelTable;

    /**
     * Stores the last successfully calculated target velocity.
     * Used as a fallback when vision is unavailable.
     * Initialized to a safe mid-range value rather than 0.
     */
    private double lastKnownGoodVelocity = 1000.0;  // Default to reasonable mid-range velocity

    // ========================================
    // CONSTRUCTOR
    // ========================================
    /**
     * Initializes all robot subsystems and configures the flywheel lookup table.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param telemetry The telemetry object for displaying data
     */
    public P3_Robot3(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize robot configuration
        RobotConfig config = P3Bot3Config.create();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, config);
        intake = new P3_IntakeUtil(hardwareMap);
        launcher = new P3_LauncherUtil(hardwareMap);
        vision = new VisionUtil(hardwareMap, telemetry, config.hardware.limelight);
        imu = hardwareMap.get(IMU.class, "imu");
        led = new LedUtil(hardwareMap, config.hardware.led);

        // Initialize Turret subsystem
        ServoImplEx turretServo = hardwareMap.get(ServoImplEx.class, "turret");
        DigitalChannel magSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
        turret = new Turret(turretServo, magSwitch);

        indexer = new P3_RubberBandIndexerUtil(hardwareMap);

        // Shared launch sequence with the two P3-specific adapters.
        // Settings are the values this class used before 2026-09-07 (feed 1.5 s, cooldown 0.05 s,
        // 2.0 s spin-up timeout, feed at 97% of target, abort below 80%, keep spinning between shots).
        Flywheel p3Flywheel = new Flywheel() {
            @Override public void setVelocity(double v) { launcher.setShooterMotorVelocity(v); }
            @Override public double getVelocity()       { return launcher.getShooterMotorVelocity(); }
        };
        Feeder p3Feeder = new Feeder() {
            @Override public void start() { indexer.start(); }
            @Override public void stop()  { indexer.stop(); launcher.setIndexerServoPower(0); }
        };
        launchController = new LaunchController(p3Flywheel, p3Feeder,
                new LaunchController.Settings()
                        .feedTimeSec(1.5)
                        .cooldownSec(0.05)
                        .spinUpTimeoutSec(2.0)
                        .readyFraction(0.97)
                        .stallFraction(0.80)
                        .keepSpinning(true),
                telemetry);

        // Set launcher to safe initial state
        launcher.setStopPosition();

        // Initialize flywheel velocity lookup table (distance in inches -> velocity in ticks/sec)
        // These values are empirically tuned for the P3 robot
        flywheelTable = new InterpolatingLookupTable();
        flywheelTable.add(30.0, 950.0*1.10);
        flywheelTable.add(40.0, 960.0*1.10);
        flywheelTable.add(50.0, 1080.0*1.10);
        flywheelTable.add(60.0, 1120.0*1.10);
        flywheelTable.add(70.0, 1080.0*1.10);   // 1180 - 100
        flywheelTable.add(80.0, 1120.0*1.10);   // 1220 - 100
        flywheelTable.add(90.0, 1220.0*1.10);   // 1320 - 100
        flywheelTable.add(100.0, 1300.0*1.10);  // 1400 - 100
        flywheelTable.add(110.0, 1340.0*1.10);  // 1440 - 100
        flywheelTable.add(120.0, 1380.0*1.10);  // 1480 - 100
        flywheelTable.add(130.0, 1420.0*1.10);  // 1520 - 100
        flywheelTable.add(140.0, 1460.0*1.10);  // 1560 - 100
        flywheelTable.add(150.0, 1500.0*1.10);  // 1600 - 100
    }

    // ========================================
    // PERIODIC UPDATE
    // ========================================
    /**
     * The main periodic update method for the robot.
     * This MUST be called in every iteration of the OpMode's loop().
     *
     * Updates all subsystems that require continuous processing.
     */
    public void update() {
        vision.update();
        updateLedStatus();

        // Note: drive.update() is currently a no-op
        if (drive != null) {
            drive.update();
        }
    }

    // ========================================
    // SHUTDOWN
    // ========================================
    /**
     * Stops all motors and mechanisms on the robot.
     * Call this in the OpMode's stop() method for safe shutdown.
     */
    public void stopAll() {
        drive.stopRobot();
        intake.setIntakePower(0);
        launcher.setShooterMotorVelocity(0);
        launcher.setIndexerServoPower(0);
        launcher.setStopPosition();
        vision.stop();

        // Reset launch state
        launchController.stop();
    }

    // ========================================
    // LAUNCH SEQUENCE CONTROL
    // ========================================
    /**
     * Manages the automated launch sequence state machine.
     * This method should be called every loop iteration when a launch is desired.
     *
     * The sequence progresses through these states:
     * 1. IDLE -> SPIN_UP: When shootCommand is true, start spinning flywheels
     * 2. SPIN_UP -> FEEDING: When flywheels reach target speed (within tolerance)
     * 3. FEEDING -> COOLDOWN: After feeding for settings.feedTimeSec
     * 4. COOLDOWN -> IDLE: After brief cooldown period
     *
     * @param shootCommand Set to true to initiate a launch (only checked in IDLE state)
     * @param targetVelocity Desired flywheel velocity in ticks per second
     * @return true when the sequence completes (one shot fired), false while busy
     */
    public boolean launchSequence(boolean shootCommand, double targetVelocity) {
        return launchController.update(shootCommand, targetVelocity);
    }

    /**
     * Immediately stops the current launch sequence and resets to IDLE state.
     * Stops flywheels and indexer.
     */
    public void stopLaunchSequence() {
        launchController.stop();
    }

    /**
     * Checks if a launch sequence is currently active.
     *
     * @return true if in SPIN_UP, FEEDING, or COOLDOWN state
     */
    public boolean isLaunchSequenceBusy() {
        return launchController.isBusy();
    }

    /**
     * Gets the current launch state for debugging.
     *
     * @return Current LaunchState
     */
    public String getLaunchStateString() {
        return launchController.getState().toString();
    }

    // ========================================
    // VISION-BASED VELOCITY CALCULATION
    // ========================================
    /**
     * Updates and returns the target velocity based on current vision data.
     * Uses the interpolating lookup table to convert distance to velocity.
     *
     * Behavior:
     * - If target visible: Calculate velocity from distance, cache it as "last known good"
     * - If target not visible: Return last known good velocity (or default if never seen target)
     *
     * This allows the robot to maintain shooting capability even when target is temporarily obscured.
     *
     * @return Target velocity in ticks per second
     */
    public double updateAndGetTargetVelocity() {
        if (vision.isTargetVisible()) {
            // Target visible - calculate velocity from distance
            double distanceInches = vision.getDistanceToTagInches();
            lastKnownGoodVelocity = flywheelTable.get(distanceInches);
            return lastKnownGoodVelocity;
        } else {
            // Target not visible - use last known good velocity
            return lastKnownGoodVelocity;
        }
    }

    /**
     * Gets the last successfully calculated target velocity.
     * This is the velocity that will be used if the target is not currently visible.
     *
     * @return Last known good velocity in ticks per second
     */
    public double getLastKnownGoodVelocity() {
        return lastKnownGoodVelocity;
    }

    // ========================================
    // VISION CONFIGURATION
    // ========================================
    /**
     * Configures vision system for TeleOp based on alliance color.
     * Sets up Limelight pipeline and detection priorities.
     *
     * @param alliance RED or BLUE alliance
     */
    public void configureVisionForTeleOp(CommonConstants.Alliance alliance) {
        // Configure Limelight for appropriate AprilTag detection
        // This would set pipeline based on alliance if needed
        // For now, just ensure vision is ready
        vision.update();
    }

    // ========================================
    // LED STATUS MANAGEMENT
    // ========================================
    /**
     * Updates LED color based on robot state.
     * Call this in the periodic update() method.
     */
    private void updateLedStatus() {

        if (led == null) {
            return;
        }

        if (!vision.isTargetVisible()) {
            led.setColor(LedUtil.Color.OFF);
            return;
        }

        double headingError = vision.getTargetAngleX();
        final double AIMING_TOLERANCE_DEG = 4.0;

        if (Math.abs(headingError) <= AIMING_TOLERANCE_DEG) {
            led.setColor(LedUtil.Color.GREEN);  // On target
        } else if (headingError > AIMING_TOLERANCE_DEG) {
            led.setColor(LedUtil.Color.YELLOW); // Turn right
        } else {
            led.setColor(LedUtil.Color.BLUE);   // Turn left
        }
    }

    // ========================================
    // CONFIGURATION METHODS
    // ========================================

    /**
     * Sets the velocity tolerance threshold.
     * Flywheels must reach (targetVelocity * tolerance) before feeding begins.
     *
     * Example: tolerance of 0.97 means flywheels must reach 97% of target speed
     *
     * @param tolerance Decimal between 0.0 and 1.0 (e.g., 0.97 for 97%)
     */
    public void setVelocityTolerance(double tolerance) {
        if (tolerance > 0.0 && tolerance <= 1.0) {
            launchController.settings.readyFraction = tolerance;
        } else {
            telemetry.addData("⚠ Warning", "Invalid velocity tolerance: %.2f (must be 0.0-1.0)", tolerance);
        }
    }

    /**
     * Gets the current velocity tolerance threshold.
     *
     * @return Current tolerance as decimal (e.g., 0.97 for 97%)
     */
    public double getVelocityTolerance() {
        return launchController.settings.readyFraction;
    }

    /**
     * Sets the stall detection threshold.
     * If velocity drops below (targetVelocity * threshold) during feeding, the launch is aborted.
     *
     * Example: threshold of 0.80 means launch aborts if velocity drops below 80%
     *
     * @param threshold Decimal between 0.0 and 1.0 (e.g., 0.80 for 80%)
     */
    public void setStallDetectionThreshold(double threshold) {
        if (threshold > 0.0 && threshold <= 1.0) {
            launchController.settings.stallFraction = threshold;
        } else {
            telemetry.addData("⚠ Warning", "Invalid stall threshold: %.2f (must be 0.0-1.0)", threshold);
        }
    }

    /**
     * Gets the current stall detection threshold.
     *
     * @return Current threshold as decimal (e.g., 0.80 for 80%)
     */
    public double getStallDetectionThreshold() {
        return launchController.settings.stallFraction;
    }

    /**
     * Sets whether flywheels should keep spinning between shots.
     *
     * true:  Flywheels stay at speed for rapid follow-up shots (faster but more power draw)
     * false: Flywheels stop after each shot (slower but conserves battery)
     *
     * @param keepSpinning true to maintain flywheel speed, false to stop after each shot
     */
    public void setKeepFlywheelsSpinning(boolean keepSpinning) {
        launchController.settings.keepSpinning = keepSpinning;
    }

    /**
     * Gets the current flywheel coast-down setting.
     *
     * @return true if flywheels stay spinning between shots, false otherwise
     */
    public boolean isKeepFlywheelsSpinning() {
        return launchController.settings.keepSpinning;
    }

    // ========================================
    // SHOT TRACKING METHODS
    // ========================================

    /**
     * Gets the total number of shots successfully fired this match.
     * A shot is counted when the feeding phase completes successfully.
     *
     * @return Number of completed shots
     */
    public int getShotsFired() {
        return launchController.getShotsFired();
    }

    /**
     * Gets the total number of launch attempts this match.
     * An attempt is counted when a launch sequence is initiated.
     *
     * @return Number of launch attempts
     */
    public int getShotsAttempted() {
        return launchController.getShotsAttempted();
    }

    /**
     * Gets the total number of aborted launches this match.
     * A launch is aborted due to timeout or stall detection.
     *
     * @return Number of aborted shots
     */
    public int getShotsAborted() {
        return launchController.getShotsAborted();
    }

    /**
     * Calculates launch success rate.
     *
     * @return Success rate as percentage (0.0 to 100.0), or 0.0 if no attempts
     */
    public double getLaunchSuccessRate() {
        int attempted = launchController.getShotsAttempted();
        if (attempted == 0) {
            return 0.0;
        }
        return (launchController.getShotsFired() * 100.0) / attempted;
    }

    /**
     * Resets all shot counters to zero.
     * Useful at the start of a new match.
     */
    public void resetShotCounters() {
        launchController.resetShotCounters();
    }

    // ========================================
    // MANUAL FLYWHEEL CONTROL
    // ========================================

    /**
     * Manually spins up flywheels without starting the full launch sequence.
     * Useful for pre-spinning before a shot or testing flywheel performance.
     *
     * Note: This bypasses the launch state machine. To stop, call stopFlywheels()
     * or use the normal stopLaunchSequence().
     *
     * @param targetVelocity Desired velocity in ticks per second
     */
    public void manualSpinUpFlywheels(double targetVelocity) {
        launcher.setShooterMotorVelocity(targetVelocity);
    }

    /**
     * Manually stops the flywheels.
     * Use this to stop flywheels that were started with manualSpinUpFlywheels().
     */
    public void stopFlywheels() {
        launcher.setShooterMotorVelocity(0);
    }

    /**
     * Checks if flywheels are at the target velocity.
     *
     * @param targetVelocity The target velocity to check against
     * @return true if current velocity >= target * velocityTolerance
     */
    public boolean areFlywheelsReady(double targetVelocity) {
        return launchController.isFlywheelReady(targetVelocity);
    }

    /**
     * Gets the current actual flywheel velocity.
     *
     * @return Current velocity in ticks per second
     */
    public double getCurrentFlywheelVelocity() {
        return launcher.getShooterMotorVelocity();
    }

    // ========================================
    // TELEMETRY
    // ========================================
    /**
     * Adds consolidated robot telemetry to the display.
     * Call this at the end of your OpMode loop for debugging info.
     */
    public void addTelemetry() {
        telemetry.addLine("=== P3 Robot Status ===");
        telemetry.addData("Launch State", launchController.getState());
        telemetry.addData("Flywheel Velocity", "%.0f ticks/sec", launcher.getShooterMotorVelocity());

        // Shot statistics
        telemetry.addLine();
        telemetry.addData("Shots Fired", "%d / %d attempted", launchController.getShotsFired(), launchController.getShotsAttempted());
        if (launchController.getShotsAborted() > 0) {
            telemetry.addData("⚠ Shots Aborted", launchController.getShotsAborted());
        }
        telemetry.addData("Success Rate", "%.1f%%", getLaunchSuccessRate());

        // Add subsystem telemetry
        vision.addTelemetry();

        // Add any additional robot-level metrics here
    }

    /**
     * Adds detailed launch sequence telemetry for debugging.
     * Useful during testing and tuning.
     */
    public void addLaunchDebugTelemetry() {
        telemetry.addLine("--- Launch Debug ---");
        telemetry.addData("State", launchController.getState());
        telemetry.addData("Target Velocity", "%.0f", launchController.getTargetVelocity());
        telemetry.addData("Current Velocity", "%.0f", launcher.getShooterMotorVelocity());

        // Show velocity as percentage of target
        if (launchController.getTargetVelocity() > 0) {
            double percentOfTarget = (launcher.getShooterMotorVelocity() / launchController.getTargetVelocity()) * 100.0;
            telemetry.addData("Velocity %", "%.1f%%", percentOfTarget);
        }

        telemetry.addData("Indexer Power", "%.2f", launcher.getShooterMotorPower());
        telemetry.addData("Stopper Position", "%.2f", launcher.getStopperServoPosition());

        // Configuration info
        telemetry.addLine();
        telemetry.addData("Velocity Tolerance", "%.1f%%", launchController.settings.readyFraction * 100);
        telemetry.addData("Stall Threshold", "%.1f%%", launchController.settings.stallFraction * 100);
        telemetry.addData("Keep Spinning", launchController.settings.keepSpinning ? "YES" : "NO");
    }

    /**
     * Adds comprehensive match statistics telemetry.
     * Shows detailed shot tracking and performance metrics.
     */
    public void addMatchStatsTelemetry() {
        telemetry.addLine("=== Match Statistics ===");
        telemetry.addData("Total Attempts", launchController.getShotsAttempted());
        telemetry.addData("Successful Shots", launchController.getShotsFired());
        telemetry.addData("Aborted Shots", launchController.getShotsAborted());
        telemetry.addData("Success Rate", "%.1f%%", getLaunchSuccessRate());

        // Calculate some derived stats
        if (launchController.getShotsAttempted() > 0) {
            telemetry.addData("Abort Rate", "%.1f%%", (launchController.getShotsAborted() * 100.0) / launchController.getShotsAttempted());
        }
    }
}