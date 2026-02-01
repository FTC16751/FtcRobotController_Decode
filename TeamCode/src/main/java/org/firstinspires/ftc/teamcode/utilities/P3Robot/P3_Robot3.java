package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.utilities.Common.InterpolatingLookupTable;
import org.firstinspires.ftc.teamcode.utilities.Common.LedUtil;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;
import org.firstinspires.ftc.teamcode.utilities.Common.VisionUtil;

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
    //public final P3_TurretUtil_Velocity turret;
    public final P3_RubberBandIndexerUtil indexer;

    // ========================================
    // LAUNCH SEQUENCE STATE MACHINE
    // ========================================
    /**
     * States for the automated launch sequence.
     * IDLE:      The sequence is not running. Ready to start a new launch.
     * SPIN_UP:   Flywheels are accelerating to target speed.
     * FEEDING:   Indexer is running to push artifact into flywheels.
     * COOLDOWN:  A brief pause after a shot before returning to IDLE.
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        FEEDING,
        COOLDOWN
    }

    private LaunchState launchState = LaunchState.IDLE;
    private final ElapsedTime launchTimer = new ElapsedTime();

    // Launch sequence timing constants (in seconds)
    private static final double FEED_TIME_SECONDS = 0.25;       // Duration to run indexer per shot (reduced for speed)
    private static final double COOLDOWN_TIME_SECONDS = 0.05;   // Minimal pause between shots (reduced for rapid fire)
    private static final double SPIN_UP_TIMEOUT_SECONDS = 2.0;  // Safety timeout for flywheel acceleration

    // Configurable launch parameters
    private double velocityTolerancePercent = 0.97;  // Flywheels must reach 97% of target before feeding
    private double stallDetectionPercent = 0.80;     // If velocity drops below 80% during feeding, abort
    private boolean keepFlywheelsSpinning = true;    // Keep flywheels running between shots for faster follow-up

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
    private double currentTargetVelocity = 0.0;     // Target velocity for current launch sequence

    // Shot tracking
    private int shotsFired = 0;
    private int shotsAttempted = 0;
    private int shotsAborted = 0;

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
        RobotConfig config = RobotConfig.createP3Robot2Config();

        // Initialize all subsystems
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, config);
        intake = new P3_IntakeUtil(hardwareMap);
        launcher = new P3_LauncherUtil(hardwareMap);
        vision = new VisionUtil(hardwareMap, telemetry);
        imu = hardwareMap.get(IMU.class, "imu");
        led = new LedUtil(hardwareMap, "light");
        //turret = new P3_TurretUtil_Velocity(hardwareMap);
        indexer = new P3_RubberBandIndexerUtil(hardwareMap);

        // Set launcher to safe initial state
        launcher.setStopPosition();

        // Initialize flywheel velocity lookup table (distance in inches -> velocity in ticks/sec)
        // These values are empirically tuned for the P3 robot
        flywheelTable = new InterpolatingLookupTable();
        flywheelTable.add(30.0, 950.0);
        flywheelTable.add(40.0, 960.0);
        flywheelTable.add(50.0, 1080.0);
        flywheelTable.add(60.0, 1120.0);
        flywheelTable.add(70.0, 1080.0);   // 1180 - 100
        flywheelTable.add(80.0, 1120.0);   // 1220 - 100
        flywheelTable.add(90.0, 1220.0);   // 1320 - 100
        flywheelTable.add(100.0, 1300.0);  // 1400 - 100
        flywheelTable.add(110.0, 1340.0);  // 1440 - 100
        flywheelTable.add(120.0, 1380.0);  // 1480 - 100
        flywheelTable.add(130.0, 1420.0);  // 1520 - 100
        flywheelTable.add(140.0, 1460.0);  // 1560 - 100
        flywheelTable.add(150.0, 1500.0);  // 1600 - 100
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

        // Note: drive.update() and turret.update() are currently no-ops
        // Remove these calls if they remain empty, or implement if needed
        if (drive != null) {
            drive.update();
        }
//        if (turret != null) {
//            turret.update();
//        }
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
        //turret.emergencyStop();

        // Reset launch state
        launchState = LaunchState.IDLE;
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
     * 3. FEEDING -> COOLDOWN: After feeding for FEED_TIME_SECONDS
     * 4. COOLDOWN -> IDLE: After brief cooldown period
     *
     * @param shootCommand Set to true to initiate a launch (only checked in IDLE state)
     * @param targetVelocity Desired flywheel velocity in ticks per second
     * @return true when the sequence completes (one shot fired), false while busy
     */
    public boolean launchSequence(boolean shootCommand, double targetVelocity) {
        switch (launchState) {
            case IDLE:
                if (shootCommand) {
                    // Start a new launch sequence
                    currentTargetVelocity = targetVelocity;
                    launcher.setShooterMotorVelocity(targetVelocity);
                    launchTimer.reset();
                    launchState = LaunchState.SPIN_UP;
                    shotsAttempted++;
                }
                break;

            case SPIN_UP:
                // Continuously command the velocity to ensure it reaches target
                launcher.setShooterMotorVelocity(currentTargetVelocity);

                // Safety timeout check
                if (launchTimer.seconds() > SPIN_UP_TIMEOUT_SECONDS) {
                    telemetry.addData("⚠ ERROR", "Flywheel spin-up timeout - aborting launch");
                    shotsAborted++;
                    stopLaunchSequence();
                    break;
                }

                // Check if flywheels have reached target speed (within tolerance)
                if (launcher.getShooterMotorVelocity() >= currentTargetVelocity * velocityTolerancePercent) {
                    // Flywheels are ready - start feeding
                    //launcher.setIndexerServoPower(-1.0);  // Note: Negative is "feed" direction
                    launcher.setShootingPosition();
                    //intake.setIntakeServos();
                    launchTimer.reset();
                    launchState = LaunchState.FEEDING;
                }
                break;

            case FEEDING:
                // Emergency stall detection
                if (launcher.getShooterMotorVelocity() < currentTargetVelocity * stallDetectionPercent) {
                    telemetry.addData("⚠ ERROR", "Flywheel stall detected - aborting launch");
                    shotsAborted++;
                    stopLaunchSequence();
                    break;
                }

                // Run the indexer for a fixed duration to push one artifact
                if (launchTimer.seconds() > FEED_TIME_SECONDS) {
                    // Feeding complete - stop indexer and begin cooldown
                    //launcher.setIndexerServoPower(0.0);
                    //intake.stopIntakeServos();
                      launcher.setStopPosition();
                    // Optionally stop flywheels or keep them spinning for rapid follow-up
                    if (!keepFlywheelsSpinning) {
                        launcher.setShooterMotorVelocity(0);
                    }

                    shotsFired++;  // Successfully completed a shot
                    launchTimer.reset();
                    launchState = LaunchState.COOLDOWN;
                }
                // Continue feeding (redundant but explicit for clarity)
                else {
                    //launcher.setIndexerServoPower(-1.0);
                    launcher.setShootingPosition();
                }
                break;

            case COOLDOWN:
                // Brief pause to allow systems to settle before next shot
                if (launchTimer.seconds() > COOLDOWN_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    return true;  // Signal that one complete launch cycle has finished
                }
                break;
        }

        return false;  // Sequence still in progress
    }

    /**
     * Checks if the launch sequence is currently running.
     *
     * @return true if the state machine is not in IDLE, false otherwise
     */
    public boolean isLaunchSequenceBusy() {
        return launchState != LaunchState.IDLE;
    }

    /**
     * Immediately stops the launch sequence and resets to IDLE state.
     * Use this for emergency stops or when the driver deactivates the launcher.
     * Always stops flywheels regardless of keepFlywheelsSpinning setting.
     */
    public void stopLaunchSequence() {
        launcher.setShooterMotorVelocity(0);
        launcher.setIndexerServoPower(0.0);
        launcher.setStopPosition();
        launchState = LaunchState.IDLE;
        currentTargetVelocity = 0.0;
    }

    // ========================================
    // FLYWHEEL VELOCITY CALCULATION
    // ========================================
    /**
     * Gets the target flywheel velocity for a given distance using the lookup table.
     *
     * @param distanceInches Distance to target in inches
     * @return Target velocity in ticks per second
     */
    public double getTargetVelocityForDistance(double distanceInches) {
        return flywheelTable.get(distanceInches);
    }

    /**
     * Updates and returns the target flywheel velocity based on current vision data.
     *
     * If vision has a valid target:
     *   - Calculates velocity based on distance to target
     *   - Updates lastKnownGoodVelocity for future fallback
     *
     * If vision is unavailable:
     *   - Returns the last known good velocity (from when vision was last valid)
     *
     * @return Target flywheel velocity in ticks per second
     */
    public double updateAndGetTargetVelocity() {
        String dataSource;
        double newVelocity;

        if (vision.isTargetVisible()) {
            // Vision is active - calculate based on current distance
            dataSource = "VISION";
            double distanceInches = vision.getDistanceToTagInches();
            newVelocity = getTargetVelocityForDistance(distanceInches);

            // Store this as our fallback in case vision drops
            this.lastKnownGoodVelocity = newVelocity;

            telemetry.addData("Vision Distance", "%.1f in", distanceInches);
        } else {
            // Vision unavailable - use last known value
            dataSource = "LAST KNOWN";
            newVelocity = this.lastKnownGoodVelocity;
        }

        telemetry.addData("Target Velocity Source", dataSource);
        telemetry.addData("Target Velocity", "%.0f ticks/sec", newVelocity);

        return newVelocity;
    }

    // ========================================
    // VISION CONFIGURATION
    // ========================================
    /**
     * Configures vision system for TeleOp based on alliance color.
     * This determines which AprilTags the robot should target.
     *
     * @param allianceColor The alliance color (RED or BLUE)
     */
    public void configureVisionForTeleOp(CommonConstants.Alliance allianceColor) {
        if (vision != null) {
            vision.setTargetingAlliance(allianceColor);
            telemetry.addData("Vision", "Configured for %s Alliance", allianceColor);
        }
    }

    // ========================================
    // LED STATUS INDICATOR
    // ========================================
    /**
     * Updates LED color based on vision targeting status.
     *
     * Color meanings:
     * - OFF:    No target visible
     * - GREEN:  Aimed at target (within tolerance)
     * - YELLOW: Target visible, needs to turn right
     * - BLUE:   Target visible, needs to turn left
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
        final double AIMING_TOLERANCE_DEG = 2.0;

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
     * Sets the velocity tolerance for the launch sequence.
     * The flywheels must reach (targetVelocity * tolerance) before feeding begins.
     *
     * @param tolerance Percentage as decimal (e.g., 0.97 for 97%, 0.95 for 95%)
     */
    public void setVelocityTolerance(double tolerance) {
        if (tolerance > 0.0 && tolerance <= 1.0) {
            this.velocityTolerancePercent = tolerance;
        } else {
            telemetry.addData("⚠ Warning", "Invalid velocity tolerance: %.2f (must be 0.0-1.0)", tolerance);
        }
    }

    /**
     * Gets the current velocity tolerance setting.
     *
     * @return Current tolerance as decimal (e.g., 0.97 for 97%)
     */
    public double getVelocityTolerance() {
        return velocityTolerancePercent;
    }

    /**
     * Sets the stall detection threshold.
     * If velocity drops below (targetVelocity * threshold) during feeding, the launch aborts.
     *
     * @param threshold Percentage as decimal (e.g., 0.80 for 80%)
     */
    public void setStallDetectionThreshold(double threshold) {
        if (threshold > 0.0 && threshold <= 1.0) {
            this.stallDetectionPercent = threshold;
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
        return stallDetectionPercent;
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
        this.keepFlywheelsSpinning = keepSpinning;
    }

    /**
     * Gets the current flywheel coast-down setting.
     *
     * @return true if flywheels stay spinning between shots, false otherwise
     */
    public boolean isKeepFlywheelsSpinning() {
        return keepFlywheelsSpinning;
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
        return shotsFired;
    }

    /**
     * Gets the total number of launch attempts this match.
     * An attempt is counted when a launch sequence is initiated.
     *
     * @return Number of launch attempts
     */
    public int getShotsAttempted() {
        return shotsAttempted;
    }

    /**
     * Gets the total number of aborted launches this match.
     * A launch is aborted due to timeout or stall detection.
     *
     * @return Number of aborted shots
     */
    public int getShotsAborted() {
        return shotsAborted;
    }

    /**
     * Calculates launch success rate.
     *
     * @return Success rate as percentage (0.0 to 100.0), or 0.0 if no attempts
     */
    public double getLaunchSuccessRate() {
        if (shotsAttempted == 0) {
            return 0.0;
        }
        return (shotsFired * 100.0) / shotsAttempted;
    }

    /**
     * Resets all shot counters to zero.
     * Useful at the start of a new match.
     */
    public void resetShotCounters() {
        shotsFired = 0;
        shotsAttempted = 0;
        shotsAborted = 0;
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
        return launcher.getShooterMotorVelocity() >= targetVelocity * velocityTolerancePercent;
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
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Flywheel Velocity", "%.0f ticks/sec", launcher.getShooterMotorVelocity());

        // Shot statistics
        telemetry.addLine();
        telemetry.addData("Shots Fired", "%d / %d attempted", shotsFired, shotsAttempted);
        if (shotsAborted > 0) {
            telemetry.addData("⚠ Shots Aborted", shotsAborted);
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
        telemetry.addData("State", launchState);
        telemetry.addData("Timer", "%.2f sec", launchTimer.seconds());
        telemetry.addData("Target Velocity", "%.0f", currentTargetVelocity);
        telemetry.addData("Current Velocity", "%.0f", launcher.getShooterMotorVelocity());

        // Show velocity as percentage of target
        if (currentTargetVelocity > 0) {
            double percentOfTarget = (launcher.getShooterMotorVelocity() / currentTargetVelocity) * 100.0;
            telemetry.addData("Velocity %", "%.1f%%", percentOfTarget);
        }

        telemetry.addData("Indexer Power", "%.2f", launcher.getShooterMotorPower());
        telemetry.addData("Stopper Position", "%.2f", launcher.getStopperServoPosition());

        // Configuration info
        telemetry.addLine();
        telemetry.addData("Velocity Tolerance", "%.1f%%", velocityTolerancePercent * 100);
        telemetry.addData("Stall Threshold", "%.1f%%", stallDetectionPercent * 100);
        telemetry.addData("Keep Spinning", keepFlywheelsSpinning ? "YES" : "NO");
    }

    /**
     * Adds comprehensive match statistics telemetry.
     * Shows detailed shot tracking and performance metrics.
     */
    public void addMatchStatsTelemetry() {
        telemetry.addLine("=== Match Statistics ===");
        telemetry.addData("Total Attempts", shotsAttempted);
        telemetry.addData("Successful Shots", shotsFired);
        telemetry.addData("Aborted Shots", shotsAborted);
        telemetry.addData("Success Rate", "%.1f%%", getLaunchSuccessRate());

        // Calculate some derived stats
        if (shotsAttempted > 0) {
            telemetry.addData("Abort Rate", "%.1f%%", (shotsAborted * 100.0) / shotsAttempted);
        }
    }
}