package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

/**
 * Production-ready Turret Control System for FTC
 *
 * Hardware Setup:
 * - goBILDA 5-turn super speed servo (2000 series)
 * - 4:1 gear reduction (50T driving 200T)
 * - REV Magnetic Limit Switch (active-low)
 * - Total turret range: 450° (5 turns ÷ 4)
 * - Usable range: ±160° from home (320° total)
 *
 * Features:
 * - Automatic homing on init using magnetic switch
 * - Soft limits to protect wiring
 * - Manual control with joystick
 * - Auto-aim assist with Limelight
 * - Snap-to-home button
 * - No position snap between Auto→TeleOp
 */
public class Turret {

    // Hardware
    private final ServoImplEx servo;
    private final DigitalChannel magSwitch;

    // Constants - Mechanical
    private static final double SERVO_TURNS = 5.0;  // 5-turn servo
    private static final double GEAR_RATIO = 4.0;   // 50T:200T = 1:4
    private static final double TURRET_DEG_PER_SERVO_UNIT = (SERVO_TURNS * 360.0) / GEAR_RATIO; // 450°

    // Constants - Limits
    private static final double SOFT_LIMIT_DEG = 128.0;  // ±158° (2° buffer from ±160°)
    private static final double SOFT_LIMIT_SERVO_UNITS = SOFT_LIMIT_DEG / TURRET_DEG_PER_SERVO_UNIT; // 0.3511

    // Constants - Homing (IMPROVED)
    private static final double HOME_STEP = 0.0015;  // Slower step for more precise homing
    private static final double HOME_FAST_STEP = 0.005;  // Faster step for initial approach
    private static final double HOME_TARGET = 0.6611;  // Expected home position based on your calibration
    private static final double HOME_TOLERANCE = 0.15;  // How close to approach before slowing down

    // Constants - Control
    private static final double MANUAL_SPEED = 0.005;  // Manual joystick speed multiplier
    private static final double LIMELIGHT_GAIN = 0.00005;  // P-loop gain for auto-aim (20-25% correction per loop)
    private static final double LIMELIGHT_DEADBAND = 0.7;  // degrees - ignore tiny errors

    // State
    private double currentPos;      // Current commanded servo position (0.0-1.0)
    private Double homePos;         // Servo position when magnet triggers (null until homed)
    private boolean isHomed;        // True after successful homing

    // Homing state
    private enum HomingPhase { INITIAL, FAST_APPROACH, SLOW_APPROACH, BACKING_OFF, FINAL_APPROACH, COMPLETE }
    private HomingPhase homingPhase = HomingPhase.INITIAL;
    private int homingStableCount = 0;  // Count stable switch readings

    // Debug tracking
    private int manualCallCount = 0;     // Track how many times moveManual is called
    private double lastManualInput = 0;  // Last joystick value received

    /**
     * Constructor
     * @param servo The turret servo (must be ServoImplEx for PWM control)
     * @param magSwitch REV Magnetic Limit Switch (active-low)
     */
    public Turret(ServoImplEx servo, DigitalChannel magSwitch) {
        this.servo = servo;
        this.magSwitch = magSwitch;
        this.magSwitch.setMode(DigitalChannel.Mode.INPUT);

        // CRITICAL: Set PWM range for full 5-turn travel
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Seed from last commanded position to avoid snap between OpModes
        this.currentPos = servo.getPosition();
        servo.setPosition(currentPos);

        this.isHomed = false;
    }

    // ========== HOMING ==========

    /**
     * Check if magnetic limit switch is triggered (active-low)
     * @return true when magnet is detected
     */
    public boolean isHomeSwitchTriggered() {
        return !magSwitch.getState();  // Active-low: triggered = false
    }

    /**
     * Execute one step of the IMPROVED homing routine
     * This version:
     * - Intelligently chooses direction based on current position
     * - Moves fast far from home, slow when close
     * - Handles overshoot by backing off and approaching slowly
     *
     * Call this repeatedly in init_loop() until it returns true
     *
     * @return true when homing is complete
     */
    public boolean updateHoming() {
        // Already homed - nothing to do
        if (isHomed) {
            return true;
        }

        boolean switchTriggered = isHomeSwitchTriggered();

        switch (homingPhase) {
            case INITIAL:
                // Determine which direction to go based on current position
                // Home is expected around 0.6611
                if (switchTriggered) {
                    // Already at home!
                    homingPhase = HomingPhase.BACKING_OFF;
                } else {
                    homingPhase = HomingPhase.FAST_APPROACH;
                }
                break;

            case FAST_APPROACH:
                // Move quickly toward expected home position
                double distanceToHome = HOME_TARGET - currentPos;

                if (switchTriggered) {
                    // Hit the switch - back off slightly
                    homingPhase = HomingPhase.BACKING_OFF;
                    homingStableCount = 0;
                } else if (Math.abs(distanceToHome) < HOME_TOLERANCE) {
                    // Getting close - slow down
                    homingPhase = HomingPhase.SLOW_APPROACH;
                } else {
                    // Move toward home at fast speed
                    double direction = Math.signum(distanceToHome);
                    currentPos = Range.clip(currentPos + (direction * HOME_FAST_STEP), 0.0, 1.0);
                    servo.setPosition(currentPos);
                }
                break;

            case SLOW_APPROACH:
                // Move slowly toward expected home position
                distanceToHome = HOME_TARGET - currentPos;

                if (switchTriggered) {
                    // Hit the switch - back off slightly
                    homingPhase = HomingPhase.BACKING_OFF;
                    homingStableCount = 0;
                } else {
                    // Move toward home at slow speed
                    double direction = Math.signum(distanceToHome);
                    currentPos = Range.clip(currentPos + (direction * HOME_STEP), 0.0, 1.0);
                    servo.setPosition(currentPos);
                }
                break;

            case BACKING_OFF:
                // Back off from the switch to find the edge precisely
                if (!switchTriggered) {
                    // Switch released - now approach slowly to find exact trigger point
                    homingPhase = HomingPhase.FINAL_APPROACH;
                    homingStableCount = 0;
                } else {
                    // Keep backing off (away from HOME_TARGET)
                    double direction = -Math.signum(HOME_TARGET - currentPos);
                    currentPos = Range.clip(currentPos + (direction * HOME_STEP), 0.0, 1.0);
                    servo.setPosition(currentPos);
                }
                break;

            case FINAL_APPROACH:
                // Slowly approach until switch triggers consistently
                if (switchTriggered) {
                    homingStableCount++;
                    if (homingStableCount >= 3) {
                        // Switch has been triggered for 3 consecutive readings - we're homed!
                        homePos = currentPos;
                        isHomed = true;
                        homingPhase = HomingPhase.COMPLETE;
                        return true;
                    }
                } else {
                    // Switch not triggered - keep approaching
                    homingStableCount = 0;
                    double direction = Math.signum(HOME_TARGET - currentPos);
                    currentPos = Range.clip(currentPos + (direction * HOME_STEP), 0.0, 1.0);
                    servo.setPosition(currentPos);
                }
                break;

            case COMPLETE:
                return true;
        }

        return false;
    }

    /**
     * Get current homing phase for debugging
     */
    public String getHomingPhase() {
        return homingPhase.toString();
    }

    /**
     * Check if turret has been homed
     */
    public boolean isHomed() {
        return isHomed;
    }

    /**
     * Get the servo position where home was detected
     * @return homePos or null if not homed yet
     */
    public Double getHomePosition() {
        return homePos;
    }

    // ========== POSITION CONTROL ==========

    /**
     * Set turret to a specific angle in degrees
     * 0° = front of robot (home position)
     * Positive = right rotation (clockwise when viewed from above)
     * Negative = left rotation (counter-clockwise when viewed from above)
     *
     * @param degrees Target angle (-158° to +158°)
     */
    public void setAngleDegrees(double degrees) {
        // Refuse to move if not homed
        if (!isHomed) {
            return;
        }

        // Clamp to soft limits
        degrees = Range.clip(degrees, -SOFT_LIMIT_DEG, SOFT_LIMIT_DEG);

        // Convert degrees to servo position
        // Mechanical reality: higher position = counter-clockwise, lower = clockwise
        // Convention: positive degrees = clockwise (right)
        // Therefore: positive degrees needs LOWER servo position
        double targetPos = homePos - (degrees / TURRET_DEG_PER_SERVO_UNIT);

        // Apply position-based soft limits (double protection)
        double minPos = homePos - SOFT_LIMIT_SERVO_UNITS;
        double maxPos = homePos + SOFT_LIMIT_SERVO_UNITS;
        currentPos = Range.clip(targetPos, minPos, maxPos);

        servo.setPosition(currentPos);
    }

    /**
     * Snap turret back to home position (0°)
     */
    public void snapToHome() {
        //if (isHomed) {
            currentPos = homePos;
            servo.setPosition(currentPos);
       // }
    }

    /**
     * Get current turret angle in degrees (relative to home)
     * @return Current angle or null if not homed
     */
    public Double getCurrentAngleDegrees() {
        if (!isHomed) {
            return null;
        }
        // Mechanical reality: higher position = counter-clockwise = negative angle
        // Lower position = clockwise = positive angle
        return (homePos - currentPos) * TURRET_DEG_PER_SERVO_UNIT;
    }

    // ========== MANUAL CONTROL ==========

    /**
     * Move turret based on joystick input
     * Respects soft limits
     *
     * @param joystickValue Joystick input (-1.0 to +1.0)
     *                      Positive = right (clockwise), Negative = left (counter-clockwise)
     */
    public void moveManual(double joystickValue) {
        // Debug tracking
        manualCallCount++;
        lastManualInput = joystickValue;

        // Refuse to move if not homed
        if (!isHomed) {
            return;
        }

        // Apply deadband - ignore tiny joystick drift
        if (Math.abs(joystickValue) < 0.05) {
            return;
        }

        // Calculate new position
        // User expectation: stick right = turret moves right (clockwise)
        // Mechanical reality: lower position = clockwise
        // Therefore: stick right (positive) should DECREASE position
        double adjustment = -joystickValue * MANUAL_SPEED;
        double targetPos = currentPos + adjustment;

        // Apply soft limits to prevent damage
        double minPos = homePos - SOFT_LIMIT_SERVO_UNITS;
        double maxPos = homePos + SOFT_LIMIT_SERVO_UNITS;
        currentPos = Range.clip(targetPos, minPos, maxPos);

        servo.setPosition(currentPos);
    }

    // ========== AUTO-AIM (LIMELIGHT) ==========

    /**
     * Auto-aim using Limelight horizontal offset
     * Uses simple P-loop with deadband and rate limiting
     *
     * @param tx Limelight horizontal offset in degrees (positive = target is right)
     * @param targetValid True if Limelight sees a valid target
     */
    public void autoAim(double tx, boolean targetValid) {
        // Refuse to move if not homed or no target
        if (!isHomed || !targetValid) {
            return;
        }

        // Apply deadband - don't chase tiny errors
        if (Math.abs(tx) < LIMELIGHT_DEADBAND) {
            return;
        }

        // Calculate adjustment (P-loop with rate limiting)
        // tx is the error in degrees
        // LIMELIGHT_GAIN controls how aggressively we correct (should be 0.0004-0.0006)
        // Positive tx (target right) = need to turn clockwise = lower servo position
        double adjustment = -tx * LIMELIGHT_GAIN;

        // CRITICAL: Rate limit to prevent overshoot
        // Limit max movement per loop to ~4.5° (0.01 servo units)
        adjustment = Range.clip(adjustment, -0.001, 0.001);

        double targetPos = currentPos + adjustment;

        // Apply soft limits
        double minPos = homePos - SOFT_LIMIT_SERVO_UNITS;
        double maxPos = homePos + SOFT_LIMIT_SERVO_UNITS;
        currentPos = Range.clip(targetPos, minPos, maxPos);

        servo.setPosition(currentPos);
    }

    // ========== TELEMETRY HELPERS ==========

    /**
     * Get formatted status string for telemetry
     */
    public String getStatusString() {
        if (!isHomed) {
            return "NOT HOMED - Run homing in init!";
        }

        Double angle = getCurrentAngleDegrees();
        return String.format("%.1f° (pos: %.3f)", angle, currentPos);
    }

    /**
     * Get formatted soft limit status for telemetry
     */
    public String getLimitStatus() {
        if (!isHomed) {
            return "N/A";
        }

        double minPos = homePos - SOFT_LIMIT_SERVO_UNITS;
        double maxPos = homePos + SOFT_LIMIT_SERVO_UNITS;

        return String.format("Min: %.3f | Max: %.3f | Current: %.3f", minPos, maxPos, currentPos);
    }

    /**
     * Check if home position is in safe range for full ±160° travel
     * Home should be between 0.356 and 0.644 for full range
     */
    public boolean isHomePositionSafe() {
        if (!isHomed) {
            return false;
        }
        return homePos >= 0.356 && homePos <= 0.644;
    }

    // ========== DEBUG METHODS ==========

    /**
     * Get number of times moveManual has been called (for debugging)
     */
    public int getManualCallCount() {
        return manualCallCount;
    }

    /**
     * Get last joystick input received (for debugging)
     */
    public double getLastManualInput() {
        return lastManualInput;
    }

    /**
     * Get current commanded servo position (for debugging)
     */
    public double getCurrentPosition() {
        return currentPos;
    }

    /**
     * Manually set home position (bypasses homing process)
     * ⚠️ FOR DEBUGGING ONLY - normal operation should use updateHoming()
     *
     * @param position Servo position to use as home (should be 0.356-0.644)
     */
    public void setHomePositionManual(double position) {
        homePos = position;
        isHomed = true;
        homingPhase = HomingPhase.COMPLETE;
    }
}