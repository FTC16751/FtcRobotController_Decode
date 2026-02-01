package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Velocity-controlled turret utility with improved safety and mode handling.
 *
 * Strategy:
 * - Motor runs in RUN_USING_ENCODER with velocity control
 * - Outer loop converts angle error (deg) → commanded velocity (deg/sec)
 * - Manual override temporarily switches to RUN_WITHOUT_ENCODER
 * - Smooth transitions between manual and automatic control
 *
 * Hardware:
 * - AndyMark 5.5in Turntable Assembly
 * - 200-tooth turntable gear (am-robits-200t)
 * - 50-tooth driving gear (am-5020_50)
 * - GoBilda 312 RPM motor (19.2:1, 537.7 ticks/rev)
 */
public class P3_TurretUtil_Velocity {

    // --- Hardware ---
    private final DcMotorEx turretMotor;

    // --- Mechanics ---
    private static final double TICKS_PER_MOTOR_REV = 537.7;
    private static final double GEAR_RATIO = 200.0 / 50.0;
    private static final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;
    private static final double TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0;

    // --- Limits (degrees) ---
    public static final double MAX_ANGLE_CW = 100.0;
    public static final double MAX_ANGLE_CCW = -180.0;

    // --- Targeting / motion ---
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;

    // Outer loop tuning: error → velocity
    // TUNE THESE for your turret's responsiveness
    private static final double K_POS_TO_VEL = 6.0;              // (deg/sec) per deg error - increase for faster response
    private static final double MAX_VEL_DEG_PER_SEC = 200.0;     // max turret speed cap
    private static final double MIN_VEL_DEG_PER_SEC = 20.0;      // overcome stiction/friction
    private static final double MIN_MOVE_ERROR_DEG = 3.0;        // don't force min velocity when very close

    // --- Inner loop (motor velocity PIDF) ---
    // F ≈ 32767 / maxTicksPerSecond
    // For 312RPM motor: max ~537.7 ticks/rev * (312/60) rev/s = ~2795 ticks/s → F ≈ 11.7
    // TUNE THESE if motor doesn't track velocity well
    private static final PIDFCoefficients VEL_PIDF = new PIDFCoefficients(0.230, 0.2, 0.8, 14.5);

    // --- Manual override ---
    private static final double JOYSTICK_DEADZONE = 0.10;
    private static final double MANUAL_MAX_POWER = 0.80;
    private boolean manualOverrideActive = false;

    // --- Safety / diagnostics ---
    private boolean isCalibrated = false;
    private boolean limitWarning = false;

    // --- Target state ---
    private double targetAngleDeg = 0.0;

    // --- Velocity estimate (deg/sec) ---
    private long lastUpdateTimeMs = 0;
    private double lastAngleDeg = 0.0;
    private double currentVelocityDegPerSec = 0.0;

    // --- Current monitoring ---
    private static final double NORMAL_CURRENT_THRESHOLD = 3.0;
    private static final double JAM_CURRENT_THRESHOLD = 4.0;
    private static final double MIN_VELOCITY_THRESHOLD = 5.0;
    private static final double SLIP_DETECTION_TIME = 0.5;
    private static final double JAM_AUTO_STOP_TIME = 2.0;

    private double currentDrawA = 0.0;
    private long highCurrentStartTimeMs = 0;
    private boolean jamDetected = false;
    private boolean slipDetected = false;
    private boolean jamAutoStopTriggered = false;

    /**
     * Constructor - initializes turret in velocity control mode.
     */
    public P3_TurretUtil_Velocity(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");

        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder - physically align turret to front before init!
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set up velocity control
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, VEL_PIDF);
        //turretMotor.setPower(1.0); // Full power for velocity control

        targetAngleDeg = 0.0;
        lastUpdateTimeMs = System.currentTimeMillis();
        lastAngleDeg = 0.0;
        isCalibrated = true;
    }

    // -------------------------
    // Position Control API
    // -------------------------

    /**
     * Sets the target angle for automatic control.
     * The turret will smoothly move to this angle using velocity control.
     */
    public void setTargetAngle(double targetAngleDegrees) {
        double clamped = Range.clip(targetAngleDegrees, MAX_ANGLE_CCW, MAX_ANGLE_CW);
        limitWarning = (clamped != targetAngleDegrees);
        targetAngleDeg = clamped;
    }

    /**
     * API compatibility - power parameter ignored in velocity mode.
     */
    public void setTargetAngle(double targetAngleDegrees, double unusedPowerParam) {
        setTargetAngle(targetAngleDegrees);
    }

    /**
     * Rotates relative to current position.
     */
    public void rotateRelative(double deltaDegrees) {
        setTargetAngle(getCurrentAngle() + deltaDegrees);
    }

    /**
     * Returns turret to home (0°) position.
     */
    public void returnToHome() {
        setTargetAngle(0.0);
    }

    /**
     * Holds current position - useful when switching from manual to auto.
     */
    public void holdPosition() {
        setTargetAngle(getCurrentAngle());
    }

    // -------------------------
    // Manual Override
    // -------------------------

    /**
     * Direct joystick control with deadzone and scaling.
     * Automatically handles switching between manual and auto control.
     */
    public void setTurretPower(double joystickInput) {
        double filtered = (Math.abs(joystickInput) < JOYSTICK_DEADZONE) ? 0.0 : joystickInput;
        setDirectPower(filtered * MANUAL_MAX_POWER);
    }

    /**
     * Low-level power control with safety limits.
     * Switches motor to direct power mode when active.
     */
    public void setDirectPower(double power) {
        // Block all movement if jam fault is active
        if (jamAutoStopTriggered) {
            ensureVelocityMode();
            turretMotor.setVelocity(0);
            manualOverrideActive = false;
            return;
        }

        double currentAngle = getCurrentAngle();
        double safePower = power;

        // Enforce angle limits
        if (currentAngle >= MAX_ANGLE_CW && safePower > 0) {
            safePower = 0;
            limitWarning = true;
        } else if (currentAngle <= MAX_ANGLE_CCW && safePower < 0) {
            safePower = 0;
            limitWarning = true;
        } else {
            limitWarning = false;
        }

        // Active joystick input - enter manual mode
        if (Math.abs(safePower) > 0.02) {
            manualOverrideActive = true;

            // Switch to direct power control
            if (turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            turretMotor.setPower(safePower);

            // Keep target synced so when released, it holds position
            targetAngleDeg = getCurrentAngle();

        } else {
            // Joystick released - exit manual mode
            manualOverrideActive = false;

            // Return to velocity control mode
            ensureVelocityMode();
            turretMotor.setVelocity(0);

            // Hold at release position
            holdPosition();
        }
    }

    /**
     * Helper to ensure motor is in velocity control mode with correct settings.
     */
    private void ensureVelocityMode() {
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, VEL_PIDF);
            turretMotor.setPower(1.0); // Full power for velocity control
        }
    }

    // -------------------------
    // Safety & Calibration
    // -------------------------

    /**
     * Emergency stop - immediately halts turret and holds position.
     */
    public void emergencyStop() {
        manualOverrideActive = false;
        ensureVelocityMode();
        turretMotor.setVelocity(0);
        holdPosition();
    }

    /**
     * Recalibrates encoder to current position as 0°.
     * WARNING: Only call when turret is physically facing forward!
     */
    public void recalibrate() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ensureVelocityMode();

        targetAngleDeg = 0.0;
        manualOverrideActive = false;
        isCalibrated = true;
        limitWarning = false;
    }

    // -------------------------
    // State Getters
    // -------------------------

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getAngularVelocity() {
        return currentVelocityDegPerSec;
    }

    public boolean isAtTarget() {
        return Math.abs(getTargetAngle() - getCurrentAngle()) <= ANGLE_TOLERANCE_DEGREES;
    }

    public boolean hasLimitWarning() {
        return limitWarning;
    }

    public void clearLimitWarning() {
        limitWarning = false;
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public double getRemainingCW() {
        return MAX_ANGLE_CW - getCurrentAngle();
    }

    public double getRemainingCCW() {
        return getCurrentAngle() - MAX_ANGLE_CCW;
    }

    // -------------------------
    // Main Update Loop
    // CRITICAL: Call this every loop iteration!
    // -------------------------

    /**
     * Periodic update - MUST be called every loop.
     * Handles velocity control, velocity estimation, and jam detection.
     */
    public void update() {
        updateVelocityEstimate();
        updateCurrentMonitoring();

        // Don't interfere with manual control
        if (manualOverrideActive) return;

        // Don't move if jam fault is active
        if (jamAutoStopTriggered) {
            ensureVelocityMode();
            turretMotor.setVelocity(0);
            return;
        }

        // Ensure we're in velocity control mode
        ensureVelocityMode();

        // Calculate position error
        double currentAngle = getCurrentAngle();
        double errorDeg = targetAngleDeg - currentAngle;

        // Stop if at target
        if (Math.abs(errorDeg) <= ANGLE_TOLERANCE_DEGREES) {
            turretMotor.setVelocity(0);
            return;
        }

        // Proportional control: error → velocity
        double cmdVelDegPerSec = K_POS_TO_VEL * errorDeg;

        // Clamp to max velocity
        cmdVelDegPerSec = Range.clip(cmdVelDegPerSec, -MAX_VEL_DEG_PER_SEC, MAX_VEL_DEG_PER_SEC);

        // Add minimum velocity to overcome stiction (but not when very close)
        if (Math.abs(errorDeg) >= MIN_MOVE_ERROR_DEG) {
            if (Math.abs(cmdVelDegPerSec) < MIN_VEL_DEG_PER_SEC) {
                cmdVelDegPerSec = Math.signum(cmdVelDegPerSec) * MIN_VEL_DEG_PER_SEC;
            }
        }

        // Enforce soft limits - don't command velocity past limits
        if (currentAngle >= MAX_ANGLE_CW && cmdVelDegPerSec > 0) {
            cmdVelDegPerSec = 0;
        }
        if (currentAngle <= MAX_ANGLE_CCW && cmdVelDegPerSec < 0) {
            cmdVelDegPerSec = 0;
        }

        // Convert to ticks/sec and command motor
        double cmdTicksPerSec = cmdVelDegPerSec * TICKS_PER_DEGREE;
        turretMotor.setVelocity(cmdTicksPerSec);
    }

    /**
     * Calculates current velocity from encoder changes.
     */
    private void updateVelocityEstimate() {
        long now = System.currentTimeMillis();
        double angle = getCurrentAngle();
        double dt = (now - lastUpdateTimeMs) / 1000.0;

        if (dt > 0.001 && dt < 0.5) {
            currentVelocityDegPerSec = (angle - lastAngleDeg) / dt;
        } else if (dt >= 0.5) {
            currentVelocityDegPerSec = 0.0;
        }

        lastUpdateTimeMs = now;
        lastAngleDeg = angle;
    }

    /**
     * Monitors current draw and detects jam/slip conditions.
     */
    private void updateCurrentMonitoring() {
        currentDrawA = turretMotor.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;

        // Only check for jams when we expect motion
        boolean expectingMotion = !manualOverrideActive && !isAtTarget();

        if (expectingMotion) {
            boolean highCurrent = currentDrawA > JAM_CURRENT_THRESHOLD;
            boolean lowVelocity = Math.abs(currentVelocityDegPerSec) < MIN_VELOCITY_THRESHOLD;

            if (highCurrent && lowVelocity) {
                if (highCurrentStartTimeMs == 0) {
                    highCurrentStartTimeMs = System.currentTimeMillis();
                }

                long durationMs = System.currentTimeMillis() - highCurrentStartTimeMs;

                // Initial jam detection
                if (durationMs > (long) (SLIP_DETECTION_TIME * 1000)) {
                    jamDetected = true;
                    slipDetected = true;
                }

                // Auto-stop after sustained jam
                if (durationMs > (long) (JAM_AUTO_STOP_TIME * 1000) && !jamAutoStopTriggered) {
                    emergencyStop();
                    jamAutoStopTriggered = true;
                }
            } else {
                // Motion is normal - reset detection
                highCurrentStartTimeMs = 0;
                jamDetected = false;
                slipDetected = false;
            }
        } else {
            // Not expecting motion - don't flag as jam
            highCurrentStartTimeMs = 0;
            jamDetected = false;
            slipDetected = false;
        }
    }

    // -------------------------
    // Jam Detection API
    // -------------------------

    public double getMotorCurrent() {
        return currentDrawA;
    }

    public boolean isJammed() {
        return jamDetected;
    }

    public boolean wasAutoStopTriggered() {
        return jamAutoStopTriggered;
    }

    public boolean isSlipping() {
        return slipDetected;
    }

    public void clearJamDetection() {
        jamDetected = false;
        slipDetected = false;
        jamAutoStopTriggered = false;
        highCurrentStartTimeMs = 0;
    }

    public boolean isHighCurrent() {
        return currentDrawA > NORMAL_CURRENT_THRESHOLD;
    }

    // -------------------------
    // Telemetry
    // -------------------------

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Hi George ", VEL_PIDF);

        telemetry.addData("Turret Angle", "Current: %.1f°, Target: %.1f°", getCurrentAngle(), getTargetAngle());
        telemetry.addData("Turret Mode", manualOverrideActive ? "MANUAL" : "AUTO (Velocity)");
        telemetry.addData("Turret Status", isAtTarget() ? "At Target" : "Moving...");
        telemetry.addData("Turret Velocity", "%.1f °/sec", currentVelocityDegPerSec);
        telemetry.addData("Turret Range", "CW: %.1f° | CCW: %.1f°", getRemainingCW(), getRemainingCCW());
        telemetry.addData("Motor Current", "%.2f A", currentDrawA);

        if (limitWarning) {
            telemetry.addData("⚠ TURRET WARNING", "At angle limit!");
        }

        if (!isCalibrated) {
            telemetry.addData("⚠ TURRET WARNING", "NOT CALIBRATED!");
        }

        if (jamAutoStopTriggered) {
            telemetry.addData("🔴 TURRET FAULT", "AUTO-STOP! Clear jam and call clearJamDetection()");
        } else if (jamDetected) {
            long duration = System.currentTimeMillis() - highCurrentStartTimeMs;
            double timeRemaining = JAM_AUTO_STOP_TIME - (duration / 1000.0);
            telemetry.addData("🔴 TURRET ALERT", "JAM DETECTED! Auto-stop in %.1fs", timeRemaining);
        }

        if (isHighCurrent() && !jamDetected && !jamAutoStopTriggered) {
            telemetry.addData("⚠ TURRET WARNING", "High current draw");
        }
    }
}