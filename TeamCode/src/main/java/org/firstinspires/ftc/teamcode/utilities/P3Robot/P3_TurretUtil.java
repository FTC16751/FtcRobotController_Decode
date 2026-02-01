package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A utility class for controlling the robot's rotating launcher turret.
 * This class manages the motor, encoder, gear ratios, and software limits
 * to provide a simple angle-based control system with current monitoring.
 *
 * Hardware:
 * - AndyMark 5.5in Turntable Assembly
 * - 200-tooth turntable gear (am-robits-200t)
 * - 50-tooth driving gear (am-5020_50)
 * - GoBilda 312 RPM motor (19.2:1, 537.7 ticks/rev)
 *
 * Coordinate System:
 * - 0 degrees = turret facing front of robot
 * - Positive angles = clockwise rotation (max +100 degrees)
 * - Negative angles = counter-clockwise rotation (max -180 degrees)
 */
public class P3_TurretUtil {

    // --- Hardware ---
    private final DcMotorEx turretMotor;

    // --- Constants for Turret Mechanics ---
    private static final double TICKS_PER_MOTOR_REV = 537.7; // For GoBilda 312 RPM motor (19.2:1 gearbox)
    private static final double GEAR_RATIO = 200.0 / 50.0;   // 4:1 ratio (200T turntable / 50T drive gear)
    private static final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO; // 2150.8 ticks
    private static final double TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0; // ~5.974 ticks/degree

    // --- Constants for Software Limits ---
    public static final double MAX_ANGLE_CW = 100.0;   // Maximum clockwise rotation in degrees
    public static final double MAX_ANGLE_CCW = -180.0; // Maximum counter-clockwise rotation in degrees

    // --- Constants for Motor Control ---
    private static final double GOTO_POSITION_POWER = 0.7; // Power applied when moving to a target position
    private static final double SLOW_APPROACH_POWER = 0.3; // Reduced power near limits for safety
    private static final double SOFT_LIMIT_ZONE = 10.0;    // Degrees before limit where we slow down
    private static final PIDFCoefficients TURRET_PIDF_COEFFS = new PIDFCoefficients(20.0, 0.05, 0.0, 11.7); // P, I, D, F - These require tuning!

    // --- Tolerance for "at target" checks ---
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0; // Degrees within target to consider "at position"

    // --- Current Monitoring Constants ---
    private static final double NORMAL_CURRENT_THRESHOLD = 3.0;  // Amps - typical current during movement
    private static final double JAM_CURRENT_THRESHOLD = 5.0;     // Amps - current indicating a jam
    private static final double MIN_VELOCITY_THRESHOLD = 5.0;    // Degrees/sec - minimum velocity when moving
    private static final double SLIP_DETECTION_TIME = 0.5;       // Seconds of high current + low velocity = jam/slip
    private static final double JAM_AUTO_STOP_TIME = 2.0;        // Seconds - auto emergency stop after sustained jam

    // --- Safety and Diagnostics ---
    private boolean isCalibrated = false;
    private boolean limitWarning = false;
    private long lastUpdateTime = 0;
    private double lastAngle = 0.0;
    private double currentVelocity = 0.0; // degrees per second

    // --- Current Monitoring State ---
    private double currentDraw = 0.0;
    private long highCurrentStartTime = 0;
    private boolean jamDetected = false;
    private boolean slipDetected = false;
    private boolean jamAutoStopTriggered = false;

    /**
     * Constructor for the Turret Utility.
     * @param hardwareMap The OpMode's hardwareMap, used to find the motor.
     */
    public P3_TurretUtil(HardwareMap hardwareMap) {
        // Initialize the motor from the robot's configuration.
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor"); // Make sure this name is in your robot config!

        // Set the motor direction. You may need to change this to REVERSE.
        // Test: If you set a positive angle and it moves counter-clockwise, set this to REVERSE.
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // BRAKE will actively hold position better when stopped.
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoder when the robot starts. This defines the front-facing
        // position as the zero-degree reference point.
        // IMPORTANT: Make sure the turret is physically facing forward when you initialize!
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set custom PIDF coefficients for better performance.
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, TURRET_PIDF_COEFFS);

        // Set the motor to use its built-in PID controller to hold a target position.
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power that the motor will use when moving.
        turretMotor.setPower(GOTO_POSITION_POWER);

        // Initialize timing for velocity calculation
        lastUpdateTime = System.currentTimeMillis();
        lastAngle = 0.0;

        // Mark as calibrated (assumes turret is at front-facing position)
        isCalibrated = true;
    }

    /**
     * Sets the turret's target angle.
     * The angle will be automatically clamped to the defined software limits.
     * @param targetAngleDegrees The desired angle in degrees (-180 to +100).
     */
    public void setTargetAngle(double targetAngleDegrees) {
        // 1. Clamp the requested angle to our software limits to prevent damage.
        double clampedAngle = Range.clip(targetAngleDegrees, MAX_ANGLE_CCW, MAX_ANGLE_CW);

        // Set warning flag if we had to clamp the angle
        limitWarning = (clampedAngle != targetAngleDegrees);

        // 2. Convert the desired angle in degrees to the required number of encoder ticks.
        int targetTicks = (int) Math.round(clampedAngle * TICKS_PER_DEGREE);

        // 3. Command the motor to move to the calculated tick position.
        turretMotor.setTargetPosition(targetTicks);

        // 4. Ensure the motor is in the correct mode and has power.
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // 5. Adjust power based on CURRENT position proximity to limits (not target!)
        double currentAngle = getCurrentAngle();
        double desiredPower = isNearLimit(currentAngle) ? SLOW_APPROACH_POWER : GOTO_POSITION_POWER;

        // Use centralized power setting method
        setMotorPowerSafe(desiredPower);
    }
    private void setMotorPowerSafe(double power) {
        // Final safety check: Block if auto-stop fault is active
        if (jamAutoStopTriggered) {
            turretMotor.setPower(0.0);
            return;
        }

        // Additional safety: Double-check angle limits
        double currentAngle = getCurrentAngle();
        if ((currentAngle >= MAX_ANGLE_CW && power > 0) ||
                (currentAngle <= MAX_ANGLE_CCW && power < 0)) {
            turretMotor.setPower(0.0);
            return;
        }

        // All checks passed - set the power
        turretMotor.setPower(power);
    }
    /**
     * Holds the current position using position control.
     * Switches from power control back to position control.
     */
    public void holdPosition() {
        int currentTicks = turretMotor.getCurrentPosition();
        turretMotor.setTargetPosition(currentTicks);

        if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(GOTO_POSITION_POWER);
        }
    }
    /**
     * Sets the turret's target angle with a custom power level.
     * Useful for slow, precise movements or testing.
     * @param targetAngleDegrees The desired angle in degrees (-180 to +100).
     * @param power The motor power to use (0.0 to 1.0).
     */
    public void setTargetAngle(double targetAngleDegrees, double power) {
        double clampedAngle = Range.clip(targetAngleDegrees, MAX_ANGLE_CCW, MAX_ANGLE_CW);
        limitWarning = (clampedAngle != targetAngleDegrees);

        int targetTicks = (int) Math.round(clampedAngle * TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetTicks);

        if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Clamp power to reasonable range
        double clampedPower = Range.clip(power, 0.1, 1.0);

        // Use centralized power setting method
        setMotorPowerSafe(clampedPower);
    }

    /**
     * Rotates the turret by a relative amount from its current position.
     * @param deltaDegrees The amount to rotate (positive = CW, negative = CCW).
     */
    public void rotateRelative(double deltaDegrees) {
        double newTarget = getCurrentAngle() + deltaDegrees;
        setTargetAngle(newTarget);
    }

    /**
     * Returns the turret to the front-facing (0 degree) position.
     */
    public void returnToHome() {
        setTargetAngle(0.0);
    }

    /**
     * Emergency stop - immediately stops the turret and holds current position.
     * Useful for safety or when switching control modes.
     */
    public void emergencyStop() {
        // Set target to current position to stop movement
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setPower(0.0);
    }

    /**
     * Sets the turret motor power directly based on joystick input (manual control mode).
     * This method provides smooth, snappy control with automatic safety limits.
     * Applies deadzone filtering and scales power appropriately.
     *
     * @param joystickInput The joystick value (-1.0 to 1.0). Positive = CW, Negative = CCW.
     */
    public void setTurretPower(double joystickInput) {
        // Apply deadzone
        double filteredInput = Math.abs(joystickInput) < 0.1 ? 0.0 : joystickInput;

        // Scale to reasonable max power for smooth control
        double desiredPower = filteredInput * 0.8;

        // Use the direct power method with safety limits
        setDirectPower(desiredPower);
    }

    /**
     * Sets the turret motor power directly (manual control mode).
     * Automatically enforces safety limits - will stop motor if at limits.
     * This switches the motor to RUN_WITHOUT_ENCODER mode for direct control.
     *
     * @param power The desired motor power (-1.0 to 1.0). Positive = CW, Negative = CCW.
     */
    public void setDirectPower(double power) {
        // Switch to direct power mode if not already
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double currentAngle = getCurrentAngle();
        double safePower = power;

        // Safety: prevent movement beyond limits
        if (currentAngle >= MAX_ANGLE_CW && power > 0) {
            safePower = 0; // At CW limit, block further CW rotation
            limitWarning = true;
        } else if (currentAngle <= MAX_ANGLE_CCW && power < 0) {
            safePower = 0; // At CCW limit, block further CCW rotation
            limitWarning = true;
        } else {
            limitWarning = false;
        }

        setMotorPowerSafe(safePower);
    }

    /**
     * Recalibrates the turret by setting the current position as 0 degrees.
     * WARNING: Only call this when the turret is physically facing forward!
     */
    public void recalibrate() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Use centralized power setting
        setMotorPowerSafe(GOTO_POSITION_POWER);

        isCalibrated = true;
        limitWarning = false;
    }

    /**
     * Gets the current target angle of the turret in degrees.
     * @return The current target angle.
     */
    public double getTargetAngle() {
        return turretMotor.getTargetPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Gets the current actual angle of the turret based on the encoder reading.
     * @return The current angle in degrees.
     */
    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Gets the current angular velocity in degrees per second.
     * This is calculated in the update() method.
     * @return The angular velocity.
     */
    public double getAngularVelocity() {
        return currentVelocity;
    }

    /**
     * Checks if the turret is at or very close to its target angle.
     * This is often more useful than isBusy() for checking if a movement is complete.
     * @return true if within ANGLE_TOLERANCE_DEGREES of the target.
     */
    public boolean isAtTarget() {
        // isBusy() can sometimes be unreliable; checking the error is more robust.
        double error = Math.abs(getTargetAngle() - getCurrentAngle());
        return error <= ANGLE_TOLERANCE_DEGREES;
    }

    /**
     * Checks if the turret is near a software limit (within SOFT_LIMIT_ZONE degrees).
     * @param angle The angle to check.
     * @return true if near either limit.
     */
    private boolean isNearLimit(double angle) {
        return (angle > MAX_ANGLE_CW - SOFT_LIMIT_ZONE) ||
                (angle < MAX_ANGLE_CCW + SOFT_LIMIT_ZONE);
    }

    /**
     * Checks if the last setTargetAngle() call was clamped due to limits.
     * @return true if a limit warning is active.
     */
    public boolean hasLimitWarning() {
        return limitWarning;
    }

    /**
     * Clears the limit warning flag.
     */
    public void clearLimitWarning() {
        limitWarning = false;
    }

    /**
     * Checks if the turret has been calibrated.
     * @return true if calibrated.
     */
    public boolean isCalibrated() {
        return isCalibrated;
    }

    /**
     * Gets the remaining range of motion in the clockwise direction.
     * @return Degrees remaining before hitting CW limit.
     */
    public double getRemainingCW() {
        return MAX_ANGLE_CW - getCurrentAngle();
    }

    /**
     * Gets the remaining range of motion in the counter-clockwise direction.
     * @return Degrees remaining before hitting CCW limit (positive number).
     */
    public double getRemainingCCW() {
        return getCurrentAngle() - MAX_ANGLE_CCW;
    }

    /**
     * A periodic update method that should be called regularly in your OpMode loop.
     * This calculates velocity and performs current monitoring.
     * IMPORTANT: Must be called consistently every loop for accurate velocity calculation.
     */
    public void update() {
        // Calculate velocity (degrees per second)
        long currentTime = System.currentTimeMillis();
        double currentAngle = getCurrentAngle();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds

        // Only calculate velocity if deltaTime is reasonable (between 1ms and 500ms)
        // This prevents bad calculations if update() isn't called regularly
        if (deltaTime > 0.001 && deltaTime < 0.5) {
            double deltaAngle = currentAngle - lastAngle;
            currentVelocity = deltaAngle / deltaTime;
        } else if (deltaTime >= 0.5) {
            // Update() wasn't called for a while - reset velocity to zero
            currentVelocity = 0.0;
        }
        // If deltaTime < 0.001, keep previous velocity (called too quickly)

        lastUpdateTime = currentTime;
        lastAngle = currentAngle;

        // Update current monitoring
        updateCurrentMonitoring();
    }

    /**
     * Updates current monitoring and detects jam/slip conditions.
     * Called automatically by update().
     */
    private void updateCurrentMonitoring() {
        // Read current draw from motor in milliamps, convert to amps
        currentDraw = turretMotor.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;

        double motorPower = Math.abs(turretMotor.getPower());

        // Only monitor when motor is actively trying to move
        if (motorPower > 0.1) {
            // Check for jam condition: high current + low velocity
            boolean highCurrent = currentDraw > JAM_CURRENT_THRESHOLD;
            boolean lowVelocity = Math.abs(currentVelocity) < MIN_VELOCITY_THRESHOLD;

            if (highCurrent && lowVelocity) {
                // Start tracking high current condition
                if (highCurrentStartTime == 0) {
                    highCurrentStartTime = System.currentTimeMillis();
                }

                long duration = System.currentTimeMillis() - highCurrentStartTime;

                // Initial jam detection after SLIP_DETECTION_TIME
                if (duration > SLIP_DETECTION_TIME * 1000) {
                    jamDetected = true;
                    slipDetected = true;
                }

                // Automatic emergency stop after sustained jam
                if (duration > JAM_AUTO_STOP_TIME * 1000 && !jamAutoStopTriggered) {
                    emergencyStop();
                    jamAutoStopTriggered = true;
                    // Note: jamDetected remains true so driver knows what happened
                }
            } else {
                // Reset tracking - motor is moving normally
                highCurrentStartTime = 0;
                jamDetected = false;
                slipDetected = false;
                // Don't reset jamAutoStopTriggered - this is a persistent fault
            }
        } else {
            // Motor not active, reset detection
            highCurrentStartTime = 0;
            jamDetected = false;
            slipDetected = false;
            // Don't reset jamAutoStopTriggered unless explicitly cleared
        }
    }

    /**
     * Gets the current motor current draw in amps.
     * @return The current draw in amps.
     */
    public double getMotorCurrent() {
        return currentDraw;
    }

    /**
     * Checks if the turret is jammed (high current, low velocity).
     * A jam indicates the turret is blocked and cannot move.
     * @return true if a jam is detected.
     */
    public boolean isJammed() {
        return jamDetected;
    }

    /**
     * Checks if the automatic emergency stop was triggered due to sustained jam.
     * This is a persistent fault that requires manual clearance.
     * @return true if auto-stop was triggered.
     */
    public boolean wasAutoStopTriggered() {
        return jamAutoStopTriggered;
    }

    /**
     * Checks if the turret gear is slipping (high current, no movement).
     * Slip detection uses the same logic as jam detection but can indicate
     * a mechanical problem like a loose gear or worn teeth.
     * @return true if slip is detected.
     */
    public boolean isSlipping() {
        return slipDetected;
    }

    /**
     * Resets the jam and slip detection flags.
     * Useful after clearing a jam or performing maintenance.
     * This also clears the auto-stop fault flag.
     */
    public void clearJamDetection() {
        jamDetected = false;
        slipDetected = false;
        jamAutoStopTriggered = false;
        highCurrentStartTime = 0;
    }

    /**
     * Gets whether the turret motor is drawing excessive current.
     * This can indicate a problem even if not yet classified as a jam.
     * @return true if current exceeds normal operating threshold.
     */
    public boolean isHighCurrent() {
        return currentDraw > NORMAL_CURRENT_THRESHOLD;
    }

    /**
     * Adds telemetry data for the turret to a Telemetry object for debugging.
     * @param telemetry The OpMode's telemetry object.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret Angle", "Current: %.1f°, Target: %.1f°", getCurrentAngle(), getTargetAngle());
        telemetry.addData("Turret Status", isAtTarget() ? "At Target" : "Moving...");
        telemetry.addData("Turret Velocity", "%.1f °/sec", currentVelocity);
        telemetry.addData("Turret Range", "CW: %.1f° | CCW: %.1f°", getRemainingCW(), getRemainingCCW());
        telemetry.addData("Turret Power", "%.2f", turretMotor.getPower());
        telemetry.addData("Motor Current", "%.2f A", currentDraw);

        if (limitWarning) {
            telemetry.addData("⚠ TURRET WARNING", "Target clamped to limits!");
        }

        if (!isCalibrated) {
            telemetry.addData("⚠ TURRET WARNING", "NOT CALIBRATED!");
        }

        if (jamAutoStopTriggered) {
            telemetry.addData("🔴 TURRET FAULT", "AUTO-STOP! Jam sustained >2s. Clear jam and call clearJamDetection()");
        } else if (jamDetected) {
            long duration = System.currentTimeMillis() - highCurrentStartTime;
            double timeRemaining = JAM_AUTO_STOP_TIME - (duration / 1000.0);
            telemetry.addData("🔴 TURRET ALERT", "JAM DETECTED! Will auto-stop in %.1fs", timeRemaining);
        }

        if (slipDetected && !jamDetected && !jamAutoStopTriggered) {
            telemetry.addData("🔴 TURRET ALERT", "GEAR SLIP DETECTED!");
        }

        if (isHighCurrent() && !jamDetected && !jamAutoStopTriggered) {
            telemetry.addData("⚠ TURRET WARNING", "High current draw");
        }
    }
}