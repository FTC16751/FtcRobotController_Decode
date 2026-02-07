package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Utility class for controlling the robot's intake mechanism.
 *
 * This class provides a simplified interface for operating the intake motor. It handles
 * the initialization of the motor from the hardware map, including setting its
 * direction and zero power behavior. It offers methods to set the motor's power,
 * stop it, and retrieve its current status.
 *
 * Features:
 * - Preset power levels for common operations
 * - Emergency stop capability
 * - Status and diagnostic methods
 * - Proper error handling
 *
 * @version 2.0 (No Diverter)
 * @author GearGirls Team
 */
public class IntakeUtilV2 {

    // --- Hardware Configuration ---
    private static final String INTAKE_MOTOR_NAME = "intake";
    private static final String INTAKE_MOTOR_NAME2 = "intake2";

    // --- Hardware ---
    private final DcMotor intakeMotor, intakeMotor2;

    // --- Motor Configuration ---
    private static final DcMotor.Direction DEFAULT_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.ZeroPowerBehavior DEFAULT_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    public void intakeOn() {
        setIntakeMotorPower(1);
    }

    /**
     * Preset power levels for common intake operations.
     * These values can be tuned based on your robot's performance needs.
     */
    public static final class IntakePower {
        /** Full power intake - pulling game elements in */
        public static final double INTAKE = 1.0;

        /** Full power outtake - ejecting game elements */
        public static final double OUTTAKE = -1.0;

        /** Slow intake for precise control */
        public static final double INTAKE_SLOW = 0.5;

        /** Slow outtake for precise control */
        public static final double OUTTAKE_SLOW = -0.5;

        /** Stop the motor */
        public static final double STOP = 0.0;
    }

    /**
     * Constructs a new IntakeUtil and initializes the hardware.
     *
     * @param hardwareMap The hardware map from the OpMode
     * @throws IllegalArgumentException if the intake motor cannot be found
     */
    public IntakeUtilV2(@NonNull HardwareMap hardwareMap) {
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
            intakeMotor2 = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME2);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(
                    "Failed to initialize IntakeUtil. Ensure '" + INTAKE_MOTOR_NAME +
                            "' is configured in the hardware map.", e);
        }

        // Configure motor behavior
        // Note: REVERSE direction is used because of the physical mounting orientation
        // of the motor on the robot. Change this if your motor spins the wrong way.
        intakeMotor.setDirection(DEFAULT_MOTOR_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DEFAULT_ZERO_POWER_BEHAVIOR);

        intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor2.setZeroPowerBehavior(DEFAULT_ZERO_POWER_BEHAVIOR);

        // Initialize to stopped state
        stop();
    }

    // --- Primary Control Methods ---

    /**
     * Sets the intake motor power to a specific value.
     * Positive values intake, negative values outtake.
     *
     * @param power The power level (-1.0 to 1.0)
     */
    public void setIntakeMotorPower(double power) {
        intakeMotor.setPower(clampPower(power));
        intakeMotor2.setPower(clampPower(power));
    }

    /**
     * Stops the intake motor by setting its power to zero.
     */
    public void stop() {
        intakeMotor.setPower(IntakePower.STOP);
        intakeMotor2.setPower(IntakePower.STOP);
    }

    // --- Convenience Methods ---

    /**
     * Runs the intake at full power to pull in game elements.
     */
    public void intake() {
        setIntakeMotorPower(IntakePower.INTAKE);
    }

    /**
     * Runs the intake in reverse at full power to eject game elements.
     */
    public void outtake() {
        setIntakeMotorPower(IntakePower.OUTTAKE);
    }

    /**
     * Runs the intake at half power for more controlled operation.
     */
    public void intakeSlow() {
        setIntakeMotorPower(IntakePower.INTAKE_SLOW);
    }

    /**
     * Runs the outtake at half power for more controlled operation.
     */
    public void outtakeSlow() {
        setIntakeMotorPower(IntakePower.OUTTAKE_SLOW);
    }

    // --- Configuration Methods ---

    /**
     * Sets the motor direction.
     * Use this if the motor spins the wrong way for your robot's configuration.
     *
     * @param direction The desired motor direction
     */
    public void setMotorDirection(DcMotor.Direction direction) {
        intakeMotor.setDirection(direction);
    }

    /**
     * Reverses the current motor direction.
     * Useful for testing or if you discover the motor is spinning backwards.
     */
    public void reverseMotorDirection() {
        DcMotor.Direction current = intakeMotor.getDirection();
        DcMotor.Direction reversed = (current == DcMotor.Direction.FORWARD)
                ? DcMotor.Direction.REVERSE
                : DcMotor.Direction.FORWARD;
        intakeMotor.setDirection(reversed);
    }

    /**
     * Sets the motor's zero power behavior.
     *
     * @param behavior BRAKE to actively stop, FLOAT to coast
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        intakeMotor.setZeroPowerBehavior(behavior);
        intakeMotor2.setZeroPowerBehavior(behavior);

    }

    // --- Status and Diagnostic Methods ---

    /**
     * Gets the current power level of the intake motor.
     *
     * @return The current power (-1.0 to 1.0)
     */
    public double getCurrentPower() {
        return intakeMotor.getPower();
    }

    /**
     * Checks if the intake motor is currently running.
     *
     * @return true if power is non-zero, false otherwise
     */
    public boolean isRunning() {
        return Math.abs(intakeMotor.getPower()) > 0.01; // Small threshold for floating point
    }

    /**
     * Gets the current motor direction.
     *
     * @return The motor direction (FORWARD or REVERSE)
     */
    public DcMotor.Direction getMotorDirection() {
        return intakeMotor.getDirection();
    }

    /**
     * Gets the current zero power behavior.
     *
     * @return The zero power behavior (BRAKE or FLOAT)
     */
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return intakeMotor.getZeroPowerBehavior();
    }

    /**
     * Gets a brief status string suitable for telemetry.
     *
     * @return A status string showing power and running state
     */
    public String getStatus() {
        if (!isRunning()) {
            return "Stopped";
        }

        double power = getCurrentPower();
        if (power > 0) {
            return String.format("Intaking (%.1f%%)", power * 100);
        } else {
            return String.format("Outtaking (%.1f%%)", Math.abs(power) * 100);
        }
    }

    /**
     * Gets detailed debug information for troubleshooting.
     *
     * @return A formatted debug string
     */
    public String getDebugInfo() {
        return String.format("Power: %.2f | Dir: %s | ZPB: %s | Running: %s",
                getCurrentPower(),
                getMotorDirection(),
                getZeroPowerBehavior(),
                isRunning() ? "YES" : "NO");
    }

    // --- Utility Methods ---

    /**
     * Clamps power values to the valid range [-1.0, 1.0].
     *
     * @param power The power value to clamp
     * @return The clamped power value
     */
    private double clampPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }
}