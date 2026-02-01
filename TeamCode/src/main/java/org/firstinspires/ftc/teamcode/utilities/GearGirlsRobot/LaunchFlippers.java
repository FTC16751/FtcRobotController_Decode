package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manages the dual-servo "flipper" or "kicker" mechanism for launching game elements.
 * This class controls two standard servos that move between a retracted and a flipped
 * position to push an artifact into the launcher's flywheel.
 *
 * The core functionality is a state-based trigger. When a flipper is triggered,
 * it moves to the flipped position, holds for a moment, and then automatically returns.
 *
 * To ensure this automatic sequence completes, the {@link #update()} method must be called
 * in every iteration of the main OpMode loop.
 *
 * @version 2.0
 * @author GearGirls Team
 */
public class LaunchFlippers {

    // --- Hardware ---
    private final Servo leftFlipper;
    private final Servo rightFlipper;

    // --- Flipper State Machine ---
    private static enum FlipperState { IDLE, FLIPPING, RETRACTING }
    private FlipperState leftState = FlipperState.IDLE;
    private FlipperState rightState = FlipperState.IDLE;
    private final ElapsedTime leftTimer = new ElapsedTime();
    private final ElapsedTime rightTimer = new ElapsedTime();

    // --- Hardware Configuration ---
    private static final String LEFT_FLIPPER_NAME = "left_flipper";
    private static final String RIGHT_FLIPPER_NAME = "right_flipper";

    // --- Servo Direction Configuration ---
    // Set these to true if a servo needs to be reversed
    private boolean leftReversed = false;
    private boolean rightReversed = false;

    // --- Position Constants ---
    /**
     * Defines the servo positions for different states.
     * These values MUST be tuned by testing the physical robot.
     * Valid servo positions range from 0.0 to 1.0.
     */
    public static final class FlipperPosition {
        public static final double LEFT_HOME = 0.5;//0.5;//0.0;     // The retracted position for the left flipper
        public static final double LEFT_FLIPPED = 1.0;//0.0;//0.5;  // The extended/flipped position for the left flipper
        public static final double RIGHT_HOME = 0.5;//1.0;    // The retracted position for the right flipper
        public static final double RIGHT_FLIPPED = 0.0;//1.0;//0.5; // The extended/flipped position for the right flipper

        // Position bounds for safety
        private static final double MIN_POSITION = 0.0;
        private static final double MAX_POSITION = 1.0;
    }

    // --- Timing Constants ---
    private static final double DEFAULT_FLIP_HOLD_TIME_SECONDS = 0.25;
    private static final double MIN_FLIP_TIME = 0.05;  // Minimum safe flip time
    private static final double MAX_FLIP_TIME = 1.0;   // Maximum flip time

    // Separate constant for the physical servo travel time
    // This represents how long it takes the servo to physically move back to home position
    // This is NOT the same as flipHoldTime (which is how long to hold the flipped position)
    private static final double DEFAULT_SERVO_TRAVEL_TIME_SECONDS = 0.3;
    private static final double MIN_SERVO_TRAVEL_TIME = 0.05;
    private static final double MAX_SERVO_TRAVEL_TIME = 1.0;

    private double flipHoldTime = DEFAULT_FLIP_HOLD_TIME_SECONDS;
    private double servoTravelTime = DEFAULT_SERVO_TRAVEL_TIME_SECONDS;

    /**
     * Enum for selecting which flipper to control
     */
    public enum FlipperSide {
        LEFT,
        RIGHT,
        BOTH
    }

    /**
     * Constructs a new LaunchFlippers instance and initializes the hardware.
     * @param hardwareMap The hardware map from the OpMode
     * @throws IllegalArgumentException if servos cannot be found in hardware map
     */
    public LaunchFlippers(@NonNull HardwareMap hardwareMap) {
        try {
            leftFlipper = hardwareMap.get(Servo.class, LEFT_FLIPPER_NAME);
            rightFlipper = hardwareMap.get(Servo.class, RIGHT_FLIPPER_NAME);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(
                    "Failed to initialize LaunchFlippers. Ensure '" + LEFT_FLIPPER_NAME +
                            "' and '" + RIGHT_FLIPPER_NAME + "' are configured in the hardware map.", e);
        }

        // Initialize to a known starting position
        retractLeft();
        retractRight();
    }

    /**
     * Triggers the flip sequence for the specified servo(s).
     * If a flipper is already busy, that flipper's command is ignored.
     * @param side The flipper(s) to trigger (LEFT, RIGHT, or BOTH).
     */
    public void trigger(FlipperSide side) {
        switch (side) {
            case LEFT:
                triggerLeft();
                break;
            case RIGHT:
                triggerRight();
                break;
            case BOTH:
                triggerLeft();
                triggerRight();
                break;
        }
    }

    /**
     * Triggers the left flipper if it's not currently busy.
     */
    private void triggerLeft() {
        if (leftState == FlipperState.IDLE) {
            leftState = FlipperState.FLIPPING;
            setLeftPosition(FlipperPosition.LEFT_FLIPPED);
            leftTimer.reset();
        }
    }

    /**
     * Triggers the right flipper if it's not currently busy.
     */
    private void triggerRight() {
        if (rightState == FlipperState.IDLE) {
            rightState = FlipperState.FLIPPING;
            setRightPosition(FlipperPosition.RIGHT_FLIPPED);
            rightTimer.reset();
        }
    }

    /**
     * This method MUST be called in every cycle of the main OpMode loop.
     * It manages the state machine for the timed flip-and-return sequence.
     */
    public void update() {
        updateLeftFlipper();
        updateRightFlipper();
    }

    /**
     * Updates the left flipper state machine.
     */
    private void updateLeftFlipper() {
        switch (leftState) {
            case IDLE:
                // No action needed - ready to be triggered
                break;

            case FLIPPING:
                // Check if we've held the flipped position long enough
                if (leftTimer.seconds() > flipHoldTime) {
                    retractLeft();
                    leftState = FlipperState.RETRACTING;
                    leftTimer.reset();
                }
                break;

            case RETRACTING:
                // Wait for the servo to physically travel back to home position
                // Use servoTravelTime, not flipHoldTime, as this is a physical constraint
                if (leftTimer.seconds() > servoTravelTime) {
                    leftState = FlipperState.IDLE;
                }
                break;
        }
    }

    /**
     * Updates the right flipper state machine.
     */
    private void updateRightFlipper() {
        switch (rightState) {
            case IDLE:
                // No action needed - ready to be triggered
                break;

            case FLIPPING:
                // Check if we've held the flipped position long enough
                if (rightTimer.seconds() > flipHoldTime) {
                    retractRight();
                    rightState = FlipperState.RETRACTING;
                    rightTimer.reset();
                }
                break;

            case RETRACTING:
                // Wait for the servo to physically travel back to home position
                // Use servoTravelTime, not flipHoldTime, as this is a physical constraint
                if (rightTimer.seconds() > servoTravelTime) {
                    rightState = FlipperState.IDLE;
                }
                break;
        }
    }

    /**
     * Checks if either of the flipper servos are currently in the middle of a flip cycle.
     * @return true if a flipper is currently operating, false otherwise.
     */
    public boolean isBusy() {
        return leftState != FlipperState.IDLE || rightState != FlipperState.IDLE;
    }

    /**
     * Checks if the left flipper is currently busy.
     * @return true if the left flipper is operating, false otherwise.
     */
    public boolean isLeftBusy() {
        return leftState != FlipperState.IDLE;
    }

    /**
     * Checks if the right flipper is currently busy.
     * @return true if the right flipper is operating, false otherwise.
     */
    public boolean isRightBusy() {
        return rightState != FlipperState.IDLE;
    }

    // --- Configuration Methods ---

    /**
     * Sets the time the flipper holds in the flipped position before returning.
     * @param seconds Time in seconds (clamped between 0.05 and 1.0)
     */
    public void setFlipHoldTime(double seconds) {
        this.flipHoldTime = Math.max(MIN_FLIP_TIME, Math.min(MAX_FLIP_TIME, seconds));
    }

    /**
     * Gets the current flip hold time.
     * @return The flip hold time in seconds
     */
    public double getFlipHoldTime() {
        return flipHoldTime;
    }

    /**
     * Resets the flip hold time to the default value.
     */
    public void resetFlipHoldTime() {
        this.flipHoldTime = DEFAULT_FLIP_HOLD_TIME_SECONDS;
    }

    /**
     * Sets the time to wait for the servo to physically travel back to home position.
     * This should be tuned based on your servo's actual movement speed.
     * @param seconds Time in seconds (clamped between 0.05 and 1.0)
     */
    public void setServoTravelTime(double seconds) {
        this.servoTravelTime = Math.max(MIN_SERVO_TRAVEL_TIME, Math.min(MAX_SERVO_TRAVEL_TIME, seconds));
    }

    /**
     * Gets the current servo travel time.
     * @return The servo travel time in seconds
     */
    public double getServoTravelTime() {
        return servoTravelTime;
    }

    /**
     * Resets the servo travel time to the default value.
     */
    public void resetServoTravelTime() {
        this.servoTravelTime = DEFAULT_SERVO_TRAVEL_TIME_SECONDS;
    }

    /**
     * Reverses the direction of the left servo.
     * @param reversed true to reverse, false for normal direction
     */
    public void setLeftReversed(boolean reversed) {
        this.leftReversed = reversed;
    }

    /**
     * Reverses the direction of the right servo.
     * @param reversed true to reverse, false for normal direction
     */
    public void setRightReversed(boolean reversed) {
        this.rightReversed = reversed;
    }

    // --- Manual Control Methods (for testing and direct control) ---

    /**
     * Immediately commands the left flipper to its retracted (home) position.
     */
    public void retractLeft() {
        setLeftPosition(FlipperPosition.LEFT_HOME);
    }

    /**
     * Immediately commands the right flipper to its retracted (home) position.
     */
    public void retractRight() {
        setRightPosition(FlipperPosition.RIGHT_HOME);
    }

    /**
     * Immediately retracts both flippers to their home positions.
     */
    public void retractBoth() {
        retractLeft();
        retractRight();
    }

    /**
     * Emergency stop - immediately halts all flipper operations and retracts both.
     * Use this method if something goes wrong and you need to immediately stop the mechanism.
     */
    public void emergencyStop() {
        retractLeft();
        retractRight();
        leftState = FlipperState.IDLE;
        rightState = FlipperState.IDLE;
    }

    // --- Position Control with Validation and Direction Support ---

    /**
     * Sets the left servo position with range validation and direction reversal.
     * @param position The target position (0.0 to 1.0)
     */
    private void setLeftPosition(double position) {
        double validatedPosition = validatePosition(position);
        double finalPosition = leftReversed ? (1.0 - validatedPosition) : validatedPosition;
        leftFlipper.setPosition(finalPosition);
    }

    /**
     * Sets the right servo position with range validation and direction reversal.
     * @param position The target position (0.0 to 1.0)
     */
    private void setRightPosition(double position) {
        double validatedPosition = validatePosition(position);
        double finalPosition = rightReversed ? (1.0 - validatedPosition) : validatedPosition;
        rightFlipper.setPosition(finalPosition);
    }

    /**
     * Validates that a position is within the acceptable range.
     * @param position The position to validate
     * @return The validated position, clamped to [0.0, 1.0]
     */
    private double validatePosition(double position) {
        return Math.max(FlipperPosition.MIN_POSITION,
                Math.min(FlipperPosition.MAX_POSITION, position));
    }

    // --- Diagnostic and Testing Methods ---

    /**
     * Gets the current position of the left servo.
     * Useful for debugging and tuning.
     * @return The servo position (0.0 to 1.0)
     */
    public double getLeftPosition() {
        return leftFlipper.getPosition();
    }

    /**
     * Gets the current position of the right servo.
     * Useful for debugging and tuning.
     * @return The servo position (0.0 to 1.0)
     */
    public double getRightPosition() {
        return rightFlipper.getPosition();
    }

    /**
     * Gets the current state of the left flipper.
     * @return The current state (IDLE, FLIPPING, or RETRACTING)
     */
    public String getLeftState() {
        return leftState.toString();
    }

    /**
     * Gets the current state of the right flipper.
     * @return The current state (IDLE, FLIPPING, or RETRACTING)
     */
    public String getRightState() {
        return rightState.toString();
    }

    /**
     * Returns a formatted debug string showing the current state of both flippers.
     * Useful for telemetry display during testing.
     * @return A formatted string with flipper states and timings
     */
    public String getDebugInfo() {
        return String.format("L: %s (%.2fs) | R: %s (%.2fs) | Hold: %.2fs | Travel: %.2fs",
                leftState, leftTimer.seconds(),
                rightState, rightTimer.seconds(),
                flipHoldTime, servoTravelTime);
    }

    /**
     * Returns a compact status string for telemetry.
     * @return A brief status string
     */
    public String getStatus() {
        if (!isBusy()) {
            return "Ready";
        } else {
            String status = "";
            if (isLeftBusy()) status += "L:" + leftState + " ";
            if (isRightBusy()) status += "R:" + rightState;
            return status.trim();
        }
    }
}