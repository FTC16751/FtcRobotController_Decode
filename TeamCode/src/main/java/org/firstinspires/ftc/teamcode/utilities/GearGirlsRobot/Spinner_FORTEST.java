package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Spinner_FORTEST {
    /*
       The below two rotations should be hardcoded. We know where we want the spinner to spin to. We'll never
       ask the spinner to spin to any positions other than the shooting positions.
       We only need to spin to two positions: "left" and "right". These two positions cover all possible shooting cases.
       We don't need a third position. Default starting position allows us to shoot two balls. To shoot the third,
       we rotate left. Then to reset we reverse, or rotate right.

       TODO: figure out how much to rotate. These are just placeholder values. These values will then be used in the calls to setPosition(...)
        inside the rotateLeft() and rotateRight() methods below.

       */


    // --- Hardware ---
    private final Servo spinnerServo;
    private final ElapsedTime spinnerTimer = new ElapsedTime();

    // --- State Machine ---
    private enum SpinnerState {
        IDLE,       // Ready for commands
        SPINNING,   // Currently moving
        HAS_SPUN    // Completed movement (optional state for feedback)
    }
    private SpinnerState spinnerState = SpinnerState.IDLE;

    // --- Position Constants ---
    private static final double LEFT_ROTATION = 0.2;   // TODO: Tune this value
    private static final double RIGHT_ROTATION = 0.63;  // TODO: Tune this value
    private static final double CENTER_POSITION = 0.63;     // Default/home position

    // --- Timing Constants ---
    /**
     * Time to wait for the servo to complete its movement.
     */
    private static final double SPIN_TIME_SECONDS = 0.3;  // TODO: Tune based on servo speed

    // --- Hardware Configuration ---
    private static final String SPINNER_NAME = "spinner";

    public Spinner_FORTEST(@NonNull HardwareMap hardwareMap) {
        spinnerServo = hardwareMap.get(Servo.class, SPINNER_NAME);

        // Initialize to center/home position
        spinnerServo.setPosition(CENTER_POSITION);
        spinnerState = SpinnerState.IDLE;
    }

    /**
     * This method MUST be called in every cycle of the main OpMode loop.
     * It manages the state machine for the timed spin sequence.
     */
    public void update() {
        switch (spinnerState) {
            case IDLE:
                break;

            case SPINNING:
                // Check if the servo has had enough time to complete its movement
                if (spinnerTimer.seconds() > SPIN_TIME_SECONDS) {
                    spinnerState = SpinnerState.HAS_SPUN;
                }
                break;

            case HAS_SPUN:
                spinnerState = SpinnerState.IDLE;
                break;
        }
    }


    public void rotateLeft() {
        if (spinnerState == SpinnerState.IDLE) {
            spinnerState = SpinnerState.SPINNING;
            spinnerServo.setPosition(LEFT_ROTATION);
            spinnerTimer.reset();
        }
    }


    public void rotateRight() {
        if (spinnerState == SpinnerState.IDLE) {
            spinnerState = SpinnerState.SPINNING;
            spinnerServo.setPosition(RIGHT_ROTATION);
            spinnerTimer.reset();
        }
    }


    //if needed could be handy
    public void rotateCenter() {
        if (spinnerState == SpinnerState.IDLE) {
            spinnerState = SpinnerState.SPINNING;
            spinnerServo.setPosition(CENTER_POSITION);
            spinnerTimer.reset();
        }
    }

    /**
     * Forces the spinner to a specific position immediately.
     * Use with caution - bypasses the state machine.
     *
     */
    public void setPosition(double position) {
        double clampedPosition = Math.max(0.0, Math.min(1.0, position));
        spinnerServo.setPosition(clampedPosition);
        spinnerState = SpinnerState.SPINNING;
        spinnerTimer.reset();
    }


    /**
     * Checks if the spinner is currently busy with a movement.
     * @return true if the spinner is moving, false if idle
     */
    public boolean isSpinnerBusy() {
        return spinnerState == SpinnerState.SPINNING;
    }


    /**
     * warning about servo 'getPosition'.
     * It doesn't return the actual position of the servo
     * It only returns the last position you commanded it to turn to
     * Our servos do not have encoders built in
    */
    public double getSpinnerPosition() {
        return spinnerServo.getPosition();
    }

    public String getSpinnerState() {
        return spinnerState.toString();
    }



    public String getStatus() {
        if (spinnerState == SpinnerState.IDLE) {
            return "Ready";
        } else {
            return String.format("Spinning (%.2fs)", spinnerTimer.seconds());
        }
    }


}