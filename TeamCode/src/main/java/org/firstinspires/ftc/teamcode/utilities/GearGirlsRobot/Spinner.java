package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Spinner {

    /*
    The below two rotations should be hardcoded. We know where we want the spinner to spin to. We'll never
    ask the spinner to spin to any positions other than the shooting positions.
    We only need to spin to two positions: "left" and "right". These two positions cover all possible shooting cases.
    We don't need a third position. Default starting position allows us to shoot two balls. To shoot the third,
    we rotate left. Then to reset we reverse, or rotate right.

    TODO: figure out how much to rotate. These are just placeholder values. These values will then be used in the calls to setPosition(...)
     inside the rotateLeft() and rotateRight() methods below.

    */

    private static final double LEFT_ROTATION = 0.33333;
    private static final double RIGHT_ROTATION = -0.33333;

    private final Servo spinnerServo;

    private enum SpinnerState {IDLE, SPINNING, HAS_SPUN}
    private static enum SpinnerDirection { LEFT, RIGHT }

    private SpinnerState spinnerState = SpinnerState.IDLE;

    //TODO: write the code related to the time it takes the spinner to spin. see LaunchFlippers.java for an example.
    private final ElapsedTime spinnerTimer = new ElapsedTime();

    //Name for Hardware Map
    private static final String SPINNER_NAME = "spinner";

    public Spinner(@NonNull HardwareMap hardwareMap) {
        try {
            spinnerServo = hardwareMap.get(Servo.class, SPINNER_NAME);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(
                    "Failed to initialize Spinner. Ensure '" + SPINNER_NAME + "' is configured in the hardware map.", e);
        }

    }

    //TODO: Write the function to update the simple two-state state machine. We'll need code that sets the spinner back to idle once the
    // timer has elapsed


    public void rotateLeft() {
        spinnerState = SpinnerState.SPINNING;
        spinnerServo.setPosition(LEFT_ROTATION);
    }

    public void rotateRight () {
        spinnerState = SpinnerState.SPINNING;
        spinnerServo.setPosition(RIGHT_ROTATION);
    }

    public boolean isSpinnerBusy() {
        return spinnerState == SpinnerState.SPINNING;
    }

    public double getSpinnerPosition() {
        return spinnerServo.getPosition();
    }

    public String getSpinnerState() {
        return spinnerState.toString();
    }

}


     //   if (gamepad1.dpadLeftWasPressed() && spinnerPosition > 1) { spinnerPosition--; }
     //   if (gamepad1.dpadRightWasPressed() && spinnerPosition < 3) { spinnerPosition++; }