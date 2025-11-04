package org.firstinspires.ftc.teamcode.utilities.PriorSeason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawUtil {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private Servo   leftHand = null;
    private Servo   rightHand = null;
    private boolean leftClawClosed = false;
    private boolean rightClawClosed = false;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    private static final double LEFT_CLAW_OPEN_POSITION = 0.25;
    private static final double LEFT_CLAW_CLOSED_POSITION = 0.70;
    private static final double RIGHT_CLAW_OPEN_POSITION = 0.75;
    private static final double RIGHT_CLAW_CLOSED_POSITION = 0.30;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ClawUtil(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap hardwareMap)    {

        // Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "Left Hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "Right Hand");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

    }


    public void setClawOpen() {
        openLeftHand();
        openRightHand();
    }
    public void setClawClosed(){
        closeLeftHand();
        closeRightHand();
    }

    public void openLeftHand(){
        leftHand.setPosition(LEFT_CLAW_OPEN_POSITION);
        leftClawClosed = false;
    }
    public void closeLeftHand() {
        leftHand.setPosition(LEFT_CLAW_CLOSED_POSITION);
        leftClawClosed = true;
    }
    public void openRightHand(){
        rightHand.setPosition(RIGHT_CLAW_OPEN_POSITION);
        rightClawClosed = false;
    }
    public void closeRightHand() {
        rightHand.setPosition(RIGHT_CLAW_CLOSED_POSITION);
        rightClawClosed = true;
    }
    public void toggleLeftClawWithBumper(boolean leftBumper) {
        boolean leftBumperEdge = leftBumper && !lastLeftBumperState;

        if (leftBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (leftClawClosed) {
                openLeftHand();
            } else {
                closeLeftHand();
            }
        }
        lastLeftBumperState = leftBumper;
    }
    public void toggleRightClawWithBumper(boolean rightBumper) {
        boolean rightBumperEdge = rightBumper && !lastRightBumperState;

        if (rightBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (rightClawClosed) {
                openRightHand();
            } else {
                closeRightHand();
            }
        }
        lastRightBumperState = rightBumper;
    }
    public double getLeftClawPosition(){
        return leftHand.getPosition();

    }
    public double getRightClawPosition(){
        return rightHand.getPosition();
    }
    public double getWristPosition(){
        return getWristPosition();
    }

}

