package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manages the dual-servo indexing mechanism for launching game elements.
 * This class controls two continuous rotation (CR) servos that feed game elements
 * into a launcher.
 *
 * The core functionality is a time-based trigger. When a feeder is triggered,
 * it runs for a predefined duration and then automatically stops.
 *
 * To ensure automatic stopping, the {@link #update()} method must be called
 * in every iteration of the main OpMode loop.
 */
public class LaunchIndexer {
    private final CRServo leftFeeder;
    private final CRServo rightFeeder;
    private final ElapsedTime leftFeederTimer = new ElapsedTime();
    private final ElapsedTime rightFeederTimer = new ElapsedTime();
    private static final String LEFT_FEEDER_NAME = "left_feeder";
    private static final String RIGHT_FEEDER_NAME = "right_feeder";
    private boolean isLeftFeederBusy = false;
    private boolean isRightFeederBusy = false;
    // Add this public enum
    public enum FeederSide {
        LEFT,
        RIGHT
    }

    public LaunchIndexer(@NonNull HardwareMap hardwareMap) {
        // We no longer need a separate initHardware method
        leftFeeder = hardwareMap.get(CRServo.class, LEFT_FEEDER_NAME);
        rightFeeder = hardwareMap.get(CRServo.class, RIGHT_FEEDER_NAME);

        // It's good practice to set directions during initialization
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    /**
     * Triggers the left feeder to run for a predefined duration.
     * This method starts the left feeder servo at full speed, marks it as busy,
     * and resets its timer. The {@link #update()} method must be called periodically
     * to automatically stop the feeder after the time has elapsed.
     */
    public void triggerLeftFeeder() {
        leftFeederTimer.reset();
        setLeftFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        isLeftFeederBusy = true;
    }

    /**
     * Activates the right feeder servo for a timed duration to feed a game element.
     * This method sets the servo to full power, marks it as busy, and resets its timer.
     * The servo will automatically be stopped by the {@link #update()} method after the
     * duration specified by {@code GGRobotConstants.Feeder.FEED_TIME_SECONDS} has elapsed.
     */
    public void triggerRightFeeder() {
        rightFeederTimer.reset();
        setRightFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        isRightFeederBusy = true;
    }

    /**
     * THIS IS THE KEY: This method must be called in every cycle of the main OpMode loop.
     * It checks if the feeders have been running long enough and stops them if they have.
     */
    public void update() {
        if (leftFeederTimer.seconds() > GGRobotConstants.Feeder.FEED_TIME_SECONDS) {
            setLeftFeederPower(GGRobotConstants.Feeder.STOP_SPEED);
            isLeftFeederBusy = false; // The feeder is no longer busy
        }
        if (rightFeederTimer.seconds() > GGRobotConstants.Feeder.FEED_TIME_SECONDS) {
            setRightFeederPower(GGRobotConstants.Feeder.STOP_SPEED);
            isRightFeederBusy = false; // The feeder is no longer busy
        }
    }


    /**
     * Checks if either of the feeder servos are currently in an active cycle.
     * @return true if a feeder is currently operating, false otherwise.
     */
    public boolean isBusy() {
        return isLeftFeederBusy || isRightFeederBusy;
    }


    // do a bunch of functions for left feeder
    public void setLeftFeederPower(double power) {
        leftFeeder.setPower(power);
    }
    public void startLeftFeeder() {
        setLeftFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
    }
    public void stopLeftFeeder() {
        setLeftFeederPower(GGRobotConstants.Feeder.STOP_SPEED);
    }
    public void reverseLeftFeeder() {
        setLeftFeederPower(-GGRobotConstants.Feeder.FULL_SPEED);
    }

    //do a bunch for right feeder
    public void setRightFeederPower(double power) {
        rightFeeder.setPower(power);
    }
    public void startRightFeeder() {
        setRightFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
    }
    public void stopRightFeeder() {
        setRightFeederPower(GGRobotConstants.Feeder.STOP_SPEED);
    }
    public void reverseRightFeeder() {
        setRightFeederPower(-GGRobotConstants.Feeder.FULL_SPEED);
    }

    // do a bunch for combined feeder actions
    public void setFeederPower(double power) {
        setLeftFeederPower(power);
        setRightFeederPower(power);
    }
    public void startFeeder() {
        setFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
    }
    public void stopFeeder() {
        setFeederPower(GGRobotConstants.Feeder.STOP_SPEED);
    }
    public void reverseFeeder() {
        setFeederPower(-GGRobotConstants.Feeder.FULL_SPEED);
    }



}
