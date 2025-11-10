package org.firstinspires.ftc.teamcode.utilities.Skyline;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A utility class for controlling the Skyline robot's feeder servos.
 */
public class Skyline_FeederUtil {

    private final CRServo leftFeeder;
    private final CRServo rightFeeder;

    public Skyline_FeederUtil(HardwareMap hardwareMap) {
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Set one servo to reverse so they spin in the same functional direction
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure servos are stopped on initialization
        stop();
    }

    /**
     * Sets the power for the left feeder servo.
     * @param power The power, from -1.0 to 1.0.
     */
    public void setLeftPower(double power) {
        leftFeeder.setPower(power);
    }

    /**
     * Sets the power for the right feeder servo.
     * @param power The power, from -1.0 to 1.0.
     */
    public void setRightPower(double power) {
        rightFeeder.setPower(power);
    }

    /**
     * Sets the power for both feeder servos simultaneously.
     * @param power The power, from -1.0 to 1.0.
     */
    public void setPower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }

    /**
     * Stops both feeder servos.
     */
    public void stop() {
        setPower(0.0);
    }
}
