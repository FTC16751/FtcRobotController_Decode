package org.firstinspires.ftc.teamcode.utilities.P3Robot;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Utility class for controlling the robot's intake mechanism.
 * This class encapsulates the logic for initializing and operating the intake motor.
 */
public class P3_HoodServoUtil {

    private final Servo hoodServo;


    public P3_HoodServoUtil(HardwareMap hardwareMap) { // Pass HardwareMap in constructor
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }


    public void setPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getPosition() {
        return hoodServo.getPosition();
    }

    public void setLowPosition() {
        hoodServo.setPosition(0);
    }

    public void setMidPosition() {
        hoodServo.setPosition(0.5);
    }

    public void setHighPosition() {
        hoodServo.setPosition(0.1);
    }

}