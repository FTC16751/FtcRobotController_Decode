package org.firstinspires.ftc.teamcode.utilities.P3Robot;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Utility class for controlling the robot's intake mechanism.
 * This class encapsulates the logic for initializing and operating the intake motor.
 */
public class P3_IndexerUtil {
    private final DcMotorEx feederMotor;

    private double m_speed;

    /* The default hardware map name for the intake motor. */
    /**
     * Constructs an IntakeUtil object.
     *
     * @param hardwareMap The HardwareMap from the OpMode, used to retrieve the intake motor.
     * @throws IllegalArgumentException if hardwareMap is null.
     */
    public P3_IndexerUtil(HardwareMap hardwareMap) { // Pass HardwareMap in constructor
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        //Intake Motor (new bot)
        feederMotor = hardwareMap.get(DcMotorEx.class, "feeder_motor");

    }

    public void setFeederMotorPower(double m_speed) {
        feederMotor.setPower(m_speed);
    }

    public double getFeederMotorPower() {
        return feederMotor.getPower();
    }

}
//end program