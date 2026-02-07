package org.firstinspires.ftc.teamcode.utilities.P3Robot;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class P3_RubberBandIndexerUtil {
    private final DcMotor indexMotor;


    /** The default hardware map name for the intake motor. */
    public static final String MOTOR_NAME = "indexer";

    public P3_RubberBandIndexerUtil(HardwareMap hardwareMap) { // Pass HardwareMap in constructor
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }

        indexMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        indexMotor.setDirection(DcMotor.Direction.REVERSE);
        indexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /**
     * Sets the power of the intake motor.
     *
     * @param /power The desired power level, typically between -1.0 and 1.0.
     */
    public void setPower(double power) {
        indexMotor.setPower(power);
    }
    public void start() {
        indexMotor.setPower(1);
    }

    public void stop() {
        indexMotor.setPower(0);
    }
    public void reverse() {
        indexMotor.setPower(-1);
    }



    /**
     * Gets the current power of the intake motor.
     *
     * @return The current power level of the motor.
     */
    public double getMotorPower() {
        return indexMotor.getPower();
    }

    /**
     * Gets the current run mode of the intake motor.
     *
     * @return The current DcMotor.RunMode.
     */
    public DcMotor.RunMode getIntakeMode(){
        return indexMotor.getMode();
    }

}   //end program