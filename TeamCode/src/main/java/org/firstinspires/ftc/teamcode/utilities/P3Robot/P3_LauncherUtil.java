package org.firstinspires.ftc.teamcode.utilities.P3Robot;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Utility class for controlling the robot's intake mechanism.
 * This class encapsulates the logic for initializing and operating the intake motor.
 */
public class P3_LauncherUtil {
    private final DcMotorEx shooterMotorLeft;
    private final DcMotorEx shooterMotorRight;
    private final CRServo indexerServo;
    private final Servo stopperServo;

    /** The default hardware map name for the intake motor. */
    /**
     * Constructs an IntakeUtil object.
     *
     * @param hardwareMap The HardwareMap from the OpMode, used to retrieve the intake motor.
     * @throws IllegalArgumentException if hardwareMap is null.
     */
    public P3_LauncherUtil(HardwareMap hardwareMap) { // Pass HardwareMap in constructor
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        //Intake Motor (new bot)
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "left_shooter");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "right_shooter");
        shooterMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        //set mode to run using encoder
        shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        indexerServo = hardwareMap.get(CRServo.class, "indexerServo");
        stopperServo = hardwareMap.get(Servo.class, "stopperServo");
    }


    /**
     * Stops the intake motor by setting its power to zero.
     */
    public void stopIntake() {
        setShooterMotorVelocity(0.0);
    }//Stop

    /**
     * Sets the power of the intake motor.
     *
     * @param /power The desired power level, typically between -1.0 and 1.0.
     */
    public void setShooterMotorVelocity(double power) {
        shooterMotorLeft.setVelocity(power);
        shooterMotorRight.setVelocity(power);

    }

    public void setIndexerServoPower(double power) {
        indexerServo.setPower(power);
    }

    /**
     * Gets the current power of the intake motor.
     *
     * @return The current power level of the motor.
     */
    public double getShooterMotorPower() {
        return shooterMotorLeft.getPower();
    }

    public double getShooterMotorVelocity() {
        return shooterMotorLeft.getVelocity();
    }

    /**
     * Gets the current run mode of the intake motor.
     *
     * @return The current DcMotor.RunMode.
     */
    public DcMotorEx.RunMode getIntakeMode(){
        return shooterMotorLeft.getMode();
    }


    public void setStopPosition() {
        stopperServo.setPosition(0.0);
        // FIX THIS LATER
    }

    public void setShootingPosition() {
        stopperServo.setPosition(0.25);
        // FIX THIS LATER
    }
}
//end program