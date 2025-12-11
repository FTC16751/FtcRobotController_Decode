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
public class P3_LauncherUtil {
    private final DcMotorEx shooterMotorLeft;
    private final DcMotorEx shooterMotorRight;
    private final CRServo indexerServo,indexerServo2;
    private final Servo stopperServo;

    private double m_speed;

    enum shooterStates {
        OFF,
        SPINNING_UP,
        FLYWHEEL_SPUN,
        LAUNCHING
    }
    private shooterStates shooterState = shooterStates.OFF;


    /* The default hardware map name for the intake motor. */
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
        shooterMotorRight.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterMotorLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        shooterMotorRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        indexerServo = hardwareMap.get(CRServo.class, "indexerServo");
        indexerServo2 = hardwareMap.get(CRServo.class, "indexerServo2");
        //reverse indexerServo2
        indexerServo2.setDirection(CRServo.Direction.REVERSE);
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
     * @param speed The desired in ticks per second.
     */
    public void setShooterMotorVelocity(double speed) {
        shooterMotorLeft.setVelocity(speed);
        shooterMotorRight.setVelocity(speed);

    }

    public void setIndexerServoPower(double power) {

        indexerServo.setPower(power);
        indexerServo2.setPower(power);
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
        stopperServo.setPosition(0.2);
        // FIX THIS LATER
    }

    public void setShootingPosition() {
        stopperServo.setPosition(0.0);
        // FIX THIS LATER
    }

    public double getStopperServoPosition() {
        return stopperServo.getPosition();
    }
    public void spinUp(double speed) {
        setShooterMotorVelocity(m_speed = speed);
        shooterState = shooterStates.SPINNING_UP;
    }

    public void launch() {
        if(shooterState == shooterStates.FLYWHEEL_SPUN) {
            setIndexerServoPower(.25);
            setShootingPosition();
            shooterState = shooterStates.LAUNCHING;
        }
        else if (shooterState == shooterStates.SPINNING_UP) {
            if(getShooterMotorVelocity() > .99 * m_speed) {
                shooterState= shooterStates.FLYWHEEL_SPUN;
            }
        }

    }

    // LEFT OFF HERE WITH MATTHEW
    // CHECK WITH HIM TO MAKE SURE LOGIC IS RIGHT
    public void stopLaunching() {
        setIndexerServoPower(0);
        setStopPosition();
        shooterState = shooterStates.FLYWHEEL_SPUN;

}

    public void launchingOff() {
        setShooterMotorVelocity(0);
        setIndexerServoPower(0);
        shooterState=shooterStates.OFF;
    }
}
//end program