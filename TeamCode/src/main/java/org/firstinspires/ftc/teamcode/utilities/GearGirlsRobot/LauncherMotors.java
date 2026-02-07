package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherMotors {

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private HardwareMap hardwareMap;

    // goBILDA 5203 Yellow Jacket motor encoder specification
    // Note: This motor has 28 PPR (Pulses Per Revolution) at the output shaft
    private static final double COUNTS_PER_REVOLUTION = 28.0;  // Encoder ticks per motor revolution
    private static final double MAX_RPM = 6000.0;  // Motor specification (no-load max speed)

    public LauncherMotors(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
        init();
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        leftLauncher.setDirection(DcMotorEx.Direction.REVERSE);

        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftLauncher.setZeroPowerBehavior(FLOAT);
        rightLauncher.setZeroPowerBehavior(FLOAT);

        leftLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(271, 0, 0, 14));
        rightLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(271, 0, 0, 14));
    }

    /**
     * Converts RPM to encoder ticks per second
     * @param rpm Revolutions per minute
     * @return Encoder ticks per second
     */
    private double rpmToTicksPerSecond(double rpm) {
        // RPM → revolutions per second → ticks per second
        return (rpm / 60.0) * COUNTS_PER_REVOLUTION;
    }

    /**
     * Converts encoder ticks per second to RPM
     * @param ticksPerSecond Encoder ticks per second
     * @return Revolutions per minute
     */
    private double ticksPerSecondToRPM(double ticksPerSecond) {
        // ticks per second → revolutions per second → RPM
        return (ticksPerSecond / COUNTS_PER_REVOLUTION) * 60.0;
    }

    /**
     * Set motor velocities using RPM values
     * @param leftRPM Target velocity for left motor in RPM
     * @param rightRPM Target velocity for right motor in RPM
     */
    public void setMotorVelocityRPM(double leftRPM, double rightRPM) {
        double leftTicksPerSec = rpmToTicksPerSecond(leftRPM);
        double rightTicksPerSec = rpmToTicksPerSecond(rightRPM);

        leftLauncher.setVelocity(leftTicksPerSec);
        rightLauncher.setVelocity(rightTicksPerSec);
    }

    /**
     * Set motor velocities using raw encoder ticks per second
     * (Legacy method for backward compatibility)
     * @param leftTicksPerSec Target velocity for left motor in ticks/second
     * @param rightTicksPerSec Target velocity for right motor in ticks/second
     */
    public void setMotorVelocity(double leftTicksPerSec, double rightTicksPerSec) {
        leftLauncher.setVelocity(leftTicksPerSec);
        rightLauncher.setVelocity(rightTicksPerSec);
    }

    /**
     * Set motor powers (0.0 to 1.0)
     * @param leftMotorPower Power for left motor (-1.0 to 1.0)
     * @param rightMotorPower Power for right motor (-1.0 to 1.0)
     */
    public void setMotorPowers(double leftMotorPower, double rightMotorPower) {
        leftLauncher.setPower(leftMotorPower);
        rightLauncher.setPower(rightMotorPower);
    }

    /**
     * Stop both motors
     */
    public void stopMotors() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }

    /**
     * Get left motor velocity in RPM
     * @return Current velocity in RPM
     */
    public double getLeftMotorVelocityRPM() {
        return ticksPerSecondToRPM(leftLauncher.getVelocity());
    }

    /**
     * Get right motor velocity in RPM
     * @return Current velocity in RPM
     */
    public double getRightMotorVelocityRPM() {
        return ticksPerSecondToRPM(rightLauncher.getVelocity());
    }

    /**
     * Get left motor velocity in raw encoder ticks per second
     * (Legacy method for backward compatibility)
     * @return Current velocity in ticks/second
     */
    public double getLeftMotorVelocity() {
        return leftLauncher.getVelocity();
    }

    /**
     * Get right motor velocity in raw encoder ticks per second
     * (Legacy method for backward compatibility)
     * @return Current velocity in ticks/second
     */
    public double getRightMotorVelocity() {
        return rightLauncher.getVelocity();
    }

    /**
     * Get the maximum achievable RPM for these motors
     * @return Maximum RPM specification
     */
    public double getMaxRPM() {
        return MAX_RPM;
    }

    /**
     * Get the encoder counts per revolution constant
     * @return Encoder ticks per motor revolution
     */
    public double getCountsPerRevolution() {
        return COUNTS_PER_REVOLUTION;
    }
}   //end program