package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherMotors {

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private HardwareMap hardwareMap; // Keep as is, or make final if initialized in constructor

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
        leftLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }



    public void setMotorPowers(double leftMotorPower, double rightMotorPower) {
        leftLauncher.setPower(leftMotorPower);
        rightLauncher.setPower(rightMotorPower);
    }

    public void setMotorVelocity(double leftMotor, double rightMotor) {
        leftLauncher.setVelocity(leftMotor);
        rightLauncher.setVelocity(rightMotor);
    }

    public void stopMotors() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }

    public double getLeftMotorVelocity() {
        return leftLauncher.getVelocity();
    }
    public double getRightMotorVelocity() {
        return rightLauncher.getVelocity();
    }
}   //end program