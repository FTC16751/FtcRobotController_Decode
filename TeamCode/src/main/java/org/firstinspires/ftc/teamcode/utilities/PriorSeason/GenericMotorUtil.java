package org.firstinspires.ftc.teamcode.utilities.PriorSeason;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GenericMotorUtil {
    private final LinearOpMode myOpMode; // Made final
    private DcMotor motor1;
    private HardwareMap hardwareMap; // Keep as is, or make final if initialized in constructor

    public GenericMotorUtil(LinearOpMode opmode, HardwareMap hwMap) {
        this.myOpMode = opmode;
        this.hardwareMap = hwMap;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "Elbow");
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopMotors() {
        motor1.setPower(0);
    }

    public void setMotorPowers(double m1) {
        motor1.setPower(m1);
    }



}   //end program