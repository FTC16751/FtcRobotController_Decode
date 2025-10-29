package org.firstinspires.ftc.teamcode.utilities;


import com.qualcomm.robotcore.hardware.CRServo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Utility class for controlling the robot's intake mechanism.
 * - * This class encapsulates the logic for initializing and operating the intake motor.
 * <p>
 * This class provides a simplified interface for operating the intake motor. It handles
 * the initialization of the motor from the hardware map, including setting its
 * direction and zero power behavior. It offers methods to set the motor's power,
 * stop it, and retrieve its current status.
 */
public class IntakeUtil {
    public static final String INTAKE_MOTOR_NAME = "intake";
    private final DcMotor intakeMotor;
    private Servo diverter = null;

    public IntakeUtil(@NonNull HardwareMap hardwareMap) { // Pass HardwareMap in constructor
        //Intake Motor (new bot)
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        diverter = hardwareMap.get(Servo.class, "diverter");
        setDiverterCenter();
    }

    /**
     * Moves the diverter servo to the predefined LEFT position.
     * This is used to direct game elements to the left side of the robot intake.
     * The position value is sourced from {@link GGRobotConstants.Diverter#LEFT_POSITION}.
     */
    public void setDiverterLeft() {
        setDiverterPosition(GGRobotConstants.Diverter.LEFT_POSITION);
    }

    /**
     * Moves the diverter servo to the predefined RIGHT position.
     * This is used to direct game elements to the right side of the robot intake.
     */
    public void setDiverterRight() {
        setDiverterPosition(GGRobotConstants.Diverter.RIGHT_POSITION);
    }

    /**
     * Moves the diverter to the predefined CENTER position.
     */
    public void setDiverterCenter() {
        setDiverterPosition(GGRobotConstants.Diverter.CENTER_POSITION);
    }

    //private helper method
    private void setDiverterPosition(double position) {
        if (diverter != null) {
            diverter.setPosition(position);
        }
    }

    /**
     * Stops the intake motor by setting its power to zero.
     */
    public void setIntakeMotorPower(double power) {
        intakeMotor.setPower(power);
    }

}
