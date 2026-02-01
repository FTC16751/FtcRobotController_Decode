package org.firstinspires.ftc.teamcode.utilities.Skyline;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A utility class for controlling the Skyline robot's launcher flywheel motor.
 * This class encapsulates all hardware setup and control for the launcher motor.
 */
public class Skyline_LauncherUtil {

    private final DcMotorEx launcher,launcher2;

    public Skyline_LauncherUtil(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");


        // Set motor configuration
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setZeroPowerBehavior(BRAKE);

        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set custom PIDF coefficients for velocity control. This is a critical tuning step.
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

    }

    /**
     * Sets the target velocity for the launcher motor.
     * @param velocity The target velocity in ticks per second.
     */
    public void setVelocity(double velocity) {

        launcher.setVelocity(velocity);
        launcher2.setVelocity(velocity);
    }

    /**
     * Gets the current velocity of the launcher motor.
     * @return The current velocity in ticks per second.
     */
    public double getVelocity() {
        return launcher.getVelocity();
    }
}
