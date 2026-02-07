package org.firstinspires.ftc.teamcode.utilities.Skyline;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderDatagram;

/**
 * A utility class for controlling the Skyline robot's launcher flywheel motor.
 * This class encapsulates all hardware setup and control for the launcher motor.
 */
public class Skyline_LauncherUtil {

    private final DcMotorEx launcher;
    private final DcMotorEx launcher2;

    public Skyline_LauncherUtil(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");


        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher2.setDirection(DcMotorEx.Direction.FORWARD);

        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set custom PIDF coefficients for velocity control. This is a critical tuning step.
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Set motor configuration
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
