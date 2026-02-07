package org.firstinspires.ftc.teamcode.utilities.Common;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "FlywheelTunerTutorial", group = " _GGopmodes")

public class FlywheelTunerTutorial extends OpMode {
    public DcMotorEx flywheelMotor_l,flywheelMotor_r;

    public double highVelocity = 2100;
    public double lowVelocity = 1800;

    double curTargetVelocity = highVelocity;

    // Initial PIDF coefficients for tuning.
    double F = 14.098; // Feedforward gain to counteract constant forces like friction.
    double P = 265;    // Proportional gain to correct error based on how far off the velocity is.

    // Array of step sizes for making fine or coarse adjustments to P and F.
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    // Index to select the current step size from the array.
    int stepIndex = 1;


    @Override
    public void init() {
        flywheelMotor_l = hardwareMap.get(DcMotorEx.class, "left_launcher");
        flywheelMotor_r = hardwareMap.get(DcMotorEx.class, "right_launcher");
        flywheelMotor_l.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelMotor_l.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor_r.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor_l.setZeroPowerBehavior(FLOAT);
        flywheelMotor_r.setZeroPowerBehavior(FLOAT);

        flywheelMotor_l.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0, 0, 0, 0));
        flywheelMotor_r.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0, 0, 0, 0));

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // --- Gamepad Controls for Tuning ---

        // 'Y' button toggles the target velocity between the high and low presets.
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }

        // 'B' button cycles through the different step sizes for tuning precision.
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        // D-pad left/right adjusts the F (Feedforward) gain.
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        // D-pad up/down adjusts the P (Proportional) gain.
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        // Apply the new coefficients to the motor in every loop iteration.
        flywheelMotor_l.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor_r.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Command the motor to run at the current target velocity.
        flywheelMotor_l.setVelocity(curTargetVelocity);
        flywheelMotor_r.setVelocity(curTargetVelocity);

        // --- Telemetry Output ---

        double curVelocity_l = flywheelMotor_l.getVelocity();
        double error_l = curTargetVelocity - curVelocity_l;

        double curVelocity_r = flywheelMotor_r.getVelocity();
        double error_r = curTargetVelocity - curVelocity_r;


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity_l);
        telemetry.addData("Error", "%.2f", error_l);
        telemetry.addData("Current Velocity Right", "%.2f", curVelocity_r);
        telemetry.addData("Error Right", "%.2f", error_r);


        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}