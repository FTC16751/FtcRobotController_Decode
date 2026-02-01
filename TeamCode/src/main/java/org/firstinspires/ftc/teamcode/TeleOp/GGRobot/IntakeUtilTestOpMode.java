package org.firstinspires.ftc.teamcode.TeleOp.GGRobot;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeUtilV2;

/**
 * Example OpMode demonstrating the usage of the IntakeUtil subsystem.
*/
@TeleOp(name = "GG IntakeUtil Test", group = "Test")
public class IntakeUtilTestOpMode extends OpMode {

    private IntakeUtilV2 intake;

    private enum IntakeState {
        ON,
        OFF,
        REVERSE; // Add the new REVERSE state
    } private IntakeState intakeState = IntakeState.OFF;

    @Override
    public void init() {
        // Initialize the intake subsystem
        intake = new IntakeUtilV2(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Press 'a' to toggle the intake between ON and OFF.
        if (gamepad1.aWasPressed()) {
            // If the intake is ON, turn it OFF; otherwise, turn it ON.
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }

        // Press 'x' to toggle the intake between REVERSE and OFF.
        if (gamepad1.xWasPressed()) {
            intakeState = (intakeState ==IntakeState.REVERSE) ? IntakeState.OFF : IntakeState.REVERSE;
        }

        // Set motor power based on the final state once per loop
        switch (intakeState) {
            case ON:
                intake.setIntakeMotorPower(GGRobotConstants.Intake.INTAKE_SPEED);
                break;
            case REVERSE:
                intake.setIntakeMotorPower(GGRobotConstants.Intake.OUTTAKE_SPEED);
                break;
            case OFF:
                intake.stop();
                break;
        }


        // --- Telemetry ---
        telemetry.addData("Status", intake.getStatus());
        telemetry.addData("Current Power", "%.0f%%", intake.getCurrentPower() * 100);
        telemetry.addData("Is Running", intake.isRunning() ? "YES" : "NO");
        telemetry.addData("Debug", intake.getDebugInfo());
        telemetry.addData("", ""); // Blank line
        telemetry.addData("Controls", "A=Intake/Stop | B=Outtake/Stop");
        telemetry.update();
    }


}
