/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.TeleOp.Concepts.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;


@TeleOp(name = "MechanumWheelTestDriveUtil", group = "Concepts")
@Disabled
public class MechanumWheelTestDriveUtil extends OpMode {
    //Declare SubSystems
    private DriveUtil2026b drive;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        RobotConfig config = RobotConfig.createDefaultStandardBotConfig();
        // --- DRIVE ---
        drive = new DriveUtil2026b(hardwareMap, telemetry, null, config); // Pass opMode context

        // --- Tell the driver that initialization is complete.---
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    @Override
    public void loop() {


        // --- Control Logic ---
        handleDriveControls();

        displayTelemetry();
    }


    /**
     * Controls the robot's drivetrain based on the gamepad inputs.
     * This method uses arcade drive, where the left stick controls translational movement
     * (forward/backward and strafing) and the right stick controls rotational movement (turning).
     * The inputs are scaled by the DRIVE_SPEED constant to adjust the overall speed.
     */
    private void handleDriveControls() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,gamepad1.right_stick_y,0.5);

        if (gamepad1.xWasPressed()) {
            drive.setMotorPowers(.5,0,0,0);
        } else if (gamepad1.yWasPressed()) {
            drive.setMotorPowers(0,0,0,.5);
        } else if (gamepad1.bWasPressed()) {
            drive.setMotorPowers(0,0,.5,0);
        }else if (gamepad1.aWasPressed()) {
            drive.setMotorPowers(0,.5,0,0);
        }

    }

    private void displayTelemetry() {
        telemetry.addData("--- MOTOR TESTS ---", "");
        telemetry.addData("PRESS X for left front motor", drive.getmotorPower(drive.leftFrontMotor));
        telemetry.addData("PRESS Y for right front motor", drive.getmotorPower(drive.rightFrontMotor));
        telemetry.addData("PRESS B for right rear motor", drive.getmotorPower(drive.rightRearMotor));
        telemetry.addData("PRESS A for left rear motor", drive.getmotorPower(drive.leftRearMotor));
        telemetry.addLine();
        telemetry.addData("Pleft front motor position", drive.getmotorPosition(drive.leftFrontMotor));
        telemetry.addData("right front motor position", drive.getmotorPosition(drive.rightFrontMotor));
        telemetry.addData("right rear motor position", drive.getmotorPosition(drive.rightRearMotor));
        telemetry.addData("left rear motor position", drive.getmotorPosition(drive.leftRearMotor));
        // This command sends all queued telemetry data to the Driver Station.
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}