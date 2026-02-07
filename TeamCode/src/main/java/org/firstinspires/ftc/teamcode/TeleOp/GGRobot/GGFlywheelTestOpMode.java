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


package org.firstinspires.ftc.teamcode.TeleOp.GGRobot;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LauncherMotors;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;

/**
 * Test Flywheels
 *
 */
@TeleOp(name = "GG Test Flywheels", group = " _GGopmodes")
@Disabled
//@Disabled
public class GGFlywheelTestOpMode extends OpMode {
    //Declare SubSystems
    private LauncherMotors launcher;

    double finalTargetVelocity = 1200;
    private static final double VELOCITY_INCREMENT = 25; // 25 RPM increments
    private boolean isRunning = false; // Track whether flywheels are running

    @Override
    public void init() {
        // --- ROBOT ---
        launcher = new LauncherMotors(hardwareMap);
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
        handleLauncherControls();
        displayTelemetry();
    }

    private void handleLauncherControls() {
        // Press 'Y' to activate the launcher, 'B' to deactivate it.
        if (gamepad1.yWasPressed()) {
            launcher.setMotorVelocity(finalTargetVelocity, finalTargetVelocity);
            isRunning = true;
        }
        if (gamepad1.bWasPressed()) {
            launcher.setMotorVelocity(0, 0);
            isRunning = false;
        }

        // Dpad Up to increase velocity by 25 RPM
        if (gamepad1.dpad_up) {
            finalTargetVelocity += VELOCITY_INCREMENT;
            // Update motors if they're currently running
            if (isRunning) {
                launcher.setMotorVelocity(finalTargetVelocity, finalTargetVelocity);
            }
        }

        // Dpad Down to decrease velocity by 25 RPM
        if (gamepad1.dpad_down) {
            finalTargetVelocity -= VELOCITY_INCREMENT;
            // Prevent negative velocities
            if (finalTargetVelocity < 0) {
                finalTargetVelocity = 0;
            }
            // Update motors if they're currently running
            if (isRunning) {
                launcher.setMotorVelocity(finalTargetVelocity, finalTargetVelocity);
            }
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("--- Launcher ---");
        telemetry.addLine("--- Flywheels ---");
        telemetry.addData("Status", isRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Target Velocity", "%.0f RPM", finalTargetVelocity);
        telemetry.addData("Left Velocity", "%.2f", launcher.getLeftMotorVelocity());
        telemetry.addData("Right Velocity", "%.2f", launcher.getRightMotorVelocity());
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Y: Start flywheels");
        telemetry.addLine("  B: Stop flywheels");
        telemetry.addLine("  DPAD UP: +25 RPM");
        telemetry.addLine("  DPAD DOWN: -25 RPM");
        telemetry.update();
    }
}