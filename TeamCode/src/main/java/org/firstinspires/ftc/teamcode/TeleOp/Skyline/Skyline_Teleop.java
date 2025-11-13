/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp.Skyline;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Skyline.Skyline_Robot;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "SKYLINE: Teleop (RUN ME)", group = " _SLopmodes")
//@Disabled
public class Skyline_Teleop extends OpMode {
    // --- Constants for this OpMode ---
    private static final double DRIVE_SPEED = 0.80;
    private static final double FEED_TIME_SECONDS = 2.50;
    private static final double LAUNCHER_TARGET_VELOCITY = 1400;
    private static final double LAUNCHER_MIN_VELOCITY = 1350; // Adjusted for a reasonable threshold

    // --- Main Robot Object ---
    private Skyline_Robot robot;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the robot object. It handles setting up all subsystems.
        robot = new Skyline_Robot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");;

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Can be used for pre-match selections in the future
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // Can be used to reset timers if needed
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. ALWAYS update the robot's state first
        robot.update();

        // 2. Delegate all control logic to helper methods
        handleDriveControls();
        handleManualLauncherControls();
        handleManualFeederControls();

        // 3. Call the main launch sequence state machine
        robot.launchSequence(
                gamepad1.rightBumperWasPressed(), // The trigger for the sequence
                LAUNCHER_TARGET_VELOCITY,
                LAUNCHER_MIN_VELOCITY,
                FEED_TIME_SECONDS
        );

        // 4. Display telemetry
        doTelemetry();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void handleDriveControls() {
        // Arcade drive with inverted Y and X for standard control
        robot.drive.arcadeDrive(
                gamepad1.left_stick_x,  // Strafing
                -gamepad1.left_stick_y, // Forward/Backward
                -gamepad1.right_stick_x, // Turning
                0, // Unused parameter
                DRIVE_SPEED
        );
    }

    private void handleManualLauncherControls() {
        if (gamepad1.y) {
            robot.launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            robot.launcher.setVelocity(0);
        }
    }

    private void handleManualFeederControls() {
        if (gamepad1.dpad_left) {
            robot.feeder.setPower(1.0); // Both forward
        } else if (gamepad1.dpad_right) {
            robot.feeder.setPower(-1.0); // Both reverse
        } else {
            // Stop only if no manual d-pad input is given.
            // This allows the launch sequence to control the feeders.
            robot.feeder.stop();
        }
    }

    private void doTelemetry() {
        // The robot object adds its own telemetry (like launch state)
        telemetry.addData("Launcher Velocity", robot.launcher.getVelocity());
        telemetry.update();
    }
}