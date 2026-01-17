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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.SharedState;
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

@TeleOp(name = "SKYLINE: Teleop (V2 RUN ME)", group = " _SLopmodes")
//@Disabled
public class Skyline_TeleopV2 extends OpMode {
    // --- Constants for this OpMode ---
    private static final double DRIVE_SPEED = 0.80;
    private static final double FEED_TIME_SECONDS = 2.50;
    private static double LAUNCHER_TARGET_VELOCITY_NEAR = 1400;
    private static double LAUNCHER_TARGET_VELOCITY_FAR = 1680;
    private static final double LAUNCHER_MIN_VELOCITY = 1350; // Adjusted for a reasonable threshold

    // --- Main Robot Object ---
    private Skyline_Robot robot;

    double launcherVelocity = 0;
    double angleOnTarget = 0.0;
    public final double TX_ALIGN_TOLERANCE_DEG = 1.0;
    public final double SHOOTER_VELOCITY_TOLERANCE_PERCENT = 0.95;
    public static final double TX_ALIGN_KP = 0.02;

    private enum LauncherMode {
        AUTO_TARGETING, // Continuously updates velocity from vision/sensors
        MANUAL_OVERRIDE  // Velocity is set by direct button presses (A, B, D-pad)
    }
    private LauncherMode launcherMode = LauncherMode.MANUAL_OVERRIDE; // Default to manual mode

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the robot object. It handles setting up all subsystems.
        robot = new Skyline_Robot(hardwareMap, telemetry);
        // Load the alliance that was saved by the Autonomous OpMode
        CommonConstants.Alliance alliance = SharedState.alliance; // This loads the value into the static variable.
        // 3. Tell the robot to configure its vision system for that alliance.
        robot.configureVisionForTeleOp(alliance);
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
        robot.update();

        handleDriveControls();
        handleManualLauncherControls();
        handleManualFeederControls();
        handleAllianceSelectionControls();

        doTelemetry();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void handleDriveControls() {
        double driveInput  = gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;
        boolean isSnappingToTarget = gamepad1.right_stick_button && robot.vision.isTargetVisible();
        double txError = robot.vision.getTargetAngleX();;

        if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
            angleOnTarget = 0.0;
        } else {
            angleOnTarget = TX_ALIGN_KP * txError;
        }
        if (isSnappingToTarget) {
            turnInput = angleOnTarget;
            telemetry.addData("TX Align", "ON | Error: %.1f deg", txError);
        } else {
            // normal right-stick turning
            turnInput = gamepad1.right_stick_x;
        }
        // Arcade drive with inverted Y and X for standard control
        robot.drive.arcadeDrive(
                strafeInput,  // Strafing
                -driveInput, // Forward/Backward
                -turnInput, // Turning
                0, // Unused parameter
                DRIVE_SPEED
        );
    }

    private void handleManualLauncherControls() {
        telemetry.addData("Launcher Mode", launcherMode);

        // Press 'Y' to enter AUTO_TARGETING mode.
        if (gamepad1.yWasPressed()) {
            launcherMode = LauncherMode.AUTO_TARGETING;
        }

        // Pressing ANY of the manual override buttons will switch to MANUAL_OVERRIDE mode.
        if (gamepad1.a || gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed() || gamepad1.dpadLeftWasReleased() || gamepad1.dpadRightWasReleased()) {
            launcherMode = LauncherMode.MANUAL_OVERRIDE;
        }
        switch (launcherMode) {
            case AUTO_TARGETING:
                // In this mode, we continuously get the velocity from the robot's calculation.
                launcherVelocity = robot.updateAndGetTargetVelocity();
                if (SharedState.alliance == CommonConstants.Alliance.BLUE)
                {
                    launcherVelocity = launcherVelocity * 1.04;
                }
                telemetry.addData("Launcher Mode", "AUTO (Y)");
                break;

            case MANUAL_OVERRIDE:
                if (gamepad1.dpadLeftWasReleased()) {
                    // Set predefined "far" velocity
                    launcherVelocity = LAUNCHER_TARGET_VELOCITY_FAR;
                    telemetry.addData("Launcher Mode", "MANUAL (A - Far)");
                } else if (gamepad1.dpadRightWasReleased()) {
                    // Set predefined 'near' velocity
                    launcherVelocity = LAUNCHER_TARGET_VELOCITY_NEAR;
                    telemetry.addData("Launcher Mode", "MANUAL (A - Far)");
                } else if (gamepad1.a) {
                    // Stop the launcher
                    launcherVelocity = 0;
                    telemetry.addData("Launcher Mode", "MANUAL (B - Off)");
                } else if (gamepad1.dpadUpWasReleased()) {
                    // Increase the CURRENT velocity by 100
                    launcherVelocity += 100;
                    telemetry.addData("Launcher Mode", "MANUAL (+100)");
                } else if (gamepad1.dpadDownWasReleased()) {
                    // Decrease the CURRENT velocity by 100
                    launcherVelocity -= 100;
                    telemetry.addData("Launcher Mode", "MANUAL (-100)");
                }
                // If no manual override button is being pressed, 'launcherVelocity' simply
                // retains its last value, which is exactly the behavior you want.
                break;
        }
        robot.launcher.setVelocity(launcherVelocity);
    }

    private void handleManualFeederControls() {
        if (gamepad1.left_bumper) {
            robot.feeder.setPower(1.0); // Both forward
        } else if (gamepad1.right_bumper) {
            robot.feeder.setPower(-1.0); // Both reverse
        } else {
            robot.feeder.stop();
        }
    }

    private void handleAllianceSelectionControls() {
        if (gamepad1.xWasPressed()) { // Set alliance to Blue
            if (SharedState.alliance != CommonConstants.Alliance.BLUE) {
                SharedState.alliance = CommonConstants.Alliance.BLUE;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("CO-PILOT OVERRIDE: Alliance switched to BLUE");
            }
        }

        if (gamepad1.bWasPressed()) { // set alliance to Red
            if (SharedState.alliance != CommonConstants.Alliance.RED) {
                SharedState.alliance = CommonConstants.Alliance.RED;
                robot.configureVisionForTeleOp(SharedState.alliance);
                telemetry.addLine("CO-PILOT OVERRIDE: Alliance switched to RED");
            }
        }
    }
    private void doTelemetry() {
        // The robot object adds its own telemetry (like launch state)
        telemetry.addData("Launcher Velocity", robot.launcher.getVelocity());
        telemetry.addData("distance from goal: ", robot.vision.getDistanceToTagInches());
        telemetry.update();
    }
}