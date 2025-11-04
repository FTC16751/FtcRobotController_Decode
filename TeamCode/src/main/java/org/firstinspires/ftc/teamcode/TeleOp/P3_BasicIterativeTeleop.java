/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.DriveUtil2025;
import org.firstinspires.ftc.teamcode.utilities.P3_IntakeUtil;
import org.firstinspires.ftc.teamcode.utilities.P3_LauncherUtil;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: p3 opmode", group="Iterative OpMode")

public class P3_BasicIterativeTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveUtil2025 drive;
    private P3_IntakeUtil intake;
    private P3_LauncherUtil launcher;

    enum allianceColor {
        RED,
        BLUE
    }



    // TODO: set this at init
    allianceColor alliance = allianceColor.RED;

    private enum IntakeState {
        ON,
        OFF,
        REVERSE;
    }
    private IntakeState intakeState = IntakeState.OFF;
    private final double INTAKE_POWER = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        drive = new DriveUtil2025(this);
        drive.init(hardwareMap,telemetry); //initialize the drive subsystem
        intake = new P3_IntakeUtil(hardwareMap);
        launcher = new P3_LauncherUtil(hardwareMap);

        // Tell the driver that initialization is complete.
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        doDriveControls();
        handleIntakeControls();
        handleLauncherControls();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    private void doDriveControls() {
// Use scaled inputs for driving
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.right_stick_y, .85);

        //drive.arcadeDrive(strafeInput, driveInput, turnInput, gamepad1.right_stick_y, DRIVE_SPEED);

    }

    private void handleIntakeControls() {
        if (gamepad1.aWasPressed()) {
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }
        switch (intakeState) {
            case ON:
                intake.setIntakeMotorPower(INTAKE_POWER);
                break;
            case OFF:
                intake.setIntakeMotorPower(0.0);
                break;

        }
    }

    private double calcShooterVelocity() {
        Pose goal;
        switch (alliance) {
            case RED:
                goal = new Pose(140, 140);
            case BLUE:
            default:
                goal = new Pose(4, 140);
        }
        Pose robotPose = follower.getPose();
        double distance = robotPose.distanceFrom(goal);
        return 10*distance;

    }

    private void handleLauncherControls() {
        if (gamepad1.right_trigger > 0.8) {
            launcher.setIndexerServoPower(-1.0);
            launcher.setShootingPosition();
        }
        else {
            launcher.setIndexerServoPower(0.0);
            launcher.setStopPosition();
        }
        if (gamepad1.yWasPressed()) {
            launcher.setShooterMotorVelocity(1000);
            // CHANGE THIS BACK TO CALC SHOOTER VELOCITY LATER
        }
        if (gamepad1.xWasPressed()) {
            launcher.setShooterMotorVelocity(0);
        }
    }
}
