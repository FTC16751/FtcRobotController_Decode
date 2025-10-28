/*
 * Copyright (c) 2025 Base 10 Assets, LLC
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
 * Neither the name of NAME nor the names of its contributors may be used to
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

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.utilities.DriveUtil2026.DriveType.MECANUM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.DriveUtil2026;
import org.firstinspires.ftc.teamcode.utilities.GBPinpointDriveToPoint;

import java.util.Locale;


@Autonomous(name="GG Auto Pathing using Pinpoint", group="StarterBot")
//@Disabled
public class GGAutonomousPathing002 extends OpMode
{
    DriveUtil2026 drive;

    private enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5
    }
    StateMachine stateMachine;
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH,-24,0, AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, -40, -23.4, AngleUnit.DEGREES, -36.9770);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH,-26.4,-30.45, AngleUnit.DEGREES,-36.977);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, -24, -30.45, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, -24, -1, AngleUnit.DEGREES, 0);

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        drive = new DriveUtil2026(hardwareMap,telemetry,null);
        drive.setDriveType(MECANUM);

        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", drive.pinpoint.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset", drive.pinpoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Device Version Number:", drive.pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", drive.pinpoint.getYawScalar());

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        resetRuntime();
        //drive.drive_p3(12,-12,90,.5);
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        drive.pinpoint.update();

        switch (stateMachine){
            case WAITING_FOR_START:
                //the first step in the autonomous
                stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                break;
            case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the xxx.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                if (drive.driveTo(drive.pinpoint.getPosition(), TARGET_1, 0.5, 2)){
                    telemetry.addLine("at position #1!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                }
                break;
            case DRIVE_TO_TARGET_2:
                //drive to the second target
                if (drive.driveTo(drive.pinpoint.getPosition(), TARGET_2, 0.5, 2)){
                    telemetry.addLine("at position #2!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                }
                break;
            case DRIVE_TO_TARGET_3:
                if(drive.driveTo(drive.pinpoint.getPosition(), TARGET_3, 0.5, 2)){
                    telemetry.addLine("at position #3");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                }
                break;
            case DRIVE_TO_TARGET_4:
                if(drive.driveTo(drive.pinpoint.getPosition(),TARGET_4,0.5,2)){
                    telemetry.addLine("at position #4");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                }
                break;
            case DRIVE_TO_TARGET_5:
                if(drive.driveTo(drive.pinpoint.getPosition(),TARGET_5,0.5,0)){
                    telemetry.addLine("There!");
                    stateMachine = StateMachine.AT_TARGET;
                }
                break;
        }

        telemetry.addData("current state:",stateMachine);

        Pose2D pos = drive.pinpoint.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }
}



