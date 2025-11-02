package org.firstinspires.ftc.teamcode.utilities.Common;

import static org.firstinspires.ftc.teamcode.utilities.Common.DriveUtil2026.DriveType.MECANUM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="Modified Pinpoint Navigation Example", group="Pinpoint")

public class GBPinpointDriveToPoint extends LinearOpMode {
    DriveUtil2026 drive;

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH,12,12,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, -12, -12, AngleUnit.DEGREES, 00);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH,-12,-12, AngleUnit.DEGREES,-90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, 90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() {
        drive = new DriveUtil2026(hardwareMap,telemetry,null);

        drive.setDriveType(MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", drive.pinpoint.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset", drive.pinpoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Device Version Number:", drive.pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", drive.pinpoint.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
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
                    if(drive.driveTo(drive.pinpoint.getPosition(), TARGET_3, 0.5, 3)){
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
                    if(drive.driveTo(drive.pinpoint.getPosition(),TARGET_5,0.5,2)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
            }

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = drive.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();

        }
    }}

