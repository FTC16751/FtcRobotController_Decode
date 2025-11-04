package org.firstinspires.ftc.teamcode.Auto.GearGirls;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.AutoAction;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.VisionUtil;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="GG Pinpoint Action xy tet", group="GGBot")
public class GGAuto_TestPinpoint_XY_Movements extends OpMode {

    // --- Subsystems ---
    private GGRobot robot;

    // --- State Machine and Action Playlist ---
    // With this new logic, the state machine becomes incredibly simple!
    private enum AutonomousState {
        EXECUTE_SEQUENCE, // The one and only active state
        DRIVE_TO_TARGET_1, DRIVE_TO_TARGET_2, DRIVE_TO_TARGET_3, DRIVE_TO_TARGET_4, DRIVE_TO_TARGET_5, AT_TARGET;
    }
    private AutonomousState autonomousState = AutonomousState.EXECUTE_SEQUENCE;


    // --- Game-Specific Variables ---
    final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH,12,0, AngleUnit.DEGREES,0);
    final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH,0,12, AngleUnit.DEGREES,0);
    final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90);

    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot(hardwareMap, telemetry);

        telemetry.addData(">", "Robot Initialized. Detecting Motif...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        autonomousState = AutonomousState.EXECUTE_SEQUENCE;
    }

    //================================================================================
    // MAIN LOOP
    //================================================================================

    @Override
    public void loop() {
        // These updates are always required for non-blocking parts (like the feeder)
        robot.update();
        robot.drive.pinpoint.update();

        switch (autonomousState) {
            case EXECUTE_SEQUENCE:
                    if (robot.drive.driveRelative(robot.drive.pinpoint.getPosition(),12,0,0,.5,2)){
                        autonomousState = AutonomousState.DRIVE_TO_TARGET_1;
                    };
                break;
            case DRIVE_TO_TARGET_1:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, 0), 0.5, 2)){
                    autonomousState = AutonomousState.DRIVE_TO_TARGET_2;
                }
                break;
            case DRIVE_TO_TARGET_2:
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0), 0.25, 2)){
                    autonomousState = AutonomousState.AT_TARGET;
                }
                break;
            case DRIVE_TO_TARGET_3:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(), TARGET_3, 0.5, 2)){
                    autonomousState = AutonomousState.DRIVE_TO_TARGET_4;
                }
                break;
            case DRIVE_TO_TARGET_4:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(),TARGET_4,0.5,2)){
                    autonomousState = AutonomousState.DRIVE_TO_TARGET_5;
                }
                break;
            case DRIVE_TO_TARGET_5:
                if(robot.drive.driveTo(robot.drive.pinpoint.getPosition(),TARGET_5,0.5,0)){
                    autonomousState = AutonomousState.AT_TARGET;
                }
                break;

            case AT_TARGET:
                robot.stopAll();
                requestOpModeStop(); // End the OpMode
                break;
        }

        telemetry.addData("Auto State", autonomousState);
        robot.addTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) robot.stopAll();
    }

}

