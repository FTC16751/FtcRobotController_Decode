package org.firstinspires.ftc.teamcode.teams.testteam2027.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Robot;

/**
 * A first autonomous. Copy this file, rename it, change the numbers.
 *
 * Every line is one move, and the robot finishes each move before starting the next. The
 * direction is in the name, distances are inches, turns are degrees, and speed comes from the
 * team's Constants unless you add a second number (for example driveForward(24, 0.3)).
 *
 * Other commands you can use here: driveBackward, strafeLeft, strafeRight, turnRight,
 * waitSeconds, stop, and driveToTag(robot.vision, tagId, standoffInches).
 */
@Autonomous(name = "Test2027: Beginner Auto (START HERE)", group = "TestTeam2027", preselectTeleOp = "Test2027: Teleop (RUN ME)")
public class Test2027BeginnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Test2027Robot robot = new Test2027Robot(hardwareMap, telemetry);
        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();
        waitForStart();

        robot.drive.driveForward(24);
        robot.drive.turnLeft(90);
        robot.drive.driveForward(12);
        robot.drive.strafeRight(12);
        robot.drive.waitSeconds(0.5);
        robot.drive.turnRight(90);
        robot.drive.driveBackward(12);
        robot.drive.stop();

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
