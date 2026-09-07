package org.firstinspires.ftc.teamcode.teams.testteam2027.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Constants;
import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Robot;

/**
 * Drives a 24 in square using Pinpoint waypoints and DriveUtil2026b.driveTo, then turns a quarter
 * turn. This is the waypoint idiom every GearGirls and P3 auto is built on, in its smallest form,
 * and the fastest way to prove a new robot's Pinpoint pod directions, offsets, and point-to-point
 * tuning are right: the robot should come back to its start mark within about an inch.
 *
 * driveTo is non-blocking: call it every loop with the current position and it returns true once
 * the robot has held the target for holdTime. Each waypoint here also has a timeout so a bad
 * tuning cannot stall the auto forever.
 */
@Autonomous(name = "Test2027: Drive Square (Pinpoint)", group = "TestTeam2027", preselectTeleOp = "Test2027: Teleop (RUN ME)")
public class Test2027DriveSquareAuto extends OpMode {

    private static final Pose2D[] PATH = {
            Test2027Constants.Waypoints.CORNER_1,
            Test2027Constants.Waypoints.CORNER_2,
            Test2027Constants.Waypoints.CORNER_3,
            Test2027Constants.Waypoints.FINISH,
    };

    private Test2027Robot robot;
    private int step = 0;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private int timeouts = 0;

    @Override
    public void init() {
        robot = new Test2027Robot(hardwareMap, telemetry);
        if (!robot.drive.hasPinpoint()) {
            telemetry.addLine("This robot has no Pinpoint in its config; this auto cannot run.");
        }
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        robot.update();
        robot.addTelemetry();
    }

    @Override
    public void start() {
        robot.drive.pinpoint.setPosition(Test2027Constants.Waypoints.START);   // wherever we are is (0, 0, 0)
        step = 0;
        stepTimer.reset();
    }

    @Override
    public void loop() {
        robot.update();

        if (step < PATH.length && robot.drive.hasPinpoint()) {
            boolean arrived = robot.drive.driveTo(robot.drive.pinpoint.getPosition(), PATH[step],
                    Test2027Constants.Auto.DRIVE_POWER, Test2027Constants.Auto.HOLD_SEC);
            boolean tooLong = stepTimer.seconds() > Test2027Constants.Auto.STEP_TIMEOUT_SEC;
            if (tooLong) timeouts++;
            if (arrived || tooLong) {
                step++;
                stepTimer.reset();
            }
        } else {
            robot.drive.stopRobot();
        }

        telemetry.addData("waypoint", "%d of %d%s", Math.min(step + 1, PATH.length), PATH.length,
                step >= PATH.length ? " (finished)" : "");
        telemetry.addData("steps that timed out", timeouts);
        robot.addTelemetry();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
