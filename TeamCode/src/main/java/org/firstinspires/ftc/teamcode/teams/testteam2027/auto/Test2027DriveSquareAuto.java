package org.firstinspires.ftc.teamcode.teams.testteam2027.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Constants;
import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Robot;

/**
 * Drives a 24 in square using Pinpoint waypoints, then turns a quarter turn. This is the
 * INTERMEDIATE pattern: the robot knows where it is, and moves are started, not waited on.
 *
 * Each waypoint is one startDriveTo; the auto then polls isBusy() each loop. While it waits,
 * anything else in loop() keeps running (a launcher spinning up, an intake, telemetry). Every
 * start* move has its own time limit, so a bad tuning cannot stall the auto; lastMoveSucceeded()
 * says whether the robot arrived or gave up.
 *
 * It is also the fastest way to prove a new robot's Pinpoint pod directions, offsets, and
 * point-to-point tuning: the robot should come back to its start mark within about an inch.
 */
@Autonomous(name = "Test2027: Drive Square (Pinpoint)", group = "TestTeam2027", preselectTeleOp = "Test2027: Teleop (RUN ME)")
public class Test2027DriveSquareAuto extends OpMode {

    private static final Pose2D[] PATH = {
            Test2027Constants.Waypoints.CORNER_1,
            Test2027Constants.Waypoints.CORNER_2,
            Test2027Constants.Waypoints.CORNER_3,
            Test2027Constants.Waypoints.FINISH,
    };

    private enum State { NEXT_WAYPOINT, DRIVING, DONE }

    private Test2027Robot robot;
    private State state = State.NEXT_WAYPOINT;
    private int nextWaypoint = 0;
    private int gaveUp = 0;

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
        robot.drive.resetPosition();   // wherever we are is (0, 0) facing 0
        nextWaypoint = 0;
        gaveUp = 0;
        state = State.NEXT_WAYPOINT;
    }

    @Override
    public void loop() {
        robot.update();   // steps the Pinpoint and whatever move is running

        switch (state) {
            case NEXT_WAYPOINT:
                if (nextWaypoint < PATH.length) {
                    robot.drive.startDriveTo(PATH[nextWaypoint]);
                    nextWaypoint++;
                    state = State.DRIVING;
                } else {
                    state = State.DONE;
                }
                break;

            case DRIVING:
                // A launcher or intake would run here too.
                if (!robot.drive.isBusy()) {
                    if (!robot.drive.lastMoveSucceeded()) gaveUp++;
                    state = State.NEXT_WAYPOINT;
                }
                break;

            case DONE:
                robot.drive.stop();
                break;
        }

        telemetry.addData("state", "%s, waypoint %d of %d", state, Math.min(nextWaypoint, PATH.length), PATH.length);
        telemetry.addData("position", "x %.1f  y %.1f  heading %.0f", robot.drive.getX(), robot.drive.getY(), robot.drive.getHeadingDegrees());
        telemetry.addData("waypoints that gave up", gaveUp);
        robot.addTelemetry();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
