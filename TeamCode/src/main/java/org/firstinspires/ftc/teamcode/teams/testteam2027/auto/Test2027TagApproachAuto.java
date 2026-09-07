package org.firstinspires.ftc.teamcode.teams.testteam2027.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Constants;
import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Robot;

/**
 * Tests the non-blocking AprilTag approach (DriveUtil2026b.driveToTagAsync / common.TagApproach).
 *
 * Put the robot about 3 ft from a wall with the test tag (Test2027Constants.TagTest.TAG_ID) taped
 * to it at camera height, roughly facing the tag. Press START. The robot should end the standoff
 * distance from the wall, centered on the tag and square to it, and telemetry should say DONE.
 * Cover the camera mid-approach and it should coast briefly, then stop and report LOST.
 *
 * The shape of this auto is the pattern for any async drive step: one state starts it, the next
 * state waits on isBusy() while other subsystems keep running in the same loop, then it moves on.
 */
@Autonomous(name = "Test2027: Tag Approach Test", group = "TestTeam2027", preselectTeleOp = "Test2027: Teleop (RUN ME)")
public class Test2027TagApproachAuto extends OpMode {

    private enum State { WAIT_FOR_TAG, START_APPROACH, WAIT_FOR_APPROACH, DONE }

    private Test2027Robot robot;
    private State state = State.WAIT_FOR_TAG;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private String result = "";

    @Override
    public void init() {
        robot = new Test2027Robot(hardwareMap, telemetry);
        robot.lookForTag(Test2027Constants.TagTest.TAG_ID);   // goal tags and motif tags are on different pipelines
        telemetry.addData("Status", "Initialized. Tag %d, standoff %.0f in",
                Test2027Constants.TagTest.TAG_ID, Test2027Constants.TagTest.STANDOFF_INCHES);
    }

    @Override
    public void init_loop() {
        robot.update();
        telemetry.addData("tag in view now", robot.vision.canSee(Test2027Constants.TagTest.TAG_ID) ? "YES" : "no");
        robot.addTelemetry();
    }

    @Override
    public void start() {
        state = State.WAIT_FOR_TAG;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        robot.update();   // steps the camera and the async approach

        switch (state) {
            case WAIT_FOR_TAG:
                // Give the camera up to 2 s to find the tag before starting; the approach itself
                // also tolerates a lost tag, so this is only to make the telemetry easy to read.
                if (robot.vision.canSee(Test2027Constants.TagTest.TAG_ID) || stateTimer.seconds() > 2.0) {
                    state = State.START_APPROACH;
                }
                break;

            case START_APPROACH:
                robot.drive.driveToTagAsync(robot.vision,
                        Test2027Constants.TagTest.TAG_ID,
                        Test2027Constants.TagTest.STANDOFF_INCHES,
                        Test2027Constants.TagTest.HOLD_SECONDS);
                stateTimer.reset();
                state = State.WAIT_FOR_APPROACH;
                break;

            case WAIT_FOR_APPROACH:
                // Other subsystems would run here too (spin up a launcher, run an intake, ...).
                if (!robot.drive.isBusy()) {
                    result = robot.drive.lastTagApproachSucceeded()
                            ? String.format("DONE in %.1f s", stateTimer.seconds())
                            : "GAVE UP: " + robot.drive.getTagApproach().getState();
                    state = State.DONE;
                }
                break;

            case DONE:
                robot.drive.stopRobot();
                break;
        }

        telemetry.addData("auto state", state);
        telemetry.addData("result", result);
        robot.addTelemetry();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
