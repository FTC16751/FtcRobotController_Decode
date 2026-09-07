package org.firstinspires.ftc.teamcode.teams.testteam2027.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Constants;
import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027Robot;

/**
 * The simplest useful TeleOp for test2027bot, and the starting point for a new team's TeleOp.
 *
 * Controls (gamepad 1):
 *   Left stick      drive (forward/back) and strafe
 *   Right stick X   turn
 *   Left bumper     hold for slow mode
 *   Right bumper    HOLD to drive to the test AprilTag (Test2027Constants.TagTest); release to cancel.
 *                   This is the interactive way to check TagApproach on a stand or the floor.
 *   Back            reset the position to (0, 0) facing 0, and set field-forward to this direction
 *
 * Stick signs: the SDK reports a pushed-forward stick as negative Y, so drive is -left_stick_y.
 * arcadeDrive's arguments are (strafe, drive, turn, unused, speed) in that order.
 */
@TeleOp(name = "Test2027: Teleop (RUN ME)", group = "TestTeam2027")
public class Test2027Teleop extends OpMode {

    private Test2027Robot robot;
    private boolean tagApproachRunning = false;

    @Override
    public void init() {
        robot = new Test2027Robot(hardwareMap, telemetry);
        robot.lookForTag(Test2027Constants.TagTest.TAG_ID);   // goal tags and motif tags are on different pipelines
        telemetry.addData("Status", "Initialized: %s", robot.config.robotName);
    }

    @Override
    public void init_loop() {
        robot.update();
        robot.addTelemetry();
    }

    @Override
    public void loop() {
        robot.update();

        handleTagApproach();
        if (!tagApproachRunning) {
            handleDriving();
        }
        if (gamepad1.backWasPressed()) {
            robot.drive.resetPosition();       // here is (0, 0) facing 0
            robot.drive.resetFieldForward();   // and this way is "forward" if field-centric is used
        }

        telemetry.addData("mode", tagApproachRunning ? "TAG APPROACH (release RB to cancel)" : "driver");
        robot.addTelemetry();
    }

    private void handleDriving() {
        double drive  = deadband(-gamepad1.left_stick_y);
        double strafe = deadband( gamepad1.left_stick_x);
        double turn   = deadband( gamepad1.right_stick_x);
        double speed  = gamepad1.left_bumper ? Test2027Constants.Drive.SLOW_SPEED
                                             : Test2027Constants.Drive.NORMAL_SPEED;
        robot.drive.arcadeDrive(strafe, drive, turn, 0, speed);
    }

    /** Right bumper held: run the async tag approach. Released, or finished: back to the driver. */
    private void handleTagApproach() {
        if (gamepad1.right_bumper) {
            if (!tagApproachRunning) {
                // Approach whatever tag the camera sees right now; the constant is the fallback.
                int tagId = robot.vision.isTargetVisible() ? robot.vision.getDetectedTagId()
                                                           : Test2027Constants.TagTest.TAG_ID;
                robot.drive.driveToTagAsync(robot.vision, tagId,
                        Test2027Constants.TagTest.STANDOFF_INCHES,
                        Test2027Constants.TagTest.HOLD_SECONDS);
                tagApproachRunning = true;
            } else if (!robot.drive.isBusy()) {
                telemetry.addData("tag approach", robot.drive.lastTagApproachSucceeded() ? "DONE" : "gave up");
                // stay in this branch (wheels stopped) until the driver releases the bumper
            }
        } else if (tagApproachRunning) {
            robot.drive.cancel();
            tagApproachRunning = false;
        }
    }

    private static double deadband(double v) {
        return Math.abs(v) < Test2027Constants.Drive.STICK_DEADBAND ? 0.0 : v;
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
