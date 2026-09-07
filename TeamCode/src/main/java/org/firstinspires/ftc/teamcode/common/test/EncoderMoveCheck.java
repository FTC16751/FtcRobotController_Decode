package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.DriveUtil2026b;
import org.firstinspires.ftc.teamcode.common.RobotConfig;

/**
 * One-button checks for the encoder moves in DriveUtil2026b, and the measurements that calibrate
 * them. Put the robot at a tape line, press a button, measure what it did, read the telemetry.
 *
 * This class is abstract because Common never names a specific robot. Each team has a tiny
 * subclass that supplies its RobotConfig (see StandardBotEncoderMoveCheck for the pattern).
 *
 * Buttons (gamepad 1):
 *   Y / A            forward / backward 24 in            (drive_p3)
 *   X / B            strafe left / right 24 in           (drive_p3)
 *   D-pad left/right turn 90 deg left / right           (turnLeft / turnRight, the beginner commands)
 *   D-pad up         turn 360 deg CW                     (drive_p3, measures the turning circle)
 *   D-pad down       turn 180 deg CW                     (drive_p3, for when there is no room for a 360)
 *   Left/right bumper  forward / backward 12 in          (driveForward / driveBackward, the beginner commands)
 *   Left/right trigger speed down / up by 0.1
 *
 * What to write down after each move is in the telemetry: whether the move reached its target or
 * timed out, how long it took, the encoder ticks on each wheel, and the wheel travel those ticks
 * mean under the current calibration. Compare that to the tape measure. The calibration formulas
 * are in doc/ROBOT_TEST_PLAN.md section F.
 *
 * Every move here blocks until it finishes, times out, or the OpMode is stopped. That is the
 * point: this OpMode is where a stalled wheel or a Stop press gets tried on a stand.
 */
public abstract class EncoderMoveCheck extends LinearOpMode {

    /** The config for the chassis this OpMode is deployed to. */
    protected abstract RobotConfig robotConfig();

    private static final double SPEED_MIN  = 0.1;
    private static final double SPEED_MAX  = 0.8;
    private static final double SPEED_STEP = 0.1;
    private static final double MOVE_IN    = 24.0;   // distance for the drive_p3 moves
    private static final double SHORT_IN   = 12.0;   // distance for the driveRobotDistance*Inches moves

    /** One blocking move; returns true if the motors reached their targets. */
    private interface Move { boolean go(); }

    private DriveUtil2026b drive;
    private RobotConfig config;
    private double speed = 0.4;

    // What the last move did, for the telemetry
    private String  lastMove    = "none yet";
    private String  lastResult  = "";
    private double  lastSeconds = 0.0;
    private int     ticksLF, ticksRF, ticksLR, ticksRR;

    private boolean leftTriggerWasDown  = false;
    private boolean rightTriggerWasDown = false;

    @Override
    public void runOpMode() {
        config = robotConfig();
        drive  = new DriveUtil2026b(hardwareMap, telemetry, null, config);

        telemetry.addData("robot config", config.robotName);
        telemetry.addLine("Put the robot at a tape line, then press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            readSpeedButtons();

            if (gamepad1.yWasPressed())         run("forward 24 in (drive_p3)",        () -> drive.drive_p3( MOVE_IN, 0, 0, speed));
            if (gamepad1.aWasPressed())         run("backward 24 in (drive_p3)",       () -> drive.drive_p3(-MOVE_IN, 0, 0, speed));
            if (gamepad1.xWasPressed())         run("strafe left 24 in (drive_p3)",    () -> drive.drive_p3(0, -MOVE_IN, 0, speed));
            if (gamepad1.bWasPressed())         run("strafe right 24 in (drive_p3)",   () -> drive.drive_p3(0,  MOVE_IN, 0, speed));
            if (gamepad1.dpadLeftWasPressed())  run("turn 90 left (turnLeft)",         () -> drive.turnLeft(90, speed));
            if (gamepad1.dpadRightWasPressed()) run("turn 90 right (turnRight)",       () -> drive.turnRight(90, speed));
            if (gamepad1.dpadUpWasPressed())    run("turn 360 CW (drive_p3)",          () -> drive.drive_p3(0, 0, 360, speed));
            if (gamepad1.dpadDownWasPressed())  run("turn 180 CW (drive_p3)",          () -> drive.drive_p3(0, 0, 180, speed));
            if (gamepad1.leftBumperWasPressed())  run("forward 12 in (driveForward)",       () -> drive.driveForward(SHORT_IN, speed));
            if (gamepad1.rightBumperWasPressed()) run("backward 12 in (driveBackward)",     () -> drive.driveBackward(SHORT_IN, speed));

            showTelemetry();
            sleep(20);
        }
        drive.stopRobot();
    }

    private void readSpeedButtons() {
        boolean leftDown  = gamepad1.left_trigger  > 0.5;
        boolean rightDown = gamepad1.right_trigger > 0.5;
        if (leftDown  && !leftTriggerWasDown)  speed = Math.max(SPEED_MIN, speed - SPEED_STEP);
        if (rightDown && !rightTriggerWasDown) speed = Math.min(SPEED_MAX, speed + SPEED_STEP);
        leftTriggerWasDown  = leftDown;
        rightTriggerWasDown = rightDown;
    }

    private void run(String name, Move move) {
        lastMove = name;
        telemetry.addData("running", name);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        boolean reached = move.go();
        lastSeconds = timer.seconds();
        lastResult  = reached ? "reached target" : "TIMED OUT or stopped";

        // driveRobotToPosition resets the encoders before each move, so these are this move's ticks
        ticksLF = drive.leftFrontMotor.getCurrentPosition();
        ticksRF = drive.rightFrontMotor.getCurrentPosition();
        ticksLR = drive.leftRearMotor.getCurrentPosition();
        ticksRR = drive.rightRearMotor.getCurrentPosition();
    }

    private void showTelemetry() {
        RobotConfig.Calibration cal = config.calibration;
        double avgTicks = (Math.abs(ticksLF) + Math.abs(ticksRF) + Math.abs(ticksLR) + Math.abs(ticksRR)) / 4.0;

        telemetry.addData("robot config", config.robotName);
        telemetry.addData("speed", "%.1f  (triggers change it)", speed);
        telemetry.addLine();
        telemetry.addData("last move", lastMove);
        telemetry.addData("result", "%s in %.2f s", lastResult, lastSeconds);
        telemetry.addData("ticks LF RF LR RR", "%d %d %d %d", ticksLF, ticksRF, ticksLR, ticksRR);
        telemetry.addData("wheel travel per calibration", "%.1f in", avgTicks / cal.encoderCountsPerInch);
        telemetry.addLine();
        telemetry.addData("calibration in use", "ticks/in %.2f  strafe x%.2f  turnCirc %.1f in",
                cal.encoderCountsPerInch, cal.strafeScale, cal.turnCircumferenceIn);
        telemetry.addLine();
        telemetry.addLine("Y/A fwd/back 24   X/B strafe L/R 24   dpad L/R turn 90   dpad up 360   dpad down 180");
        telemetry.addLine("LB/RB fwd/back 12 in via driveForward/driveBackward   LT/RT speed");
        telemetry.update();
    }
}
