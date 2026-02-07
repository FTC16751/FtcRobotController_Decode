package org.firstinspires.ftc.teamcode.Auto.GearGirls.BOT2.old;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot2;

@Autonomous(name="GG AUTO: PID Tuner", group="GGBot")
@Disabled
public class GGAutonomous_PIDTuner extends OpMode {

    // --- Subsystems ---
    private GGRobot2 robot;

    // --- Test Mode Selection ---
    private enum TestMode {
        FORWARD,
        FORWARD_BACK,
        STRAFE_LEFT,
        STRAFE_LEFT_RIGHT,
        TURN_CCW,
        TURN_CW
    }
    private TestMode currentTestMode = TestMode.FORWARD;

    // --- Configuration Parameters ---
    private double distanceInches = 24.0;  // Default 2 feet
    private double turnDegrees = 90.0;     // Default 90 degrees
    private double motorPower = 0.5;       // Default 50% power

    // Configuration step tracking
    private enum ConfigStep {
        SELECT_MODE,
        SET_DISTANCE_TENS,
        SET_DISTANCE_ONES,
        SET_POWER_TENTHS,
        SET_POWER_HUNDREDTHS,
        READY
    }
    private ConfigStep configStep = ConfigStep.SELECT_MODE;

    // --- State Machine ---
    private enum State {
        IDLE,
        MOVE_1,
        WAIT_1,
        MOVE_2,
        COMPLETE
    }
    private State currentState = State.IDLE;

    private Pose2D startPose;
    private Pose2D targetPose1;
    private Pose2D targetPose2;

    private double waitStartTime = 0;
    private final double WAIT_TIME = 1.0; // 1 second between moves

    // Button debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastA = false;

    //================================================================================
    // INITIALIZATION
    //================================================================================

    @Override
    public void init() {
        robot = new GGRobot2(hardwareMap, telemetry);
        telemetry.addData(">", "PID Tuner Initialized");
        telemetry.addData(">", "Use D-Pad and buttons to configure");
    }

    @Override
    public void init_loop() {
        handleConfiguration();
        displayTelemetry();
    }

    @Override
    public void start() {
        // Set starting position at origin
        startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        robot.drive.pinpoint.setPosition(startPose);

        // Calculate target poses based on selected test mode
        calculateTargetPoses();

        // Start the state machine
        currentState = State.MOVE_1;
    }

    //================================================================================
    // MAIN LOOP
    //================================================================================

    @Override
    public void loop() {
        robot.update();

        switch (currentState) {
            case IDLE:
                // Waiting to start
                break;

            case MOVE_1:
                telemetry.addData("State", "Moving to first target");
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), targetPose1, motorPower, motorPower * 0.5)) {
                    // First move complete
                    if (targetPose2 != null) {
                        // Two-move test, wait before second move
                        waitStartTime = getRuntime();
                        currentState = State.WAIT_1;
                    } else {
                        // Single-move test, we're done
                        currentState = State.COMPLETE;
                    }
                }
                break;

            case WAIT_1:
                telemetry.addData("State", "Waiting before return");
                if (getRuntime() - waitStartTime >= WAIT_TIME) {
                    currentState = State.MOVE_2;
                }
                break;

            case MOVE_2:
                telemetry.addData("State", "Moving to second target");
                if (robot.drive.driveTo(robot.drive.pinpoint.getPosition(), targetPose2, motorPower, motorPower * 0.5)) {
                    // Second move complete
                    currentState = State.COMPLETE;
                }
                break;

            case COMPLETE:
                telemetry.addData("State", "TEST COMPLETE");
                robot.stopAll();
                break;
        }

        robot.addTelemetry();
        displayRunTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopAll();
        }
    }

    //================================================================================
    // CONFIGURATION METHODS
    //================================================================================

    private void handleConfiguration() {


        switch (configStep) {
            case SELECT_MODE:
                if (gamepad1.dpadUpWasPressed()) {
                    currentTestMode = TestMode.values()[(currentTestMode.ordinal() - 1 + TestMode.values().length) % TestMode.values().length];
                }
                if (gamepad1.dpadDownWasPressed()) {
                    currentTestMode = TestMode.values()[(currentTestMode.ordinal() + 1) % TestMode.values().length];
                }
                if (gamepad1.aWasPressed()) {
                    configStep = ConfigStep.SET_DISTANCE_TENS;
                }
                break;

            case SET_DISTANCE_TENS:
                int tens = (int)(distanceInches / 12);
                if (gamepad1.dpadUpWasPressed() && tens < 12) {
                    distanceInches = (tens + 1) * 12 + (distanceInches % 12);
                }
                if (gamepad1.dpadDownWasPressed() && tens > 1) {
                    distanceInches = (tens - 1) * 12 + (distanceInches % 12);
                }
                if (gamepad1.aWasPressed()) {
                    configStep = ConfigStep.SET_DISTANCE_ONES;
                }
                if (gamepad1.bWasPressed()) {
                    configStep = ConfigStep.SELECT_MODE;
                }
                break;

            case SET_DISTANCE_ONES:
                int ones = ((int)distanceInches) % 12;
                tens = (int)(distanceInches / 12);
                if (gamepad1.dpadUpWasPressed() && ones < 11) {
                    distanceInches = tens * 12 + ones + 1;
                }
                if (gamepad1.dpadDownWasPressed() && ones > 0) {
                    distanceInches = tens * 12 + ones - 1;
                }
                if (gamepad1.aWasPressed()) {
                    configStep = ConfigStep.SET_POWER_TENTHS;
                }
                if (gamepad1.bWasPressed()) {
                    configStep = ConfigStep.SET_DISTANCE_TENS;
                }
                break;

            case SET_POWER_TENTHS:
                int tenths = (int)(motorPower * 10) % 10;
                int hundredths = (int)(motorPower * 100) % 10;
                if (gamepad1.dpadUpWasPressed() && motorPower < 0.95) {
                    motorPower = Math.min(1.0, (tenths + 1) * 0.1 + hundredths * 0.01);
                }
                if (gamepad1.dpadDownWasPressed() && motorPower > 0.05) {
                    motorPower = Math.max(0.0, (tenths - 1) * 0.1 + hundredths * 0.01);
                }
                if (gamepad1.aWasPressed()) {
                    configStep = ConfigStep.SET_POWER_HUNDREDTHS;
                }
                if (gamepad1.bWasPressed()) {
                    configStep = ConfigStep.SET_DISTANCE_ONES;
                }
                break;

            case SET_POWER_HUNDREDTHS:
                tenths = (int)(motorPower * 10) % 10;
                hundredths = (int)(motorPower * 100) % 10;
                if (gamepad1.dpadUpWasPressed() && motorPower < 0.99) {
                    motorPower = Math.min(1.0, tenths * 0.1 + (hundredths + 1) * 0.01);
                }
                if (gamepad1.dpadDownWasPressed() && motorPower > 0.01) {
                    motorPower = Math.max(0.0, tenths * 0.1 + (hundredths - 1) * 0.01);
                }
                if (gamepad1.aWasPressed()) {
                    configStep = ConfigStep.READY;
                }
                if (gamepad1.bWasPressed()) {
                    configStep = ConfigStep.SET_POWER_TENTHS;
                }
                break;

            case READY:
                if (gamepad1.bWasPressed()) {
                    configStep = ConfigStep.SET_POWER_HUNDREDTHS;
                }
                break;
        }
    }

    private void calculateTargetPoses() {
        double x = 0, y = 0, heading = 0;

        switch (currentTestMode) {
            case FORWARD:
                // Drive forward (positive X)
                targetPose1 = new Pose2D(DistanceUnit.INCH, distanceInches, 0, AngleUnit.DEGREES, 0);
                targetPose2 = null; // Single move
                break;

            case FORWARD_BACK:
                // Drive forward, then back to start
                targetPose1 = new Pose2D(DistanceUnit.INCH, distanceInches, 0, AngleUnit.DEGREES, 0);
                targetPose2 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
                break;

            case STRAFE_LEFT:
                // Strafe left (positive Y)
                targetPose1 = new Pose2D(DistanceUnit.INCH, 0, distanceInches, AngleUnit.DEGREES, 0);
                targetPose2 = null; // Single move
                break;

            case STRAFE_LEFT_RIGHT:
                // Strafe left, then back to start
                targetPose1 = new Pose2D(DistanceUnit.INCH, 0, distanceInches, AngleUnit.DEGREES, 0);
                targetPose2 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
                break;

            case TURN_CCW:
                // Turn counter-clockwise (positive heading)
                targetPose1 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, turnDegrees);
                targetPose2 = null; // Single move
                break;

            case TURN_CW:
                // Turn clockwise (negative heading)
                targetPose1 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -turnDegrees);
                targetPose2 = null; // Single move
                break;
        }
    }

    //================================================================================
    // TELEMETRY METHODS
    //================================================================================

    private void displayTelemetry() {
        telemetry.addLine("=== PID TUNER CONFIGURATION ===");
        telemetry.addLine();

        switch (configStep) {
            case SELECT_MODE:
                telemetry.addLine(">>> SELECT TEST MODE <<<");
                telemetry.addData("Mode", "%s (D-Pad Up/Down)", currentTestMode);
                telemetry.addLine();
                telemetry.addLine("Press A to continue");
                break;

            case SET_DISTANCE_TENS:
                telemetry.addData("Mode", currentTestMode);
                telemetry.addLine(">>> SET DISTANCE (FEET) <<<");
                telemetry.addData("Distance", "%.0f feet %.0f inches (D-Pad Up/Down)",
                        Math.floor(distanceInches / 12), distanceInches % 12);
                telemetry.addLine();
                telemetry.addLine("Press A to continue, B to go back");
                break;

            case SET_DISTANCE_ONES:
                telemetry.addData("Mode", currentTestMode);
                telemetry.addLine(">>> SET DISTANCE (INCHES) <<<");
                telemetry.addData("Distance", "%.0f feet %.0f inches (D-Pad Up/Down)",
                        Math.floor(distanceInches / 12), distanceInches % 12);
                telemetry.addLine();
                telemetry.addLine("Press A to continue, B to go back");
                break;

            case SET_POWER_TENTHS:
                telemetry.addData("Mode", currentTestMode);
                if (isDistanceMode()) {
                    telemetry.addData("Distance", "%.0f inches", distanceInches);
                } else {
                    telemetry.addData("Turn", "%.0f degrees", turnDegrees);
                }
                telemetry.addLine(">>> SET MOTOR POWER (0.X) <<<");
                telemetry.addData("Power", "%.2f (D-Pad Up/Down)", motorPower);
                telemetry.addLine();
                telemetry.addLine("Press A to continue, B to go back");
                break;

            case SET_POWER_HUNDREDTHS:
                telemetry.addData("Mode", currentTestMode);
                if (isDistanceMode()) {
                    telemetry.addData("Distance", "%.0f inches", distanceInches);
                } else {
                    telemetry.addData("Turn", "%.0f degrees", turnDegrees);
                }
                telemetry.addLine(">>> SET MOTOR POWER (0.0X) <<<");
                telemetry.addData("Power", "%.2f (D-Pad Up/Down)", motorPower);
                telemetry.addLine();
                telemetry.addLine("Press A when ready, B to go back");
                break;

            case READY:
                telemetry.addLine(">>> CONFIGURATION COMPLETE <<<");
                telemetry.addData("Mode", currentTestMode);
                if (isDistanceMode()) {
                    telemetry.addData("Distance", "%.0f inches", distanceInches);
                } else {
                    telemetry.addData("Turn", "%.0f degrees", turnDegrees);
                }
                telemetry.addData("Power", "%.2f", motorPower);
                telemetry.addLine();
                telemetry.addLine("Press START to begin test");
                telemetry.addLine("Press B to modify settings");
                break;
        }

        telemetry.update();
    }

    private void displayRunTelemetry() {
        telemetry.addLine("=== TEST IN PROGRESS ===");
        telemetry.addData("Mode", currentTestMode);
        if (isDistanceMode()) {
            telemetry.addData("Distance", "%.0f inches", distanceInches);
        } else {
            telemetry.addData("Turn", "%.0f degrees", turnDegrees);
        }
        telemetry.addData("Power", "%.2f", motorPower);
    }

    private boolean isDistanceMode() {
        return currentTestMode == TestMode.FORWARD ||
                currentTestMode == TestMode.FORWARD_BACK ||
                currentTestMode == TestMode.STRAFE_LEFT ||
                currentTestMode == TestMode.STRAFE_LEFT_RIGHT;
    }
}