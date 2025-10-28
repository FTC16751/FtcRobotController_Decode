package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.LauncherMotors;
import org.firstinspires.ftc.teamcode.utilities.DriveUtil2025;
import org.firstinspires.ftc.teamcode.utilities.IntakeUtil;
import org.firstinspires.ftc.teamcode.utilities.LaunchIndexer;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp(name = "CoachBot YEAH2", group = "Concept")
//@Disabled
public class Coachbot extends OpMode {
    private static final double DRIVE_SPEED = 0.5;
    final double FEED_TIME_SECONDS = 0.80; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1450; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;
    final double CENTER_POSITION = 0.145;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo diverter = null;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();

    private DriveUtil2025 drive;
    private IntakeUtil intake;
    private LauncherMotors launcher;

    private LaunchIndexer feeder;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT,
        CENTER;
    }
    private DiverterDirection diverterDirection = DiverterDirection.CENTER;

    private enum IntakeState {
        ON,
        OFF,
        REVERSE; // Add the new REVERSE state
    }


    private IntakeState intakeState = IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }

    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive = new DriveUtil2025(this);
        drive.init(hardwareMap,telemetry); //initialize the drive subsystem
        intake = new IntakeUtil(hardwareMap);

        launcher = new LauncherMotors(hardwareMap);

        feeder = new LaunchIndexer(hardwareMap);

        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        diverter = hardwareMap.get(Servo.class, "diverter");

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        feeder.setLeftFeederPower(STOP_SPEED);

        diverter.setPosition(CENTER_POSITION);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        drive.arcadeDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x,gamepad1.right_stick_y,DRIVE_SPEED);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            launcher.setMotorVelocity(launcherTarget, launcherTarget);
        } else if (gamepad1.b) { // stop flywheel
            launcher.setMotorVelocity(STOP_SPEED, STOP_SPEED);
        }

// Press D-pad Down to toggle between LEFT and RIGHT
        if (gamepad1.dpadDownWasPressed()) {
            // If it's currently LEFT, move to RIGHT. Otherwise, move to LEFT.
            if (diverterDirection == DiverterDirection.LEFT) {
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
            } else {
                diverterDirection = DiverterDirection.LEFT;
                diverter.setPosition(LEFT_POSITION);
            }
        }

// Press D-pad Right to explicitly move the diverter to the CENTER position
        if (gamepad1.dpadRightWasPressed()) {
            diverterDirection = DiverterDirection.CENTER;
            diverter.setPosition(CENTER_POSITION);
        }

        // --- Intake Control Logic ---

        // Press 'a' to turn the intake ON or OFF
        if (gamepad1.aWasPressed()) {
            // If it's currently ON, turn it OFF. Otherwise, turn it ON.
            if (intakeState == IntakeState.ON) {
                intakeState = IntakeState.OFF;
                intake.setIntakeMotorPower(0);
            } else {
                intakeState = IntakeState.ON;
                intake.setIntakeMotorPower(1);
            }
        }

        // Press 'x' to REVERSE the intake or turn it OFF
        if (gamepad1.xWasPressed()) {
            // If it's currently in REVERSE, turn it OFF. Otherwise, put it in REVERSE.
            if (intakeState == IntakeState.REVERSE) {
                intakeState = IntakeState.OFF;
                intake.setIntakeMotorPower(0);
            } else {
                intakeState = IntakeState.REVERSE;
                intake.setIntakeMotorPower(-1); // Use a negative value for reverse
            }
        }


        if (gamepad1.dpadUpWasPressed()) {
            switch (launcherDistance) {
                case CLOSE:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                    break;
                case FAR:
                    launcherDistance = LauncherDistance.CLOSE;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                    break;
            }
        }

        /*
         * Now we call our "Launch" function.
         */
        launchLeft(gamepad1.leftBumperWasPressed());
        launchRight(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", leftLaunchState);
        telemetry.addData("launch distance", launcherDistance);
        // telemetry.addData("Left Launcher Velocity", launcher.getLeftMotorVelocity());
        // telemetry.addData("Right Launcher Velocity", launcher.getRightMotorVelocity());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setMotorVelocity(launcherTarget, launcherTarget);
                if (launcher.getLeftMotorVelocity() > launcherMin) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feeder.setLeftFeederPower(FULL_SPEED);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftLaunchState = LaunchState.IDLE;
                    feeder.setLeftFeederPower(STOP_SPEED);
                }
                break;
        }
    }

    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setMotorVelocity(launcherTarget, launcherTarget);
                if (launcher.getLeftMotorVelocity() > launcherMin) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feeder.setRightFeederPower(FULL_SPEED);
                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightLaunchState =LaunchState.IDLE;
                    feeder.setRightFeederPower(STOP_SPEED);
                }
                break;
        }
    }
}