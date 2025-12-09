package org.firstinspires.ftc.teamcode.TeleOp.Concepts;



import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;
import org.firstinspires.ftc.teamcode.utilities.Common.EncoderOdometry;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.SharedState;
//added for camera tracking of game balls
import android.graphics.Color;
import android.util.Size;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@TeleOp(name="Coach Pedro Path DrivetrainTest2 ", group="Concept")
public class PedroPathTeleopCoachTestDriveOnly extends OpMode{
    private static final double WHEEL_DIAMETER_IN = 9.6/2.54;
    private static final double TRACK_WIDTH = 17.5;
    private static final double LATERAL_MULTIPLIER =1.0;
    private static final double TICKS_PER_REV = 537;
    private GGRobot robot;
    private Drivetrain pedro;
    private Follower follower;
    private EncoderOdometry odom;
    //ROBOT VARIABLES


    // DRIVE VARIABLES
    double driveCoefficient = 1.0;
    private enum DriveMode { FIELD_CENTRIC, ARCADE }
    private DriveMode DRIVEMODE = DriveMode.ARCADE;
    private static final double TX_ALIGN_KP = 0.02;
    private static final double TX_ALIGN_TOLERANCE_DEG = 1.0;

    //DIVERTER VARIABLES
    private enum DiverterDirection {
        LEFT,
        RIGHT,
        CENTER;
    } private DiverterDirection diverterDirection = DiverterDirection.CENTER;


    //INTAKE VARIABLES
    private enum IntakeState {
        ON,
        OFF,
        REVERSE; // Add the new REVERSE state
    } private IntakeState intakeState = IntakeState.OFF;

    //SHOOTER/LAUNCHER VARIABLES
    private GGRobotConstants.LauncherDistance launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;
    private GGRobotConstants.LauncherSystemState launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
    private GGRobotConstants.LauncherTargetingMode targetingMode = GGRobotConstants.LauncherTargetingMode.AUTO;
    double finalTargetVelocity = 0;
    final double LAUNCHER_VELOCITY_TOLERANCE_RPM = 125.0;
    final double MINIMUM_SAFE_VELOCITY = 500.0;
    private double launcherVelocity = 0;


    //webcam for Color Detection
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor colorLocator;

    //constants for color blob and auto-turn
    private static final double BLOB_TURN_KP = 0.00008; // Proportional gain, needs tuning
    private static final double BLOB_TURN_TOLERANCE_PIXELS = 15; // How close to center is "good enough"
    private static final double TARGET_X_PIXELS = 160; // Center of a 320px wide image



    @Override
    public void init() {
        // --- ROBOT ---
        robot = new GGRobot(hardwareMap, telemetry);
        // Load the alliance that was saved by the Autonomous OpMode
        CommonConstants.Alliance alliance = SharedState.alliance; // This loads the value into the static variable.
        robot.configureVisionForTeleOp(alliance);

        // --- State Initialization ---
        launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
        intakeState = IntakeState.OFF;
        diverterDirection = DiverterDirection.CENTER;
        launcherDistance = GGRobotConstants.LauncherDistance.CLOSE;

        Pose startingPose = new Pose(0,0,0);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

       pedro = new Drivetrain(gamepad1, follower, startingPose);
        driveCoefficient = 1.0;

        // Create odometry helper
        odom = new EncoderOdometry(
                robot.drive.leftFrontMotor,
                robot.drive.rightFrontMotor,
                robot.drive.leftRearMotor,
                robot.drive.rightRearMotor,
                robot.drive.imu,
                TICKS_PER_REV,
                WHEEL_DIAMETER_IN,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER
        );

        // initialize the Color Blob Vision
        //see the @conceptsVIsionColorLocator_Circle for more info
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // Set to find purple
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setCircleFitColor(Color.YELLOW) // Draw a yellow circle for debugging
                .setBlurSize(5)
                .setDilateSize(15) // These morph operations help create solid blobs
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator) // Add our color processor
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240)) // Use a low resolution for performance
                .build();
    }



    @Override
    public void start() {
        if (follower != null){
            follower.startTeleopDrive(true);
        };
    }

    @Override
    public void loop() {
        handleUpdates();

        handleDriveControls();
        handleCopilotControls();
        handleDiverterControls();
        handleIntakeControls();
        handleLauncherControls();
        displayTelemetry();
// --- Update odometry and show pose ---
        odom.update();

        Pose2D pose = odom.getPose();

        double xIn = pose.getX(DistanceUnit.INCH);
        double yIn = pose.getY(DistanceUnit.INCH);
        double headingDeg = pose.getHeading(AngleUnit.DEGREES);

        telemetry.addData("X (in)", "%.2f", xIn);
        telemetry.addData("Y (in)", "%.2f", yIn);
        telemetry.addData("Heading (deg)", "%.1f", headingDeg);
        telemetry.addLine("A = heading-only reset");
        telemetry.addLine("B = snap to (0,0,0°)");
        telemetry.update();

    }

    private void handleUpdates() {
        robot.update();
        pedro.update();
        telemetry.update();
    }


    private void handleDriveControls() {
        if (gamepad1.startWasPressed()) {
            robot.drive.pinpoint.resetPosAndIMU();
        }

        //i want to use the gamepad1.back button to toggle between using drivemode of arcadeDrive and fieldCentricDrive toogle drive mode should be within this telop
        if (gamepad1.backWasPressed()) {
            toggleDriveMode();
        }

        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean isSnappingToTarget = gamepad2.dpad_up && robot.vision.isTargetVisible();

        // add logic for auto-turn for game elments here
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        //filter for noise
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 100, 20000, blobs);
        boolean isAutoTurningToBlob = gamepad2.dpad_down && !blobs.isEmpty();

        if (isSnappingToTarget) {
            //get the error directly from the vision subsystem.
            double txError = robot.vision.getTargetAngleX();

            //Check if we are already within our tolerance.
            if (Math.abs(txError) <= TX_ALIGN_TOLERANCE_DEG) {
                // We are aimed correctly, so don't turn.
                turn = 0.0;
            } else {
                // We are not aimed. Calculate the turn power using the P-controller.
                turn = TX_ALIGN_KP * txError;
            }
            telemetry.addData("TX Align", "ON | Error: %.1f deg", txError);


        }
        else if (isAutoTurningToBlob){
            // Get the largest blob FileTime.from the filtered list
            ColorBlobLocatorProcessor.Blob primaryBlob = blobs.get(0);
            Circle circleFit = primaryBlob.getCircle();
            // Calculate the error: how far the blob's center is from the screen's center
            double turnErrorPixels = TARGET_X_PIXELS - circleFit.getX();
            // Check if we are already aimed correctly
            if (Math.abs(turnErrorPixels) <= BLOB_TURN_TOLERANCE_PIXELS) {
                turn = 0.0; // We are aimed, so stop turning
            } else {
                // We are not aimed. Calculate turn power using the P-controller.
                turn = turnErrorPixels * BLOB_TURN_KP;
            }
            telemetry.addData("Align Mode", "Blob Seek | Error: %.1f px", turnErrorPixels);
        }
        else {
            // normal right-stick turning
            turn = gamepad1.right_stick_x;
            telemetry.addData("AutoAim", "OFF");
        }

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double scaledDrive = drive / denominator;
        double scaledStrafe = strafe / denominator;
        double scaledTurn = turn / denominator;

        if (DRIVEMODE == DriveMode.ARCADE) {
           // robot.drive.arcadeDrive(-scaledStrafe,-scaledDrive, -scaledTurn, 0.0,1.0);
            pedro.runPedroTeleOpDrive(scaledDrive,scaledStrafe,scaledTurn,driveCoefficient,true);

        } else if (DRIVEMODE == DriveMode.FIELD_CENTRIC) {
            //robot.drive.fieldCentricDrive(-scaledStrafe, -scaledDrive, -scaledTurn,1.0);;
            pedro.runPedroTeleOpDrive(scaledDrive,scaledStrafe,scaledTurn,driveCoefficient,false);
        }
    }

    private void toggleDriveMode() {
        if (DRIVEMODE == DriveMode.ARCADE) {
            DRIVEMODE = DriveMode.FIELD_CENTRIC;

        } else {
            DRIVEMODE = DriveMode.ARCADE;
        }
    }

    private void handleCopilotControls() {
        // --- Manual Alliance Override ---
        if (gamepad2.x) { // Using 'x' for Blue
            if (SharedState.alliance != CommonConstants.Alliance.BLUE) {
                SharedState.alliance = CommonConstants.Alliance.BLUE;
                robot.configureVisionForTeleOp(SharedState.alliance);
            }
        }

        if (gamepad2.b) { // Using 'b' for Red
            if (SharedState.alliance != CommonConstants.Alliance.RED) {
                SharedState.alliance = CommonConstants.Alliance.RED;
                robot.configureVisionForTeleOp(SharedState.alliance);
            }
        }

        if(gamepad2.start) {
            robot.resetOdometryToVision();
        }
    }

    private void handleDiverterControls() {
        // Press D-pad Down to toggle between LEFT and RIGHT
        if (gamepad1.dpadDownWasPressed()) {
            diverterDirection = (diverterDirection == DiverterDirection.LEFT) ?
                    DiverterDirection.RIGHT : DiverterDirection.LEFT;
        }
        // Press D-pad Right to center the diverter
        if (gamepad1.dpadRightWasPressed()) {
            diverterDirection = DiverterDirection.CENTER;
        }

        // Set the position based on the final state once per loop
        switch (diverterDirection) {
            case LEFT:
                robot.intake.setDiverterRight();
                break;
            case RIGHT:
                robot.intake.setDiverterLeft();
                break;
            case CENTER:
                robot.intake.setDiverterCenter();
                break;
        }
    }

    private void handleIntakeControls() {
        // Press 'a' to toggle the intake between ON and OFF.
        if (gamepad1.aWasPressed()) {
            // If the intake is ON, turn it OFF; otherwise, turn it ON.
            intakeState = (intakeState == IntakeState.ON) ? IntakeState.OFF : IntakeState.ON;
        }

        // Press 'x' to toggle the intake between REVERSE and OFF.
        if (gamepad1.xWasPressed()) {
            intakeState = (intakeState == IntakeState.REVERSE) ? IntakeState.OFF : IntakeState.REVERSE;
        }

        // Set motor power based on the state
        switch (intakeState) {
            case ON:
                robot.intake.setIntakeMotorPower(GGRobotConstants.Intake.INTAKE_SPEED);
                break;
            case REVERSE:
                robot.intake.setIntakeMotorPower(GGRobotConstants.Intake.OUTTAKE_SPEED);
                break;
            case OFF:
                robot.intake.setIntakeMotorPower(0);
                break;
        }
    }

    private void handleLauncherControls() {
        if (gamepad1.yWasPressed()) {
            launcherVelocity = 1200;
            robot.launcher.setMotorVelocity(launcherVelocity,launcherVelocity);
        } else if (gamepad1.bWasPressed()) {
            launcherVelocity = 0;
            robot.launcher.setMotorVelocity(launcherVelocity,launcherVelocity);
        } else if (gamepad1.left_trigger>.5) {
            launcherVelocity = launcherVelocity-25;
            robot.launcher.setMotorVelocity(launcherVelocity,launcherVelocity);
        } else if (gamepad1.right_trigger>.5) {
            launcherVelocity = launcherVelocity+25;
            robot.launcher.setMotorVelocity(launcherVelocity,launcherVelocity);
        }
        if (gamepad1.leftBumperWasPressed()) {
            robot.feeder.triggerLeftFeeder();
        }
        if (gamepad1.rightBumperWasPressed()) {
            robot.feeder.triggerRightFeeder();
        }
      /*
        // Press D-Pad Left to cycle between AUTO and PRESET targeting modes.
        if (gamepad1.dpadLeftWasPressed()) {
            targetingMode = (targetingMode == GGRobotConstants.LauncherTargetingMode.AUTO) ?
                    GGRobotConstants.LauncherTargetingMode.PRESET : GGRobotConstants.LauncherTargetingMode.AUTO;
        }

        // Toggle between presets CLOSE or FAR
        if (gamepad1.dpadUpWasPressed()) {
            launcherDistance = (launcherDistance == GGRobotConstants.LauncherDistance.CLOSE) ?
                    GGRobotConstants.LauncherDistance.FAR : GGRobotConstants.LauncherDistance.CLOSE;
        }

        // Press 'Y' to activate the launcher, 'B' to deactivate it.
        if (gamepad1.yWasPressed()) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.ACTIVE;
        }
        if (gamepad1.bWasPressed()) {
            launcherSystemState = GGRobotConstants.LauncherSystemState.IDLE;
        }

        // set target velocity
        finalTargetVelocity = robot.updateLauncher(launcherSystemState, targetingMode, launcherDistance);

        // --- Handle Firing Requests  ---
        boolean isSpeedCorrect = (robot.launcher.getLeftMotorVelocity() >= (finalTargetVelocity - LAUNCHER_VELOCITY_TOLERANCE_RPM));
        boolean isSpeedSafe = (robot.launcher.getLeftMotorVelocity() > MINIMUM_SAFE_VELOCITY);
        boolean isLauncherReady = (launcherSystemState == GGRobotConstants.LauncherSystemState.ACTIVE) && isSpeedCorrect && isSpeedSafe;

        if (gamepad1.left_trigger > 0.2) {
            robot.feeder.setLeftFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        } else if (gamepad1.right_trigger > 0.2) {
            robot.feeder.setRightFeederPower(GGRobotConstants.Feeder.FULL_SPEED);
        } else {
            // A shot is only allowed if the system is active AND the motors are up to speed.
            if (isLauncherReady) {
                if (gamepad1.leftBumperWasPressed()) {
                    robot.feeder.triggerLeftFeeder();
                }
                if (gamepad1.rightBumperWasPressed()) {
                    robot.feeder.triggerRightFeeder();
                }
            }
        }

       */

    }
    private void displayTelemetry() {
        telemetry.addData("DriveMode: ", DRIVEMODE);
        telemetry.addData("requested motor speed: ", launcherVelocity);
        telemetry.addData("left shooter motor speed: ", robot.launcher.getLeftMotorVelocity());
        telemetry.addData("right shooter motor speed: ", robot.launcher.getRightMotorVelocity());
        telemetry.addData("distance to tag: ", robot.vision.getDistanceToTagInches());
        if (follower != null)
        {
            telemetry.addLine("--- Pedro Pathing Coordinates (Internal) ---");
            telemetry.addData("  X (Right/Left)", "%.1f in", pedro.position.getX());
            telemetry.addData("  Y (Forward/Back)", "%.1f in", pedro.position.getY());
            telemetry.addData("  Heading (CCW)", "%.1f deg", Math.toDegrees(pedro.position.getHeading()));
            telemetry.addLine();

            Pose ftcStandardPose = pedro.position.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            telemetry.addLine("--- Pedro Pose converted to FTC Standard ---");
            telemetry.addData("  X (LEFT/Right)", "%.1f in", ftcStandardPose.getX());
            telemetry.addData("  Y (FORWARD/Back)", "%.1f in", ftcStandardPose.getY());
            telemetry.addData("  Heading (CCW)", "%.1f deg", Math.toDegrees(ftcStandardPose.getHeading()));
            telemetry.addLine();
        }
        telemetry.addLine("--- GoBilda Pinpoint Coordinates (Raw) ---");
        telemetry.addData("  X (Forward/Back)", "%.1f in", robot.drive.pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("  Y (Right/Left)", "%.1f in", robot.drive.pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("  Heading (CCW)", "%.1f deg", robot.drive.pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();

        telemetry.addLine("--- VISION ---");
        robot.vision.addTelemetry();
    }
}
