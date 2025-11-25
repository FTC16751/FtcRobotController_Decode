/*
 * Copyright (c) 2024 Phil Malone
 * ... (copyright header remains the same)
 */

package org.firstinspires.ftc.teamcode.TeleOp.Concepts;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name = "Concept: Vision Color-Locator (Circle) - Improved", group = "Concept")
public class ConceptVisionColorLocator_Circle extends LinearOpMode {
    private GGRobot robot;

    // --- 1. ENHANCED TUNING CONSTANTS for a PD Controller ---
    // Proportional gains (how aggressively it reacts to error)
    final double TURN_KP = 0.01;
    final double DRIVE_KP = 0.02;

    // Derivative gains (how aggressively it brakes to prevent overshoot)
    final double TURN_KD = 0.008;
    final double DRIVE_KD = 0.01;

    // Power limits to prevent jerky movements
    final double MAX_AUTO_TURN = 0.6;
    final double MAX_AUTO_DRIVE = 0.7;

    // Target values
    final double TARGET_X_PIXELS = 160;
    final double TARGET_RADIUS_PIXELS = 85; // Tuned for optimal intake distance

    // --- PD Controller State Variables ---
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTurnError = 0;
    private double lastDriveError = 0;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        robot = new GGRobot(hardwareMap, telemetry);

        // ... (Your ColorBlobLocatorProcessor and VisionPortal setup is excellent and remains the same)
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        waitForStart();

        timer.reset(); // Start the timer for the PD loop

        // --- Main Loop ---
        while (opModeIsActive()) {
            // --- Vision Processing ---
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blobs);

            double turnPower = 0.0;
            double drivePower = 0.0;

            // --- Control Logic ---
            if (!blobs.isEmpty()) {
                // Get the largest blob
                ColorBlobLocatorProcessor.Blob primaryBlob = blobs.get(0);
                Circle circleFit = primaryBlob.getCircle();

                // --- 2. CALCULATE POWER USING THE NEW PD CONTROLLER ---
                double loopTime = timer.seconds();
                timer.reset();

                // --- Turn Controller ---
                double turnError = TARGET_X_PIXELS - circleFit.getX();
                double turnDerivative = (turnError - lastTurnError) / loopTime;
                lastTurnError = turnError;
                double p_turn = turnError * TURN_KP;
                double d_turn = turnDerivative * TURN_KD;
                turnPower = p_turn + d_turn;

                // --- Drive Controller ---
                // Only drive forward if we are aimed reasonably straight
                if (Math.abs(turnError) < 25) { // Loosen tolerance slightly
                    double driveError = TARGET_RADIUS_PIXELS - circleFit.getRadius();
                    double driveDerivative = (driveError - lastDriveError) / loopTime;
                    lastDriveError = driveError;
                    double p_drive = driveError * DRIVE_KP;
                    double d_drive = driveDerivative * DRIVE_KD;
                    drivePower = p_drive + d_drive;
                } else {
                    lastDriveError = TARGET_RADIUS_PIXELS - circleFit.getRadius(); // Prevent derivative spike
                }

                // --- 3. CLAMP THE OUTPUT POWER ---
                turnPower = Range.clip(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                drivePower = Range.clip(drivePower, -MAX_AUTO_DRIVE, MAX_AUTO_DRIVE);

                // --- Telemetry for Tuning ---
                telemetry.addData("Turn Power", "%.2f", turnPower);
                telemetry.addData("Drive Power", "%.2f", drivePower);

            } else {
                // No blobs found, reset errors to prevent derivative spikes on re-detection.
                lastTurnError = 0;
                lastDriveError = 0;
                telemetry.addData("Status", "No valid blobs detected.");
            }

            // --- 4. UNIFIED DRIVER CONTROLS ---
            if (gamepad1.a) {
                // HOLD 'A' to activate auto-seek.
                // The drive power is inverted because a positive driveError (robot is too far)
                // should result in positive motor power (drive forward).
                // The turn power is inverted because a positive turnError (blob is on the left)
                // should result in a positive (turn left) motor command. Adjust if your robot is different.
                robot.drive.moveRobot(drivePower, 0, turnPower);
                telemetry.addData("Mode", "AUTO-SEEK");
            } else {
                // When 'A' is not held, return to full manual control.
                robot.drive.arcadeDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, 0, 1.0);
                telemetry.addData("Mode", "MANUAL");
            }

            telemetry.update();
            sleep(20); // Give the loop a short break
        }
    }
}