package org.firstinspires.ftc.teamcode.utilities.Common;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;

/**
 * A self-contained, low-fidelity localization tracker that uses the four
 * drive motor encoders and an IMU to estimate the robot's position.
 * This is a "dead reckoning" system and is prone to drift from wheel slip,
 * but serves as an excellent "second opinion" or fallback localizer.
 */
public class EncoderOdometry {

    // --- Physical Constants (These MUST be tuned for your specific robot) ---
    // The number of encoder ticks for one full revolution of the motor.
    private final double TICKS_PER_REV;
    // The diameter of the drive wheels in inches.
    private final double WHEEL_DIAMETER_INCHES;
    // The lateral distance between the centers of the left and right wheels in inches.
    private final double TRACK_WIDTH_INCHES;
    // A tuning multiplier to correct for strafing inaccuracy due to wheel slip. Start with 1.0.
    private final double LATERAL_MULTIPLIER;

    // --- Calculated Constants ---
    private final double INCHES_PER_TICK;

    // --- Hardware and State ---
    private final DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private final IMU imu;
    private List<DcMotorEx> motors;

    // Variables to store the last known encoder positions
    private int lastLfPos, lastRfPos, lastLrPos, lastRrPos;

    // The current estimated pose of the robot
    private Pose2D currentPose;

    // heading offsest
    private double headingOffsetRad = 0.0;

    public EncoderOdometry(DcMotorEx lf, DcMotorEx rf, DcMotorEx lr, DcMotorEx rr, IMU imu,
                           double ticksPerRev, double wheelDiameter, double trackWidth, double lateralMultiplier) {
        this.leftFront = lf;
        this.rightFront = rf;
        this.leftRear = lr;
        this.rightRear = rr;
        this.motors = Arrays.asList(lf, rf, lr, rr);
        this.imu = imu;

        // Store physical properties
        this.TICKS_PER_REV = ticksPerRev;
        this.WHEEL_DIAMETER_INCHES = wheelDiameter;
        this.TRACK_WIDTH_INCHES = trackWidth;
        this.LATERAL_MULTIPLIER = lateralMultiplier;

        // Pre-calculate the conversion factor
        this.INCHES_PER_TICK = (WHEEL_DIAMETER_INCHES * Math.PI) / TICKS_PER_REV;

        // Initialize the current pose at the origin
        this.currentPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);

        // Reset the encoders and store their initial positions
        reset();
    }

    /**
     * Resets the current position to (0,0) and heading to 0, and resets the encoders.
     */
    public void reset() {
        this.currentPose = new Pose2D(DistanceUnit.INCH,0, 0,AngleUnit.DEGREES, 0);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Use RUN_WITHOUT for TeleOp control
        }

        imu.resetYaw();

        this.lastLfPos = 0;
        this.lastRfPos = 0;
        this.lastLrPos = 0;
        this.lastRrPos = 0;
    }

    /**
     * This method MUST be called in every loop cycle to update the robot's estimated position.
     */
    public void update() {
        // 1. Get Sensor Deltas
        int lfPos = leftFront.getCurrentPosition();
        int rfPos = rightFront.getCurrentPosition();
        int lrPos = leftRear.getCurrentPosition();
        int rrPos = rightRear.getCurrentPosition();

        int deltaLf = lfPos - lastLfPos;
        int deltaRf = rfPos - lastRfPos;
        int deltaLr = lrPos - lastLrPos;
        int deltaRr = rrPos - lastRrPos;

        // Raw IMU yaw
        double rawHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Apply offset so we can "re-zero" heading at any time
        double currentHeadingRad = rawHeadingRad - headingOffsetRad;

        // Optional: wrap to [-π, π] to avoid huge angles over time
        //im not sure about this one
        currentHeadingRad = Math.atan2(Math.sin(currentHeadingRad), Math.cos(currentHeadingRad));

        // forward Kinematics (Convert wheel deltas to robot-centric movement)
        // This is the core math derived from mecanum kinematics.
        double deltaForward = (deltaLf + deltaRf + deltaLr + deltaRr) / 4.0;
        double deltaStrafe = (-deltaLf + deltaRf + deltaLr - deltaRr) / 4.0;
        // Note: The turn delta can also be calculated from encoders, but using the IMU is far more accurate.

        // Convert from ticks to inches
        double deltaForwardInches = deltaForward * INCHES_PER_TICK;
        double deltaStrafeInches = deltaStrafe * INCHES_PER_TICK * LATERAL_MULTIPLIER;

        // Integrate the Pose (Add the robot's movement to the field position)
        // We need to rotate the robot-centric deltas by the robot's heading
        // to get the field-centric change in position.
        double fieldDeltaX = deltaForwardInches * Math.cos(currentHeadingRad) - deltaStrafeInches * Math.sin(currentHeadingRad);
        double fieldDeltaY = deltaForwardInches * Math.sin(currentHeadingRad) + deltaStrafeInches * Math.cos(currentHeadingRad);

        // Update the current pose
        currentPose = new Pose2D(
                DistanceUnit.INCH,
                currentPose.getX(DistanceUnit.INCH) + fieldDeltaX,
                currentPose.getY(DistanceUnit.INCH) + fieldDeltaY,
                AngleUnit.RADIANS,
                currentHeadingRad
        );

        // Store the current encoder positions for the next loop
        this.lastLfPos = lfPos;
        this.lastRfPos = rfPos;
        this.lastLrPos = lrPos;
        this.lastRrPos = rrPos;
    }

    /**
     * Gets the current estimated pose of the robot.
     * @return A Pose2D object with X and Y in inches, and heading in radians.
     */
    public Pose2D getPose() {
        return this.currentPose;
    }

    /**
     * Re-anchors the odometry mid-match to a known field pose without
     * resetting motor controllers.
     *
     * @param newXInches      known field X position (inches)
     * @param newYInches      known field Y position (inches)
     * @param newHeadingDeg   known field heading (degrees, CCW from your field zero)
     */
    public void relocalize(double newXInches, double newYInches, double newHeadingDeg) {
        // 1. Take current encoder positions as the new baseline
        lastLfPos = leftFront.getCurrentPosition();
        lastRfPos = rightFront.getCurrentPosition();
        lastLrPos = leftRear.getCurrentPosition();
        lastRrPos = rightRear.getCurrentPosition();

        // 2. Compute heading offset so IMU yaw matches our desired heading
        double rawHeadingRad    = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double desiredHeadingRad = Math.toRadians(newHeadingDeg);

        headingOffsetRad = rawHeadingRad - desiredHeadingRad;

        // 3. Snap the pose to the known field pose
        currentPose = new Pose2D(
                DistanceUnit.INCH,
                newXInches,
                newYInches,
                AngleUnit.RADIANS,
                desiredHeadingRad
        );
    }
    public void resetToOriginAtCurrentHeading() {
        double rawHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // We want the new field heading to be 0°, so:
        headingOffsetRad = rawHeadingRad - 0.0;

        lastLfPos = leftFront.getCurrentPosition();
        lastRfPos = rightFront.getCurrentPosition();
        lastLrPos = leftRear.getCurrentPosition();
        lastRrPos = rightRear.getCurrentPosition();

        currentPose = new Pose2D(
                DistanceUnit.INCH,
                0,
                0,
                AngleUnit.RADIANS,
                0
        );
    }

    /**
     * Heading-only reset: keep X/Y the same, but make the current IMU yaw
     * correspond to a heading of 0 radians in the pose.
     */
    public void resetHeadingToZeroAtCurrentYaw() {
        double rawHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // We want currentHeadingRad = rawHeadingRad - headingOffsetRad = 0
        headingOffsetRad = rawHeadingRad;

        // Keep X/Y, just set heading in pose to 0
        currentPose = new Pose2D(
                DistanceUnit.INCH,
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                0.0
        );
    }
}
