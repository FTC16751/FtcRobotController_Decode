package org.firstinspires.ftc.teamcode.common;

/**
 * What the aiming helpers (FlywheelVelocityModel, AimLed, VisionAim) need to know about the goal.
 * VisionUtil implements it on the robot; unit tests supply a fake that reports whatever they want.
 */
public interface AimTarget {
    /** True if the goal's AprilTag is currently in view. The other two values are only meaningful then. */
    boolean isTargetVisible();

    /** Straight-line distance from the camera to the tag, in inches. */
    double getDistanceToTagInches();

    /** Horizontal angle from the camera's center line to the tag, in degrees. Positive means the tag is to the right. */
    double getTargetAngleX();
}
