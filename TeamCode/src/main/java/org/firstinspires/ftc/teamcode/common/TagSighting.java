package org.firstinspires.ftc.teamcode.common;

/**
 * Where one AprilTag is relative to the robot, right now, in the robot's own frame.
 *
 * This is the seam between TagApproach (pure math, unit-tested) and the camera. VisionUtil will
 * implement it by converting the Limelight's tag-space pose into these three robot-frame numbers;
 * the tests implement it with a fake. Getting the signs of that conversion right is the one thing
 * that must be checked on a robot (see doc/ROBOT_TEST_PLAN.md), which is why the conversion lives
 * in the adapter and not here.
 *
 * Conventions, chosen to line up with DriveUtil2026b.moveRobot (drive +forward, strafe +right,
 * yaw +clockwise):
 */
public interface TagSighting {

    /** True if the tag with this id is in view on this loop. The other three values are only valid when this is true. */
    boolean canSee(int tagId);

    /** Distance from the robot to the tag's face along the robot's forward axis, in inches. Positive = the tag is ahead. */
    double forwardInches();

    /** Sideways offset of the tag's center from the robot's centerline, in inches. Positive = the tag is to the robot's RIGHT. */
    double rightInches();

    /** How far the robot must turn to face the tag squarely, in degrees. Positive = turn LEFT (counter-clockwise). */
    double squareUpDegrees();
}
