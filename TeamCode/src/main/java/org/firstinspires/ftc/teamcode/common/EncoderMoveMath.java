package org.firstinspires.ftc.teamcode.common;

/**
 * Encoder target arithmetic for DriveUtil2026b.drive_p3 and rotateRobot. Pure and unit-tested
 * (EncoderMoveMathTest).
 *
 * Conventions match MecanumMixer: forward +, strafe +RIGHT, turn +CLOCKWISE. The wheel order in
 * the returned array is the one driveRobotToPosition expects: left front, right front, left
 * rear, right rear.
 *
 * Each component is truncated toward zero to an int BEFORE the components are summed, exactly as
 * the original drive_p3 did, so a move is the same tick count it was before this class existed.
 */
public final class EncoderMoveMath {

    private EncoderMoveMath() {}

    /**
     * @param forwardInches       + forward, - backward
     * @param strafeInches        + right, - left
     * @param turnDegrees         + clockwise, - counter-clockwise
     * @param countsPerInch       RobotConfig.Calibration.encoderCountsPerInch
     * @param strafeScale         RobotConfig.Calibration.strafeScale (mecanum strafing slips)
     * @param turnCircumferenceIn RobotConfig.Calibration.turnCircumferenceIn (wheel travel per 360)
     * @return target ticks {leftFront, rightFront, leftRear, rightRear}
     */
    public static int[] ticksFor(double forwardInches, double strafeInches, double turnDegrees,
                                 double countsPerInch, double strafeScale, double turnCircumferenceIn) {
        int forwardTicks = (int) (forwardInches * countsPerInch);
        int strafeTicks  = (int) (strafeInches * countsPerInch * strafeScale);
        double turnDistanceInches = (turnDegrees / 360.0) * turnCircumferenceIn;
        int turnTicks    = (int) (turnDistanceInches * countsPerInch);

        int leftFront  = forwardTicks + strafeTicks + turnTicks;
        int rightFront = forwardTicks - strafeTicks - turnTicks;
        int leftRear   = forwardTicks - strafeTicks + turnTicks;
        int rightRear  = forwardTicks + strafeTicks - turnTicks;
        return new int[] {leftFront, rightFront, leftRear, rightRear};
    }
}
