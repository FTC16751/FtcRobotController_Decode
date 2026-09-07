package org.firstinspires.ftc.teamcode.common;

/**
 * The pure math of driving a mecanum chassis. No hardware, no state; unit-tested (MecanumMixerTest).
 *
 * Sign conventions, the same everywhere in DriveUtil2026b:
 *   drive  +forward, strafe +RIGHT, yaw +CLOCKWISE (left side forward, right side back).
 *
 * DriveUtil2026b.moveRobot calls mix(); fieldCentricDrive and driveTo call fieldToRobot().
 * If a robot ever drives the wrong way on a command, this is the file to read first, and the
 * tests are the statement of what "right" means.
 */
public final class MecanumMixer {

    private MecanumMixer() {}

    /** Four wheel powers, each -1 to 1. */
    public static final class Powers {
        public final double leftFront, rightFront, leftRear, rightRear;

        Powers(double leftFront, double rightFront, double leftRear, double rightRear) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftRear = leftRear;
            this.rightRear = rightRear;
        }
    }

    /** A drive command in the robot's frame: forward and right, both -1 to 1. */
    public static final class Command {
        public final double drive, strafeRight;

        Command(double drive, double strafeRight) {
            this.drive = drive;
            this.strafeRight = strafeRight;
        }
    }

    /**
     * Mix drive, strafe and yaw into four wheel powers, scale the right-rear wheel, then if any
     * wheel would exceed 1.0 scale all four down together so the mix is preserved.
     *
     * Note the order: the right-rear scale is applied BEFORE normalization. At full stick with a
     * scale of 1.15 the other three wheels end up at 1/1.15 and the right rear at 1.0. That is a
     * relative correction (it fixes a weak wheel's share) at the cost of top speed.
     *
     * @param rightRearScale RobotConfig.Calibration.rightRearPowerScale; 1.0 for no correction
     */
    public static Powers mix(double drive, double strafe, double yaw, double rightRearScale) {
        double leftFront  = drive + strafe + yaw;
        double rightFront = drive - strafe - yaw;
        double leftRear   = drive - strafe + yaw;
        double rightRear  = (drive + strafe - yaw) * rightRearScale;

        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftRear));
        max = Math.max(max, Math.abs(rightRear));
        if (max > 1.0) {
            leftFront  /= max;
            rightFront /= max;
            leftRear   /= max;
            rightRear  /= max;
        }
        return new Powers(leftFront, rightFront, leftRear, rightRear);
    }

    /**
     * Rotate a field-frame command into the robot's frame.
     *
     * Field frame is the Pinpoint's: forward along the field X axis, LEFT along field Y, heading
     * counter-clockwise positive in radians. With the robot turned 90 degrees to the left, "go
     * field-forward" becomes "strafe right".
     *
     * @param fieldForward how hard to push along field X
     * @param fieldLeft    how hard to push along field Y (left)
     * @param headingRad   the robot's heading, counter-clockwise from field X
     */
    public static Command fieldToRobot(double fieldForward, double fieldLeft, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double drive       = fieldForward * cos + fieldLeft * sin;
        double strafeRight = fieldForward * sin - fieldLeft * cos;
        return new Command(drive, strafeRight);
    }
}
