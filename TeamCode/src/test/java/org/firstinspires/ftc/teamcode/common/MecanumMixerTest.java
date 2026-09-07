package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for MecanumMixer: the statement of what forward, right, and clockwise mean on this
 * code base, and proof that fieldCentricDrive and driveTo do the same rotation.
 */
public class MecanumMixerTest {

    private static final double EPS = 1e-9;

    private static void assertPowers(MecanumMixer.Powers p, double lf, double rf, double lr, double rr) {
        assertEquals("leftFront",  lf, p.leftFront,  EPS);
        assertEquals("rightFront", rf, p.rightFront, EPS);
        assertEquals("leftRear",   lr, p.leftRear,   EPS);
        assertEquals("rightRear",  rr, p.rightRear,  EPS);
    }

    // --- mix: the sign conventions ---

    @Test
    public void forwardDrivesAllFourWheelsForward() {
        assertPowers(MecanumMixer.mix(0.5, 0, 0, 1.0), 0.5, 0.5, 0.5, 0.5);
    }

    @Test
    public void strafeRightIsTheDiagonalPattern() {
        // front-left and rear-right forward, the other two back
        assertPowers(MecanumMixer.mix(0, 0.5, 0, 1.0), 0.5, -0.5, -0.5, 0.5);
    }

    @Test
    public void yawPositiveIsClockwise() {
        // left side forward, right side back
        assertPowers(MecanumMixer.mix(0, 0, 0.5, 1.0), 0.5, -0.5, 0.5, -0.5);
    }

    @Test
    public void zeroInIsZeroOut() {
        assertPowers(MecanumMixer.mix(0, 0, 0, 1.15), 0, 0, 0, 0);
    }

    // --- mix: normalization ---

    @Test
    public void overOneIsScaledDownTogether() {
        // drive 1 + strafe 1 wants LF 2, RF 0, LR 0, RR 2; scaled by 2
        assertPowers(MecanumMixer.mix(1, 1, 0, 1.0), 1, 0, 0, 1);
    }

    @Test
    public void underOneIsNotTouched() {
        assertPowers(MecanumMixer.mix(0.3, 0.2, 0.1, 1.0), 0.6, 0.0, 0.2, 0.4);
    }

    // --- mix: right-rear scale ---

    @Test
    public void rightRearScaleBoostsThatWheelAtPartialPower() {
        assertPowers(MecanumMixer.mix(0.5, 0, 0, 1.15), 0.5, 0.5, 0.5, 0.575);
    }

    @Test
    public void rightRearScaleAtFullStickCapsTheOtherThreeInstead() {
        // 1, 1, 1, 1.15 -> normalized by 1.15: the correction is relative, top speed drops
        assertPowers(MecanumMixer.mix(1, 0, 0, 1.15), 1 / 1.15, 1 / 1.15, 1 / 1.15, 1.0);
    }

    @Test
    public void scaleOfOneIsNoCorrection() {
        assertPowers(MecanumMixer.mix(1, 0, 0, 1.0), 1, 1, 1, 1);
    }

    // --- fieldToRobot ---

    @Test
    public void headingZeroIsIdentity() {
        MecanumMixer.Command c = MecanumMixer.fieldToRobot(0.7, 0.2, 0);
        assertEquals(0.7, c.drive, EPS);
        assertEquals(-0.2, c.strafeRight, EPS);   // field-left 0.2 is strafe-right -0.2
    }

    @Test
    public void facingLeftFieldForwardBecomesStrafeRight() {
        MecanumMixer.Command c = MecanumMixer.fieldToRobot(1, 0, Math.PI / 2);
        assertEquals(0.0, c.drive, EPS);
        assertEquals(1.0, c.strafeRight, EPS);
    }

    @Test
    public void facingLeftFieldLeftBecomesForward() {
        MecanumMixer.Command c = MecanumMixer.fieldToRobot(0, 1, Math.PI / 2);
        assertEquals(1.0, c.drive, EPS);
        assertEquals(0.0, c.strafeRight, EPS);
    }

    @Test
    public void facingBackwardFieldForwardBecomesReverse() {
        MecanumMixer.Command c = MecanumMixer.fieldToRobot(1, 0, Math.PI);
        assertEquals(-1.0, c.drive, EPS);
        assertEquals(0.0, c.strafeRight, EPS);
    }

    @Test
    public void rotationPreservesMagnitude() {
        for (double h = -3.0; h <= 3.0; h += 0.37) {
            MecanumMixer.Command c = MecanumMixer.fieldToRobot(0.6, -0.3, h);
            assertEquals(Math.hypot(0.6, 0.3), Math.hypot(c.drive, c.strafeRight), EPS);
        }
    }

    /** The formula fieldCentricDrive used before 2026-09-07, with a right-positive stick strafe. */
    @Test
    public void matchesTheOldFieldCentricDriveFormula() {
        for (double h = -3.0; h <= 3.0; h += 0.31) {
            double strafe = 0.4, drive = -0.7;
            double rotatedX = strafe * Math.cos(-h) - drive * Math.sin(-h);
            double rotatedY = strafe * Math.sin(-h) + drive * Math.cos(-h);
            MecanumMixer.Command c = MecanumMixer.fieldToRobot(drive, -strafe, h);
            assertEquals("drive at h=" + h,  rotatedY, c.drive,       EPS);
            assertEquals("strafe at h=" + h, rotatedX, c.strafeRight, EPS);
        }
    }

    /** The formula driveTo used before 2026-09-07, with field x/y PID outputs. */
    @Test
    public void matchesTheOldDriveToFormula() {
        for (double h = -3.0; h <= 3.0; h += 0.29) {
            double xPWR = 0.55, yPWR = -0.25;
            double xOutput = (xPWR * Math.cos(h)) + (yPWR * Math.sin(h));
            double yOutput = (xPWR * Math.sin(h)) - (yPWR * Math.cos(h));
            MecanumMixer.Command c = MecanumMixer.fieldToRobot(xPWR, yPWR, h);
            assertEquals("drive at h=" + h,  xOutput, c.drive,       EPS);
            assertEquals("strafe at h=" + h, yOutput, c.strafeRight, EPS);
        }
    }
}
