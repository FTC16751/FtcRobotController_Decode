package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertArrayEquals;

import org.junit.Test;

/**
 * Unit tests for EncoderMoveMath with the default calibration (45.33 ticks/in, strafe x1.1,
 * 27.5 in per 360). Expected values are what drive_p3 produced before the math moved here,
 * including its truncation toward zero of each component.
 */
public class EncoderMoveMathTest {

    private static final double CPI    = 45.33;
    private static final double STRAFE = 1.1;
    private static final double TURN   = 27.5;

    private static int[] ticks(double fwd, double strafe, double turn) {
        return EncoderMoveMath.ticksFor(fwd, strafe, turn, CPI, STRAFE, TURN);
    }

    @Test
    public void forwardTwelveInches() {
        // 12 * 45.33 = 543.96 -> 543 on every wheel
        assertArrayEquals(new int[] {543, 543, 543, 543}, ticks(12, 0, 0));
    }

    @Test
    public void backwardTruncatesTowardZeroLikeForward() {
        assertArrayEquals(new int[] {-543, -543, -543, -543}, ticks(-12, 0, 0));
    }

    @Test
    public void strafeRightUsesTheStrafeScaleAndTheDiagonalPattern() {
        // 12 * 45.33 * 1.1 = 598.356 -> 598; LF and RR forward, RF and LR back
        assertArrayEquals(new int[] {598, -598, -598, 598}, ticks(0, 12, 0));
    }

    @Test
    public void strafeLeftIsTheMirror() {
        assertArrayEquals(new int[] {-598, 598, 598, -598}, ticks(0, -12, 0));
    }

    @Test
    public void turnNinetyClockwiseIsAQuarterOfTheTurningCircle() {
        // 27.5 / 4 = 6.875 in * 45.33 = 311.6 -> 311; left side forward, right side back
        assertArrayEquals(new int[] {311, -311, 311, -311}, ticks(0, 0, 90));
    }

    @Test
    public void turnNinetyCounterClockwiseIsTheMirror() {
        assertArrayEquals(new int[] {-311, 311, -311, 311}, ticks(0, 0, -90));
    }

    @Test
    public void componentsAddPerWheel() {
        // 543 fwd, 598 strafe, 311 turn
        assertArrayEquals(new int[] {543 + 598 + 311, 543 - 598 - 311, 543 - 598 + 311, 543 + 598 - 311},
                ticks(12, 12, 90));
    }

    @Test
    public void zeroMoveIsZeroTicks() {
        assertArrayEquals(new int[] {0, 0, 0, 0}, ticks(0, 0, 0));
    }

    @Test
    public void calibrationChangesScaleTheAnswer() {
        // double the counts per inch, double the ticks
        assertArrayEquals(new int[] {1087, 1087, 1087, 1087},
                EncoderMoveMath.ticksFor(12, 0, 0, 2 * CPI, STRAFE, TURN));
        // a bigger turning circle needs more ticks per degree
        assertArrayEquals(new int[] {997, -997, 997, -997},
                EncoderMoveMath.ticksFor(0, 0, 90, CPI, STRAFE, 88.0));
    }
}
