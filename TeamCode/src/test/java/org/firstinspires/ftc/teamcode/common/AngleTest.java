package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Unit tests for DriveUtil2026b.Angle.normDelta, the heading wrap used by driveTo and inBounds. */
public class AngleTest {

    private static final double EPS = 1e-12;

    @Test
    public void smallAnglesPassThrough() {
        assertEquals(0.3, DriveUtil2026b.Angle.normDelta(0.3), EPS);
        assertEquals(-0.3, DriveUtil2026b.Angle.normDelta(-0.3), EPS);
        assertEquals(0.0, DriveUtil2026b.Angle.normDelta(0.0), EPS);
    }

    @Test
    public void wrapsAboveAndBelowPi() {
        assertEquals(-0.5 * Math.PI, DriveUtil2026b.Angle.normDelta(3.5 * Math.PI), EPS);
        assertEquals(0.5 * Math.PI, DriveUtil2026b.Angle.normDelta(-3.5 * Math.PI), EPS);
    }

    @Test
    public void justOverPiWrapsToJustUnderMinusPi() {
        assertEquals(-Math.PI + 0.01, DriveUtil2026b.Angle.normDelta(Math.PI + 0.01), EPS);
    }

    @Test
    public void exactlyPiAndMinusPiAreLeftAlone() {
        assertEquals(Math.PI, DriveUtil2026b.Angle.normDelta(Math.PI), EPS);
        assertEquals(-Math.PI, DriveUtil2026b.Angle.normDelta(-Math.PI), EPS);
    }

    @Test
    public void headingErrorAcrossTheWrapIsTheShortWay() {
        // target 170 deg, current -170 deg: the short way is 20 deg clockwise (negative)
        double target = Math.toRadians(170), current = Math.toRadians(-170);
        assertEquals(Math.toRadians(-20), DriveUtil2026b.Angle.normDelta(target - current), 1e-9);
    }
}
