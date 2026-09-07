package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class VisionAimTest {

    private static final double KP = 0.02;
    private static final double TOL = 1.0;

    @Test
    public void zeroInsideTolerance() {
        assertEquals(0.0, VisionAim.turnPower(0.0, KP, TOL), 0.0);
        assertEquals(0.0, VisionAim.turnPower(0.9, KP, TOL), 0.0);
        assertEquals(0.0, VisionAim.turnPower(-0.9, KP, TOL), 0.0);
        assertEquals(0.0, VisionAim.turnPower(TOL, KP, TOL), 0.0);   // boundary counts as on target
    }

    @Test
    public void proportionalOutsideTolerance() {
        assertEquals(0.02 * 10.0, VisionAim.turnPower(10.0, KP, TOL), 1e-12);
        assertEquals(0.02 * -10.0, VisionAim.turnPower(-10.0, KP, TOL), 1e-12);
        assertEquals(0.04 * 3.0, VisionAim.turnPower(3.0, 0.04, TOL), 1e-12); // GearGirls gain
    }

    @Test
    public void goalToTheRightTurnsRight() {
        assertTrue(VisionAim.turnPower(5.0, KP, TOL) > 0);
        assertTrue(VisionAim.turnPower(-5.0, KP, TOL) < 0);
    }

    @Test
    public void readsFromVisionAndIsZeroWhenNotVisible() {
        FakeAimTarget t = new FakeAimTarget().see(60, 10.0);
        assertEquals(0.2, VisionAim.turnPower(t, KP, TOL), 1e-12);
        assertEquals(0.0, VisionAim.turnPower(t.lose(), KP, TOL), 0.0);
        assertEquals(0.0, VisionAim.turnPower((AimTarget) null, KP, TOL), 0.0);
    }

    @Test
    public void onTargetHelper() {
        assertTrue(VisionAim.onTarget(0.5, TOL));
        assertTrue(VisionAim.onTarget(-1.0, TOL));
        assertFalse(VisionAim.onTarget(1.01, TOL));
    }
}
