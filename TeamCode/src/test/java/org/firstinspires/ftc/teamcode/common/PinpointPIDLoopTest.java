package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests for PinpointPIDLoop, the per-axis controller behind driveTo. Time is a parameter, so
 * each test just picks the timestamps it wants. Tolerance is 1.0 throughout; accel is set huge
 * where the acceleration limit is not what is being tested.
 */
public class PinpointPIDLoopTest {

    private static final double EPS = 1e-9;
    private static final double BIG_ACCEL = 1e6;
    private static final double TOL = 1.0;

    private PinpointPIDLoop pid;

    @Before
    public void setUp() {
        pid = new PinpointPIDLoop();
    }

    /** P-only step at time t. */
    private double p(double error, double pGain, double accel, double t) {
        return pid.calculateAxisPID(error, pGain, 0, 0, accel, t, TOL);
    }

    // --- first call and reset ---

    @Test
    public void firstCallOnlyRecordsAndReturnsZero() {
        assertEquals(0.0, p(10, 0.05, BIG_ACCEL, 1.0), EPS);
    }

    @Test
    public void secondCallIsProportional() {
        p(10, 0.05, BIG_ACCEL, 1.0);
        assertEquals(0.5, p(10, 0.05, BIG_ACCEL, 1.1), EPS);
    }

    @Test
    public void pidResetMakesTheNextCallAFirstCallAgain() {
        p(10, 0.05, BIG_ACCEL, 1.0);
        p(10, 0.05, BIG_ACCEL, 1.1);
        pid.pidReset();
        assertEquals(0.0, p(10, 0.05, BIG_ACCEL, 1.2), EPS);
        assertEquals(0.5, p(10, 0.05, BIG_ACCEL, 1.3), EPS);
    }

    // --- clamp ---

    @Test
    public void outputIsClampedToPlusMinusOne() {
        p(100, 0.05, BIG_ACCEL, 1.0);
        assertEquals(1.0, p(100, 0.05, BIG_ACCEL, 1.1), EPS);
        pid.pidReset();
        p(-100, 0.05, BIG_ACCEL, 1.0);
        assertEquals(-1.0, p(-100, 0.05, BIG_ACCEL, 1.1), EPS);
    }

    // --- acceleration limit ---

    @Test
    public void rampingUpIsLimitedToAccelTimesDt() {
        // wants 0.5, but accel 1.0 over dt 0.1 allows a change of 0.1 per call
        p(10, 0.05, 1.0, 1.0);
        assertEquals(0.1, p(10, 0.05, 1.0, 1.1), EPS);
        assertEquals(0.2, p(10, 0.05, 1.0, 1.2), EPS);
        assertEquals(0.3, p(10, 0.05, 1.0, 1.3), EPS);
    }

    @Test
    public void brakingIsNotLimited() {
        p(10, 0.05, BIG_ACCEL, 1.0);
        assertEquals(0.5, p(10, 0.05, BIG_ACCEL, 1.1), EPS);
        // error collapses to 1.5: output wants 0.075 and gets there in one step even with accel 1.0
        assertEquals(0.075, p(1.5, 0.05, 1.0, 1.2), EPS);
    }

    @Test
    public void reversingIsLimitedEvenThoughMagnitudeDrops() {
        p(10, 0.05, BIG_ACCEL, 1.0);
        assertEquals(0.5, p(10, 0.05, BIG_ACCEL, 1.1), EPS);
        // wants -0.5; that is a reversal, so it may only move by accel*dt = 0.1 toward it
        assertEquals(0.4, p(-10, 0.05, 1.0, 1.2), EPS);
    }

    // --- settle ---

    @Test
    public void insideToleranceWithSmallPreviousOutputReturnsZero() {
        p(0.5, 0.05, BIG_ACCEL, 1.0);            // first call: init
        assertEquals(0.0, p(0.5, 0.05, BIG_ACCEL, 1.1), EPS);   // settled: |0.5| <= 1 and prev output 0
    }

    @Test
    public void insideToleranceButStillMovingKeepsDriving() {
        p(10, 0.05, BIG_ACCEL, 1.0);
        p(10, 0.05, BIG_ACCEL, 1.1);             // output 0.5, not small
        // now inside tolerance, but previous output 0.5 >= 0.05, so it computes P normally
        assertEquals(0.5 * 0.05, p(0.5, 0.05, BIG_ACCEL, 1.2), EPS);
    }

    // --- integral ---

    @Test
    public void integralAccumulatesErrorTimesSeconds() {
        pid.calculateAxisPID(10, 0, 0.01, 0, BIG_ACCEL, 1.0, TOL);
        // dt 0.1, error 10: sum 1.0, i = 0.01
        assertEquals(0.01, pid.calculateAxisPID(10, 0, 0.01, 0, BIG_ACCEL, 1.1, TOL), EPS);
        assertEquals(0.02, pid.calculateAxisPID(10, 0, 0.01, 0, BIG_ACCEL, 1.2, TOL), EPS);
    }

    @Test
    public void integralContributionIsCappedAtIMaxOutput() {
        double t = 1.0;
        pid.calculateAxisPID(10, 0, 0.01, 0, BIG_ACCEL, t, TOL);
        double last = 0;
        for (int n = 0; n < 60; n++) {
            t += 0.1;
            last = pid.calculateAxisPID(10, 0, 0.01, 0, BIG_ACCEL, t, TOL);
        }
        assertEquals(PinpointPIDLoop.I_MAX_OUTPUT, last, EPS);
    }

    @Test
    public void zeroIGainKeepsTheIntegralAtZero() {
        pid.calculateAxisPID(10, 0, 0, 0, BIG_ACCEL, 1.0, TOL);
        for (int n = 1; n <= 20; n++) {
            assertEquals(0.0, pid.calculateAxisPID(10, 0, 0, 0, BIG_ACCEL, 1.0 + 0.1 * n, TOL), EPS);
        }
    }

    // --- derivative ---

    @Test
    public void derivativeOpposesAShrinkingError() {
        pid.calculateAxisPID(10, 0, 0, 0.1, BIG_ACCEL, 1.0, TOL);
        // error 10 -> 9 over 0.1 s: raw rate -10, filtered 0.15 * -10 = -1.5, times dGain 0.1
        assertEquals(-0.15, pid.calculateAxisPID(9, 0, 0, 0.1, BIG_ACCEL, 1.1, TOL), EPS);
    }

    @Test
    public void derivativeIsFilteredSoASingleJumpDecays() {
        pid.calculateAxisPID(10, 0, 0, 0.1, BIG_ACCEL, 1.0, TOL);
        double first  = pid.calculateAxisPID(9, 0, 0, 0.1, BIG_ACCEL, 1.1, TOL);   // rate -10
        double second = pid.calculateAxisPID(9, 0, 0, 0.1, BIG_ACCEL, 1.2, TOL);   // rate 0: filtered decays by 0.85
        assertEquals(first * PinpointPIDLoop.D_FILTER_KEEP, second, EPS);
        assertTrue(Math.abs(second) < Math.abs(first));
    }

    // --- timing guard ---

    @Test
    public void zeroElapsedTimeDoesNotBlowUpTheDerivative() {
        pid.calculateAxisPID(10, 0, 0, 0.1, BIG_ACCEL, 1.0, TOL);
        double out = pid.calculateAxisPID(9, 0, 0, 0.1, BIG_ACCEL, 1.0, TOL);   // same timestamp: dt floors at 1 ms
        assertTrue(Double.isFinite(out));
        assertEquals(-1.0, out, EPS);   // -1/0.001 * 0.15 * 0.1 = -15, clamped
    }
}
