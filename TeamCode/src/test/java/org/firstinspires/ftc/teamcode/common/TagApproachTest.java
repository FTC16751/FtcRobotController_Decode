package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests for TagApproach. Laptop JVM, no robot:
 *   ./gradlew :TeamCode:testDebugUnitTest
 *
 * The controller only talks to a TagSighting and a Clock, so the test fakes both. The fake sighting
 * is set by the test ("tag is 30 in ahead, 4 in to the right, robot needs to turn 10 deg left"), the
 * fake clock is advanced by hand. Each test asserts the three powers in moveRobot's convention:
 * drive +forward, strafe +right, yaw +clockwise.
 */
public class TagApproachTest {

    /** A tag sighting the test controls. */
    static class FakeSighting implements TagSighting {
        int visibleId = -1;
        double forward = 0, right = 0, squareUp = 0;

        FakeSighting see(int id, double forwardIn, double rightIn, double squareUpDeg) {
            visibleId = id; forward = forwardIn; right = rightIn; squareUp = squareUpDeg;
            return this;
        }
        FakeSighting lose() { visibleId = -1; return this; }

        @Override public boolean canSee(int tagId)  { return tagId == visibleId; }
        @Override public double forwardInches()     { return forward; }
        @Override public double rightInches()       { return right; }
        @Override public double squareUpDegrees()   { return squareUp; }
    }

    static class FakeClock implements TagApproach.Clock {
        double now = 100.0;
        @Override public double seconds() { return now; }
        void advance(double sec) { now += sec; }
    }

    private static final int    TAG      = 24;
    private static final double STANDOFF = 12.0;
    private static final double HOLD     = 0.5;
    private static final double EPS      = 1e-9;

    private FakeSighting tag;
    private FakeClock clock;
    private TagApproach.Settings settings;
    private TagApproach approach;

    @Before
    public void setUp() {
        tag = new FakeSighting();
        clock = new FakeClock();
        settings = new TagApproach.Settings()
                .kpDrive(0.05).kpStrafe(0.05).kpYaw(0.02)
                .maxPower(0.4).minPower(0.0)
                .toleranceInches(1.0).toleranceDegrees(2.0)
                .lostTimeoutSec(1.5).maxTimeSec(5.0);
        approach = new TagApproach(settings, clock);
    }

    private void assertPowers(double drive, double strafe, double yaw) {
        assertEquals("drive",  drive,  approach.getDrivePower(),  EPS);
        assertEquals("strafe", strafe, approach.getStrafePower(), EPS);
        assertEquals("yaw",    yaw,    approach.getYawPower(),    EPS);
    }

    // --- life cycle ---

    @Test
    public void idleBeforeStartDoesNothing() {
        assertFalse(approach.isBusy());
        assertTrue(approach.update(tag.see(TAG, 30, 0, 0)));   // "finished" because there is nothing to do
        assertPowers(0, 0, 0);
        assertEquals(TagApproach.State.IDLE, approach.getState());
    }

    @Test
    public void startMakesItBusyAndApproaching() {
        approach.start(TAG, STANDOFF, HOLD);
        assertTrue(approach.isBusy());
        assertEquals(TagApproach.State.APPROACHING, approach.getState());
        assertEquals(TAG, approach.getTagId());
    }

    @Test
    public void stopReturnsToIdleWithZeroPower() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, 30, 0, 0));
        approach.stop();
        assertFalse(approach.isBusy());
        assertPowers(0, 0, 0);
        assertEquals(TagApproach.State.IDLE, approach.getState());
    }

    // --- proportional control and sign conventions ---

    @Test
    public void tooFarAwayDrivesForward() {
        approach.start(TAG, STANDOFF, HOLD);
        assertFalse(approach.update(tag.see(TAG, 18, 0, 0)));   // 6 in too far
        assertPowers(0.05 * 6, 0, 0);
        assertEquals(6.0, approach.getForwardErrorInches(), EPS);
    }

    @Test
    public void tooCloseDrivesBackward() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, 8, 0, 0));                 // 4 in too close
        assertPowers(-0.05 * 4, 0, 0);
    }

    @Test
    public void tagToTheRightStrafesRight() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, STANDOFF, 4, 0));
        assertPowers(0, 0.05 * 4, 0);
    }

    @Test
    public void tagToTheLeftStrafesLeft() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, STANDOFF, -4, 0));
        assertPowers(0, -0.05 * 4, 0);
    }

    @Test
    public void needingToTurnLeftGivesNegativeYawPower() {
        // squareUpDegrees +10 means "turn left"; moveRobot's yaw is +clockwise, so the power is negative
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, STANDOFF, 0, 10));
        assertPowers(0, 0, -0.02 * 10);
    }

    @Test
    public void needingToTurnRightGivesPositiveYawPower() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, STANDOFF, 0, -10));
        assertPowers(0, 0, 0.02 * 10);
    }

    @Test
    public void allThreeAxesRunAtOnce() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, 16, -2, 5));
        assertPowers(0.05 * 4, -0.05 * 2, -0.02 * 5);
    }

    @Test
    public void clampsEachAxisToMaxPower() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, 100, -100, 100));
        assertPowers(0.4, -0.4, -0.4);
    }

    @Test
    public void minPowerLiftsSmallOutputsButLeavesZeroAlone() {
        settings.minPower(0.1);
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, STANDOFF + 1.5, -1.2, 0));   // raw 0.075 fwd, -0.06 strafe, 0 yaw
        assertPowers(0.1, -0.1, 0);
    }

    // --- tolerance, hold, and done ---

    @Test
    public void insideToleranceHoldsWithZeroPowerThenFinishesDone() {
        approach.start(TAG, STANDOFF, HOLD);
        assertFalse(approach.update(tag.see(TAG, STANDOFF + 0.5, 0.5, 1.0)));
        assertEquals(TagApproach.State.HOLDING, approach.getState());
        assertPowers(0, 0, 0);
        assertTrue(approach.isBusy());

        clock.advance(HOLD - 0.1);
        assertFalse(approach.update(tag.see(TAG, STANDOFF, 0, 0)));   // not held long enough yet
        assertTrue(approach.isBusy());

        clock.advance(0.1);
        assertTrue(approach.update(tag.see(TAG, STANDOFF, 0, 0)));
        assertEquals(TagApproach.State.DONE, approach.getState());
        assertTrue(approach.succeeded());
        assertFalse(approach.isBusy());
        assertPowers(0, 0, 0);
    }

    @Test
    public void leavingToleranceDuringHoldRestartsTheHoldTimer() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, STANDOFF, 0, 0));            // in tolerance, hold starts
        clock.advance(0.4);
        approach.update(tag.see(TAG, STANDOFF + 3, 0, 0));        // bumped out of tolerance
        assertEquals(TagApproach.State.APPROACHING, approach.getState());
        clock.advance(0.2);
        assertFalse(approach.update(tag.see(TAG, STANDOFF, 0, 0)));   // back in; 0.6 s since first hold but timer restarted
        clock.advance(HOLD);
        assertTrue(approach.update(tag.see(TAG, STANDOFF, 0, 0)));
        assertTrue(approach.succeeded());
    }

    @Test
    public void zeroHoldTimeFinishesOnTheFirstInToleranceLoop() {
        approach.start(TAG, STANDOFF, 0.0);
        assertTrue(approach.update(tag.see(TAG, STANDOFF, 0, 0)));
        assertTrue(approach.succeeded());
    }

    @Test
    public void afterDoneUpdateKeepsReportingFinishedWithZeroPower() {
        approach.start(TAG, STANDOFF, 0.0);
        approach.update(tag.see(TAG, STANDOFF, 0, 0));
        assertTrue(approach.update(tag.see(TAG, 40, 0, 0)));      // tag moved, but we are done
        assertPowers(0, 0, 0);
        assertEquals(TagApproach.State.DONE, approach.getState());
    }

    // --- losing the tag ---

    @Test
    public void losingTheTagCoastsOnLastPowersThenGivesUp() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, 18, 2, 0));                  // drive 0.3, strafe 0.1
        assertPowers(0.3, 0.1, 0);

        clock.advance(1.0);
        assertFalse(approach.update(tag.lose()));                 // 1.0 s lost: still coasting
        assertPowers(0.3, 0.1, 0);
        assertTrue(approach.isBusy());

        clock.advance(0.6);
        assertTrue(approach.update(tag.lose()));                  // 1.6 s lost: give up
        assertEquals(TagApproach.State.LOST, approach.getState());
        assertFalse(approach.succeeded());
        assertFalse(approach.isBusy());
        assertPowers(0, 0, 0);
    }

    @Test
    public void neverSeeingTheTagWaitsWithZeroPowerThenGivesUp() {
        approach.start(TAG, STANDOFF, HOLD);
        assertFalse(approach.update(tag.lose()));
        assertPowers(0, 0, 0);                                    // do not guess a direction
        clock.advance(1.0);
        assertFalse(approach.update(tag.lose()));
        assertPowers(0, 0, 0);
        clock.advance(0.6);
        assertTrue(approach.update(tag.lose()));
        assertEquals(TagApproach.State.LOST, approach.getState());
    }

    @Test
    public void seeingTheTagAgainResetsTheLostTimer() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, 18, 0, 0));
        clock.advance(1.2);
        approach.update(tag.lose());
        clock.advance(0.2);
        approach.update(tag.see(TAG, 17, 0, 0));                  // fresh sighting at 1.4 s
        clock.advance(1.2);
        assertFalse(approach.update(tag.lose()));                 // only 1.2 s since the fresh sighting
        assertTrue(approach.isBusy());
    }

    @Test
    public void losingTheTagWhileHoldingCoastsAtZeroAndNeverDeclaresDoneBlind() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, STANDOFF, 0, 0));            // holding
        clock.advance(1.0);
        assertFalse(approach.update(tag.lose()));                 // hold time has passed, but no sighting
        assertPowers(0, 0, 0);
        assertTrue(approach.isBusy());
        clock.advance(0.6);
        assertTrue(approach.update(tag.lose()));
        assertEquals(TagApproach.State.LOST, approach.getState());
    }

    @Test
    public void wrongTagIdCountsAsNotVisible() {
        approach.start(TAG, STANDOFF, HOLD);
        assertFalse(approach.update(tag.see(TAG + 1, STANDOFF, 0, 0)));
        assertPowers(0, 0, 0);
        assertEquals(0.0, approach.getForwardErrorInches(), EPS);
    }

    // --- hard time limit ---

    @Test
    public void maxTimeEndsTheApproachEvenWhileTheTagIsVisible() {
        approach.start(TAG, STANDOFF, HOLD);
        approach.update(tag.see(TAG, 40, 0, 0));
        clock.advance(5.1);
        assertTrue(approach.update(tag.see(TAG, 40, 0, 0)));
        assertEquals(TagApproach.State.TIMED_OUT, approach.getState());
        assertFalse(approach.succeeded());
        assertPowers(0, 0, 0);
    }

    // --- restart ---

    @Test
    public void startAfterAFinishedApproachBeginsFresh() {
        approach.start(TAG, STANDOFF, HOLD);
        clock.advance(5.1);
        approach.update(tag.lose());                              // TIMED_OUT
        assertFalse(approach.isBusy());

        approach.start(TAG + 1, 6.0, 0.0);
        assertTrue(approach.isBusy());
        assertEquals(TagApproach.State.APPROACHING, approach.getState());
        assertEquals(TAG + 1, approach.getTagId());
        approach.update(tag.see(TAG + 1, 10, 0, 0));              // 4 in too far from the 6 in standoff
        assertPowers(0.05 * 4, 0, 0);
    }
}
