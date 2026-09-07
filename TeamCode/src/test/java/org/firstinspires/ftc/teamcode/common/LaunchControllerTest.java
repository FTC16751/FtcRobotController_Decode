package org.firstinspires.ftc.teamcode.common;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests for LaunchController. These run on the laptop JVM with no robot attached:
 *   ./gradlew :TeamCode:testDebugUnitTest
 * or right-click this class in Android Studio and choose Run.
 *
 * The controller only talks to a Flywheel, a Feeder, and a Clock, so the test supplies fakes for
 * all three. The fake clock is advanced by hand, which lets us test the spin-up timeout and the
 * feed window without sleeping. The fake flywheel's measured velocity is set by the test, so we
 * can make it "reach speed", "never reach speed", or "collapse mid-feed" at will.
 *
 * This is the pattern for testing anything in Common: put an interface in front of the hardware,
 * hand the class a fake in the test, and check what it commanded.
 */
public class LaunchControllerTest {

    /** A flywheel whose measured velocity the test controls. Records the last commanded velocity. */
    static class FakeFlywheel implements Flywheel {
        double commanded = Double.NaN;
        double measured = 0.0;
        @Override public void setVelocity(double v) { commanded = v; }
        @Override public double getVelocity()       { return measured; }
    }

    /** A feeder that records whether it is running and how many times it was started and stopped. */
    static class FakeFeeder implements Feeder {
        boolean running = false;
        int starts = 0, stops = 0;
        @Override public void start() { if (!running) starts++; running = true; }
        @Override public void stop()  { if (running) stops++; running = false; }
    }

    /** A clock the test advances by hand. */
    static class FakeClock implements LaunchController.Clock {
        double now = 100.0;
        @Override public double seconds() { return now; }
        void advance(double sec) { now += sec; }
    }

    private FakeFlywheel wheel;
    private FakeFeeder feeder;
    private FakeClock clock;
    private LaunchController.Settings settings;
    private LaunchController ctl;

    private static final double TARGET = 1500.0;

    @Before
    public void setUp() {
        wheel = new FakeFlywheel();
        feeder = new FakeFeeder();
        clock = new FakeClock();
        settings = new LaunchController.Settings()
                .feedTimeSec(1.5)
                .cooldownSec(0.05)
                .spinUpTimeoutSec(2.0)
                .readyFraction(0.97)
                .stallFraction(0.80)
                .keepSpinning(true);
        ctl = new LaunchController(wheel, feeder, settings, null, clock);
    }

    // ------------------------------------------------------------------ idle

    @Test
    public void idleDoesNothingWithoutACommand() {
        assertFalse(ctl.update(false, TARGET));
        assertEquals(LaunchController.State.IDLE, ctl.getState());
        assertTrue(Double.isNaN(wheel.commanded));
        assertFalse(feeder.running);
        assertEquals(0, ctl.getShotsAttempted());
    }

    // --------------------------------------------------------------- spin-up

    @Test
    public void commandStartsSpinUpAndCountsAnAttempt() {
        assertFalse(ctl.update(true, TARGET));
        assertEquals(LaunchController.State.SPIN_UP, ctl.getState());
        assertEquals(TARGET, wheel.commanded, 0.0);
        assertEquals(1, ctl.getShotsAttempted());
        assertTrue(ctl.isBusy());
    }

    @Test
    public void doesNotFeedUntilFlywheelReachesReadyFraction() {
        ctl.update(true, TARGET);
        wheel.measured = TARGET * 0.90;          // below 97%
        ctl.update(false, TARGET);
        assertEquals(LaunchController.State.SPIN_UP, ctl.getState());
        assertFalse(feeder.running);

        wheel.measured = TARGET * 0.97;          // exactly at the threshold
        ctl.update(false, TARGET);
        assertEquals(LaunchController.State.FEEDING, ctl.getState());
    }

    @Test
    public void absoluteMinimumVelocityOverridesReadyFraction() {
        // Skyline-style call: shoot at 1500 but feed as soon as we pass 1200
        ctl.update(true, TARGET, 1200.0);
        wheel.measured = 1250.0;                 // above the absolute minimum, below 97% of target
        ctl.update(false, TARGET, 1200.0);
        assertEquals(LaunchController.State.FEEDING, ctl.getState());
    }

    // --------------------------------------------------------------- timeout

    @Test
    public void abortsIfFlywheelNeverReachesSpeed() {
        ctl.update(true, TARGET);
        wheel.measured = 100.0;                  // stuck
        clock.advance(2.5);                      // past the 2.0 s timeout
        assertFalse(ctl.update(false, TARGET));

        assertEquals(LaunchController.State.IDLE, ctl.getState());
        assertEquals(0.0, wheel.commanded, 0.0); // flywheel stopped
        assertFalse(feeder.running);
        assertEquals(1, ctl.getShotsAborted());
        assertEquals(0, ctl.getShotsFired());
        assertFalse(ctl.getLastAbortReason().isEmpty());
    }

    @Test
    public void timeoutCanBeDisabled() {
        settings.spinUpTimeoutSec(0);
        ctl.update(true, TARGET);
        wheel.measured = 100.0;
        clock.advance(60.0);
        ctl.update(false, TARGET);
        assertEquals(LaunchController.State.SPIN_UP, ctl.getState()); // still waiting, no abort
        assertEquals(0, ctl.getShotsAborted());
    }

    // ---------------------------------------------------------------- feeding

    @Test
    public void feedsForTheFeedTimeThenCoolsDownAndReportsDoneOnce() {
        ctl.update(true, TARGET);
        wheel.measured = TARGET;
        ctl.update(false, TARGET);               // sees the flywheel ready -> state becomes FEEDING
        assertEquals(LaunchController.State.FEEDING, ctl.getState());
        assertFalse(feeder.running);             // feeder starts on the NEXT loop (same as the original P3 code)
        ctl.update(false, TARGET);               // first FEEDING loop: feeder runs, feed timer is at ~0
        assertTrue(feeder.running);
        assertEquals(1, feeder.starts);

        clock.advance(1.0);                      // inside the 1.5 s feed window
        assertFalse(ctl.update(false, TARGET));
        assertTrue(feeder.running);

        clock.advance(0.6);                      // past 1.5 s
        assertFalse(ctl.update(false, TARGET));  // feeding ends, cooldown begins
        assertFalse(feeder.running);
        assertEquals(1, feeder.stops);
        assertEquals(1, ctl.getShotsFired());
        assertEquals(LaunchController.State.COOLDOWN, ctl.getState());
        assertEquals(TARGET, wheel.commanded, 0.0); // keepSpinning: still at speed

        clock.advance(0.1);                      // past the 0.05 s cooldown
        assertTrue(ctl.update(false, TARGET));   // exactly one loop reports true
        assertEquals(LaunchController.State.IDLE, ctl.getState());
        assertFalse(ctl.update(false, TARGET));  // and not again
    }

    @Test
    public void zeroCooldownReportsDoneOnTheSameLoopFeedingEnds() {
        settings.cooldownSec(0);
        ctl.update(true, TARGET);
        wheel.measured = TARGET;
        ctl.update(false, TARGET);
        clock.advance(1.6);
        assertTrue(ctl.update(false, TARGET));
        assertEquals(LaunchController.State.IDLE, ctl.getState());
        assertEquals(1, ctl.getShotsFired());
    }

    @Test
    public void stopsFlywheelAfterShotWhenKeepSpinningIsOff() {
        settings.keepSpinning(false);
        ctl.update(true, TARGET);
        wheel.measured = TARGET;
        ctl.update(false, TARGET);               // -> FEEDING
        clock.advance(1.6);
        ctl.update(false, TARGET);               // feed window over -> COOLDOWN (flywheel still commanded to target this loop)
        assertEquals(LaunchController.State.COOLDOWN, ctl.getState());
        ctl.update(false, TARGET);               // first COOLDOWN loop applies the keepSpinning choice
        assertEquals(0.0, wheel.commanded, 0.0);
    }

    // ------------------------------------------------------------------ stall

    @Test
    public void abortsIfVelocityCollapsesWhileFeeding() {
        ctl.update(true, TARGET);
        wheel.measured = TARGET;
        ctl.update(false, TARGET);               // -> FEEDING
        ctl.update(false, TARGET);               // feeder starts on the first FEEDING loop
        assertTrue(feeder.running);

        wheel.measured = TARGET * 0.5;           // below the 80% stall line
        assertFalse(ctl.update(false, TARGET));
        assertEquals(LaunchController.State.IDLE, ctl.getState());
        assertFalse(feeder.running);
        assertEquals(1, ctl.getShotsAborted());
        assertEquals(0, ctl.getShotsFired());
    }

    @Test
    public void stallCheckCanBeDisabled() {
        settings.stallFraction(0);
        ctl.update(true, TARGET);
        wheel.measured = TARGET;
        ctl.update(false, TARGET);
        wheel.measured = 10.0;                   // would be a stall if the check were on
        ctl.update(false, TARGET);
        assertEquals(LaunchController.State.FEEDING, ctl.getState());
        assertEquals(0, ctl.getShotsAborted());
    }

    // ------------------------------------------------------------- holding fire

    @Test
    public void holdingTheCommandFiresRepeatedly() {
        wheel.measured = TARGET;
        int fired = 0;
        for (int i = 0; i < 40; i++) {           // 40 loops, 0.1 s apart, command held true
            if (ctl.update(true, TARGET)) fired++;
            clock.advance(0.1);
        }
        // each shot = ~1.5 s feed + 0.05 s cooldown, so about two complete in 4 s
        assertEquals(2, fired);
        assertEquals(2, ctl.getShotsFired());
    }

    @Test
    public void stopResetsEverything() {
        ctl.update(true, TARGET);
        wheel.measured = TARGET;
        ctl.update(false, TARGET);               // -> FEEDING
        ctl.stop();
        assertEquals(LaunchController.State.IDLE, ctl.getState());
        assertEquals(0.0, wheel.commanded, 0.0);
        assertFalse(feeder.running);
        assertFalse(ctl.isBusy());
    }

    @Test
    public void resetShotCountersClearsAll() {
        ctl.update(true, TARGET);
        clock.advance(3.0);
        ctl.update(false, TARGET);               // aborted by timeout
        assertEquals(1, ctl.getShotsAttempted());
        ctl.resetShotCounters();
        assertEquals(0, ctl.getShotsAttempted());
        assertEquals(0, ctl.getShotsAborted());
        assertEquals("", ctl.getLastAbortReason());
    }
}
