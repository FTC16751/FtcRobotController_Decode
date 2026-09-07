package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * ONE-SHOT LAUNCH SEQUENCE, shared by every robot.
 *
 * Spin the flywheel up, wait until it is fast enough, feed a game piece, then cool down.
 * Non-blocking: call {@link #update} every loop; it returns true on the loop the shot completes.
 *
 * Every robot shoots this way. What differs per robot is only:
 *   - how to command and read the flywheel      -> the team supplies a {@link Flywheel}
 *   - what "feed a piece" physically means      -> the team supplies a {@link Feeder}
 *   - timings and thresholds                     -> the team fills in {@link Settings}
 *
 * Safety features (all configurable in Settings, all inherited by every robot):
 *   - spin-up timeout: if the flywheel never reaches speed, abort instead of waiting forever
 *   - ready check:     feed only once velocity >= target * readyFraction (or an absolute minimum)
 *   - stall abort:     if velocity collapses while feeding, stop and count an aborted shot
 *   - shot counters:   attempted / fired / aborted, for telemetry and debugging
 *
 * This class was lifted from P3_Robot3's launch state machine on 2026-09-07 (the most complete of
 * the six copies that existed) and generalized. Skyline_Robot uses it too.
 *
 * Typical use inside a robot class:
 * <pre>
 *   launchController = new LaunchController(flywheelAdapter, feederAdapter,
 *           new LaunchController.Settings().feedTimeSec(1.5).cooldownSec(0.05), telemetry);
 *   ...
 *   public boolean launchSequence(boolean shoot, double targetVelocity) {
 *       return launchController.update(shoot, targetVelocity);
 *   }
 * </pre>
 */
public class LaunchController {

    /** Where the sequence is. IDLE means ready for a new shot. */
    public enum State { IDLE, SPIN_UP, FEEDING, COOLDOWN }

    /** Timings and thresholds. These are "how it operates" values and belong in the team's Constants. */
    public static class Settings {
        /** How long the feeder runs per shot, seconds. */
        public double feedTimeSec = 1.5;
        /** Pause after a shot before the sequence reports done, seconds. 0 = report done immediately. */
        public double cooldownSec = 0.05;
        /** Abort if the flywheel has not reached speed within this many seconds. <= 0 disables the timeout. */
        public double spinUpTimeoutSec = 2.0;
        /** Feed when velocity >= target * readyFraction (0.97 = 97%). Ignored when a call supplies an absolute minimum. */
        public double readyFraction = 0.97;
        /** Abort the feed if velocity drops below target * stallFraction (0.80 = 80%). <= 0 disables stall detection. */
        public double stallFraction = 0.80;
        /** Keep the flywheel at speed between shots (fast follow-up) or stop it after each shot (saves battery). */
        public boolean keepSpinning = true;

        public Settings feedTimeSec(double v)      { this.feedTimeSec = v; return this; }
        public Settings cooldownSec(double v)      { this.cooldownSec = v; return this; }
        public Settings spinUpTimeoutSec(double v) { this.spinUpTimeoutSec = v; return this; }
        public Settings readyFraction(double v)    { this.readyFraction = v; return this; }
        public Settings stallFraction(double v)    { this.stallFraction = v; return this; }
        public Settings keepSpinning(boolean v)    { this.keepSpinning = v; return this; }
    }

    private final Flywheel flywheel;
    private final Feeder feeder;
    private final Telemetry telemetry;   // may be null
    public final Settings settings;

    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private double targetVelocity = 0.0;
    private double minReadyVelocity = 0.0;   // absolute override for the ready check; 0 = use readyFraction

    private int shotsAttempted = 0;
    private int shotsFired = 0;
    private int shotsAborted = 0;
    private String lastAbortReason = "";

    public LaunchController(Flywheel flywheel, Feeder feeder, Settings settings, Telemetry telemetry) {
        this.flywheel = flywheel;
        this.feeder = feeder;
        this.settings = settings != null ? settings : new Settings();
        this.telemetry = telemetry;
    }

    /**
     * Run one loop of the sequence. Call every loop.
     * @param shootCommand  true to start a shot; only looked at while IDLE, so holding it true
     *                      fires repeatedly and pulsing it fires once.
     * @param targetVelocity flywheel velocity to shoot at
     * @return true on the single loop in which a shot completes, false otherwise
     */
    public boolean update(boolean shootCommand, double targetVelocity) {
        return update(shootCommand, targetVelocity, 0.0);
    }

    /**
     * Same as {@link #update(boolean, double)} but with an absolute minimum velocity for the ready
     * check instead of settings.readyFraction. Use when a team tunes "minimum to shoot" directly.
     */
    public boolean update(boolean shootCommand, double targetVelocity, double minReadyVelocity) {
        switch (state) {
            case IDLE:
                if (shootCommand) {
                    this.targetVelocity = targetVelocity;
                    this.minReadyVelocity = minReadyVelocity;
                    flywheel.setVelocity(targetVelocity);
                    timer.reset();
                    state = State.SPIN_UP;
                    shotsAttempted++;
                }
                break;

            case SPIN_UP:
                flywheel.setVelocity(this.targetVelocity);   // keep commanding it until it gets there
                if (settings.spinUpTimeoutSec > 0 && timer.seconds() > settings.spinUpTimeoutSec) {
                    abort("flywheel did not reach speed in " + settings.spinUpTimeoutSec + "s");
                    break;
                }
                if (isFlywheelReady()) {
                    state = State.FEEDING;
                    timer.reset();
                }
                break;

            case FEEDING:
                flywheel.setVelocity(this.targetVelocity);   // hold speed while feeding
                feeder.start();
                if (settings.stallFraction > 0 && flywheel.getVelocity() < this.targetVelocity * settings.stallFraction) {
                    abort("velocity collapsed while feeding");
                    break;
                }
                if (timer.seconds() >= settings.feedTimeSec) {
                    feeder.stop();
                    shotsFired++;
                    if (settings.cooldownSec <= 0) {
                        finishShot();
                        return true;
                    }
                    state = State.COOLDOWN;
                    timer.reset();
                }
                break;

            case COOLDOWN:
                flywheel.setVelocity(settings.keepSpinning ? this.targetVelocity : 0.0);
                if (timer.seconds() >= settings.cooldownSec) {
                    finishShot();
                    return true;
                }
                break;
        }
        return false;
    }

    /** True once the flywheel is fast enough to feed, by the absolute minimum if given, else by readyFraction. */
    public boolean isFlywheelReady() {
        double threshold = minReadyVelocity > 0 ? minReadyVelocity : targetVelocity * settings.readyFraction;
        return flywheel.getVelocity() >= threshold;
    }

    /** Same test against an arbitrary target, for callers that want to know before requesting a shot. */
    public boolean isFlywheelReady(double target) {
        return flywheel.getVelocity() >= target * settings.readyFraction;
    }

    /** Stop everything and go IDLE. Used by aborts and by the robot's stopAll(). */
    public void stop() {
        flywheel.setVelocity(0.0);
        feeder.stop();
        state = State.IDLE;
    }

    public boolean isBusy()          { return state != State.IDLE; }
    public State getState()          { return state; }
    public double getTargetVelocity(){ return targetVelocity; }

    public int getShotsAttempted()   { return shotsAttempted; }
    public int getShotsFired()       { return shotsFired; }
    public int getShotsAborted()     { return shotsAborted; }
    public String getLastAbortReason(){ return lastAbortReason; }
    public void resetShotCounters()  { shotsAttempted = 0; shotsFired = 0; shotsAborted = 0; lastAbortReason = ""; }

    public void addTelemetry(Telemetry t) {
        if (t == null) return;
        t.addData("Launch", "%s  target %.0f  actual %.0f", state, targetVelocity, flywheel.getVelocity());
        t.addData("Shots", "fired %d / attempted %d / aborted %d", shotsFired, shotsAttempted, shotsAborted);
        if (!lastAbortReason.isEmpty()) t.addData("Last abort", lastAbortReason);
    }

    private void finishShot() {
        if (!settings.keepSpinning) flywheel.setVelocity(0.0);
        state = State.IDLE;
    }

    private void abort(String reason) {
        lastAbortReason = reason;
        shotsAborted++;
        if (telemetry != null) telemetry.addData("Launch ABORT", reason);
        stop();
    }
}
