package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Non-blocking "drive to X inches in front of an AprilTag, squared up" controller.
 *
 * Pure math and state: it never touches hardware. Each loop the caller hands it a TagSighting
 * (where the tag is relative to the robot) and reads back three motor powers in
 * DriveUtil2026b.moveRobot's convention (drive +forward, strafe +right, yaw +clockwise). Because
 * it does not block, the launcher, intake, and anything else keep running in the same loop.
 *
 * Usage from an iterative auto, through DriveUtil2026b:
 * <pre>
 *   robot.drive.driveToTagAsync(vision, 24, 12.0, 0.5);   // once: tag 24, stop 12 in away, hold 0.5 s
 *   ...
 *   if (!robot.drive.isBusy()) { next step }               // each loop; robot.update() steps it
 * </pre>
 *
 * Life cycle: IDLE, then start() puts it in APPROACHING. While the tag is visible it runs one
 * proportional controller per axis. When all three errors are inside tolerance it goes to HOLDING
 * with zero power and finishes DONE after the hold time. If the tag drops out of view it coasts on
 * the last good powers for lostTimeoutSec, then gives up as LOST. A hard maxTimeSec ends it as
 * TIMED_OUT no matter what. isBusy() is true only in APPROACHING and HOLDING; succeeded() is true
 * only for DONE. A finished approach only ever declares DONE on a fresh sighting, never while
 * coasting blind.
 *
 * Tuning lives in Settings (fluent setters, like LaunchController.Settings). The per-call values
 * (tag id, standoff distance, hold time) are start() arguments because they change per step.
 */
public class TagApproach {

    public enum State { IDLE, APPROACHING, HOLDING, DONE, LOST, TIMED_OUT }

    /** Time source, so tests can advance time by hand. Same shape as LaunchController.Clock. */
    public interface Clock {
        double seconds();
    }

    public static final Clock SYSTEM_CLOCK = new Clock() {
        @Override public double seconds() { return System.nanoTime() / 1e9; }
    };

    public static class Settings {
        /** Motor power per inch of forward error. */
        public double kpDrive          = 0.05;
        /** Motor power per inch of lateral error. */
        public double kpStrafe         = 0.05;
        /** Motor power per degree of square-up error. */
        public double kpYaw            = 0.02;
        /** Clamp on each axis's power. Keep it low; the robot is about to touch something. */
        public double maxPower         = 0.4;
        /** Smallest nonzero power per axis, to get past friction. 0 turns this off. */
        public double minPower         = 0.0;
        /** Forward and lateral error inside this counts as in position, inches. */
        public double toleranceInches  = 1.0;
        /** Square-up error inside this counts as in position, degrees. */
        public double toleranceDegrees = 2.0;
        /** How long to coast on the last powers after the tag disappears before giving up, seconds. */
        public double lostTimeoutSec   = 1.5;
        /** Hard limit on one approach, seconds. */
        public double maxTimeSec       = 5.0;

        public Settings kpDrive(double v)          { this.kpDrive = v; return this; }
        public Settings kpStrafe(double v)         { this.kpStrafe = v; return this; }
        public Settings kpYaw(double v)            { this.kpYaw = v; return this; }
        public Settings maxPower(double v)         { this.maxPower = v; return this; }
        public Settings minPower(double v)         { this.minPower = v; return this; }
        public Settings toleranceInches(double v)  { this.toleranceInches = v; return this; }
        public Settings toleranceDegrees(double v) { this.toleranceDegrees = v; return this; }
        public Settings lostTimeoutSec(double v)   { this.lostTimeoutSec = v; return this; }
        public Settings maxTimeSec(double v)       { this.maxTimeSec = v; return this; }
    }

    private final Settings settings;
    private final Clock clock;

    private State  state = State.IDLE;
    private int    tagId;
    private double standoffInches;
    private double holdTimeSec;

    private double startTime;
    private double lastSeenTime;
    private double holdStartTime;
    private boolean everSeen;

    // Outputs, in moveRobot's convention
    private double drivePower, strafePower, yawPower;
    // Powers from the last loop that saw the tag; used to coast when it drops out
    private double lastGoodDrive, lastGoodStrafe, lastGoodYaw;

    // Latest errors, for telemetry
    private double forwardErrorInches, rightErrorInches, yawErrorDegrees;

    public TagApproach(Settings settings) {
        this(settings, SYSTEM_CLOCK);
    }

    public TagApproach(Settings settings, Clock clock) {
        this.settings = settings;
        this.clock = clock;
    }

    /**
     * Begin an approach. Safe to call while one is running; it restarts.
     * @param tagId          AprilTag id to approach.
     * @param standoffInches How far in front of the tag's face to stop.
     * @param holdTimeSec    How long all three errors must stay inside tolerance before DONE.
     */
    public void start(int tagId, double standoffInches, double holdTimeSec) {
        this.tagId = tagId;
        this.standoffInches = standoffInches;
        this.holdTimeSec = holdTimeSec;
        double now = clock.seconds();
        startTime = now;
        lastSeenTime = now;      // so the acquire timeout equals lostTimeoutSec
        holdStartTime = now;
        everSeen = false;
        lastGoodDrive = lastGoodStrafe = lastGoodYaw = 0.0;
        forwardErrorInches = rightErrorInches = yawErrorDegrees = 0.0;
        setPowers(0, 0, 0);
        state = State.APPROACHING;
    }

    /** Abandon the approach and zero the powers. */
    public void stop() {
        state = State.IDLE;
        setPowers(0, 0, 0);
    }

    /**
     * Step the controller once. Call every loop while isBusy(), then read the three powers.
     * @return true when the approach is finished, for any reason. Check succeeded() or getState()
     *         to tell DONE from LOST and TIMED_OUT.
     */
    public boolean update(TagSighting sighting) {
        if (!isBusy()) {
            setPowers(0, 0, 0);
            return true;
        }

        double now = clock.seconds();
        if (now - startTime > settings.maxTimeSec) {
            return finish(State.TIMED_OUT);
        }

        if (sighting.canSee(tagId)) {
            lastSeenTime = now;
            everSeen = true;

            forwardErrorInches = sighting.forwardInches() - standoffInches;  // + means too far away
            rightErrorInches   = sighting.rightInches();                     // + means tag is to our right
            yawErrorDegrees    = sighting.squareUpDegrees();                 // + means turn left

            boolean inPosition = Math.abs(forwardErrorInches) <= settings.toleranceInches
                              && Math.abs(rightErrorInches)   <= settings.toleranceInches
                              && Math.abs(yawErrorDegrees)    <= settings.toleranceDegrees;

            if (inPosition) {
                if (state != State.HOLDING) {
                    state = State.HOLDING;
                    holdStartTime = now;
                }
                lastGoodDrive = lastGoodStrafe = lastGoodYaw = 0.0;
                setPowers(0, 0, 0);
                if (now - holdStartTime >= holdTimeSec) {
                    return finish(State.DONE);
                }
            } else {
                state = State.APPROACHING;   // leaving tolerance restarts the hold next time
                lastGoodDrive  = axisPower(settings.kpDrive  * forwardErrorInches);
                lastGoodStrafe = axisPower(settings.kpStrafe * rightErrorInches);
                lastGoodYaw    = axisPower(-settings.kpYaw   * yawErrorDegrees);   // turn left = negative yaw power
                setPowers(lastGoodDrive, lastGoodStrafe, lastGoodYaw);
            }
            return false;
        }

        // Tag not in view this loop
        if (now - lastSeenTime > settings.lostTimeoutSec) {
            return finish(State.LOST);
        }
        if (everSeen) {
            setPowers(lastGoodDrive, lastGoodStrafe, lastGoodYaw);   // coast toward where it was
        } else {
            setPowers(0, 0, 0);                                      // never seen it: do not guess
        }
        return false;
    }

    /** Clamp to maxPower and, if minPower is set, lift small nonzero outputs up to it. */
    private double axisPower(double raw) {
        double p = Math.max(-settings.maxPower, Math.min(settings.maxPower, raw));
        if (settings.minPower > 0 && p != 0 && Math.abs(p) < settings.minPower) {
            p = Math.copySign(settings.minPower, p);
        }
        return p;
    }

    private boolean finish(State endState) {
        state = endState;
        setPowers(0, 0, 0);
        return true;
    }

    private void setPowers(double drive, double strafe, double yaw) {
        drivePower = drive;
        strafePower = strafe;
        yawPower = yaw;
    }

    // --- Outputs and status ---

    public double getDrivePower()  { return drivePower; }
    public double getStrafePower() { return strafePower; }
    public double getYawPower()    { return yawPower; }

    public boolean isBusy()    { return state == State.APPROACHING || state == State.HOLDING; }
    public boolean succeeded() { return state == State.DONE; }
    public State getState()    { return state; }
    public int getTagId()      { return tagId; }

    public double getForwardErrorInches() { return forwardErrorInches; }
    public double getRightErrorInches()   { return rightErrorInches; }
    public double getYawErrorDegrees()    { return yawErrorDegrees; }

    public void addTelemetry(Telemetry t) {
        t.addData("TagApproach", "%s tag %d, standoff %.1f in", state, tagId, standoffInches);
        t.addData("  error fwd/right/yaw", "%.1f in  %.1f in  %.1f deg", forwardErrorInches, rightErrorInches, yawErrorDegrees);
        t.addData("  power drive/strafe/yaw", "%.2f  %.2f  %.2f", drivePower, strafePower, yawPower);
    }
}
