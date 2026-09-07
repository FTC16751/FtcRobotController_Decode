package org.firstinspires.ftc.teamcode.common;

/**
 * Picks a flywheel velocity from how far away the goal is, and remembers the last good answer.
 *
 * The "waterfall" every robot used before 2026-09-07:
 *   1. goal visible  -> look the distance up in the team's table, remember the result
 *   2. goal not seen -> reuse the last remembered velocity (or the initial fallback if never seen)
 *
 * The table is "how the robot operates" data and lives in the team's Constants as a
 * {distance inches, velocity} array, for example GGRobotConstants.Launcher.FLYWHEEL_TABLE.
 * Distances between table rows are linearly interpolated.
 */
public class FlywheelVelocityModel {

    public static final String SOURCE_VISION = "VISION";
    public static final String SOURCE_LAST_KNOWN = "LAST KNOWN";
    public static final String SOURCE_NONE = "NONE";

    private final InterpolatingLookupTable table = new InterpolatingLookupTable();
    private double lastKnownGoodVelocity;
    private String lastSource = SOURCE_NONE;

    /**
     * @param distanceToVelocity rows of {distance inches, flywheel velocity}, in any order
     * @param initialFallback    velocity to use before the goal has ever been seen; pick a sane
     *                           mid-range shot, never 0
     */
    public FlywheelVelocityModel(double[][] distanceToVelocity, double initialFallback) {
        for (double[] row : distanceToVelocity) {
            table.add(row[0], row[1]);
        }
        this.lastKnownGoodVelocity = initialFallback;
    }

    /** Call every loop. Returns the velocity to shoot at right now. */
    public double update(AimTarget target) {
        if (target != null && target.isTargetVisible()) {
            lastKnownGoodVelocity = velocityForDistance(target.getDistanceToTagInches());
            lastSource = SOURCE_VISION;
        } else {
            lastSource = SOURCE_LAST_KNOWN;
        }
        return lastKnownGoodVelocity;
    }

    /** Table lookup only; does not touch the remembered value. */
    public double velocityForDistance(double distanceInches) {
        return table.get(distanceInches);
    }

    public double getLastKnownGoodVelocity() {
        return lastKnownGoodVelocity;
    }

    /** "VISION", "LAST KNOWN", or "NONE" before the first update. For telemetry. */
    public String getLastSource() {
        return lastSource;
    }
}
