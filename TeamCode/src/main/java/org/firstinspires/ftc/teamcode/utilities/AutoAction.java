package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.utilities.LaunchIndexer.FeederSide;

import java.util.Objects;

// Using a Java Record is a modern, concise way to create an immutable data class.// It's perfect for representing a single, unchangeable action in a sequence.

public final class AutoAction {
    public final double driveInches;
    public final double strafeInches;
    public final double turnDegrees;
    public final FeederSide feederSideToUse;
    public final String description;

    public AutoAction(
            // Drive parameters
            double driveInches,
            double strafeInches,
            double turnDegrees,

            // Shot parameter
            FeederSide feederSideToUse, // This will be null if the action is just a drive

            // Metadata for telemetry and debugging
            String description
    ) {
        this.driveInches = driveInches;
        this.strafeInches = strafeInches;
        this.turnDegrees = turnDegrees;
        this.feederSideToUse = feederSideToUse;
        this.description = description;
    }

    /**
     * A convenience constructor for an action that is ONLY a shot (no movement).
     *
     * @param feeder      The side (LEFT/RIGHT) to shoot from.
     * @param description A human-readable description for telemetry.
     */
    public AutoAction(FeederSide feeder, String description) {
        // Calls the main constructor with zero for all drive parameters.
        this(0, 0, 0, feeder, description);
    }

    /**
     * A convenience constructor for an action that is ONLY a drive (no shooting).
     *
     * @param drive       The distance to drive forward/backward in inches.
     * @param strafe      The distance to strafe left/right in inches.
     * @param turn        The angle to turn in degrees.
     * @param description A human-readable description for telemetry.
     */
    public AutoAction(double drive, double strafe, double turn, String description) {
        // Calls the main constructor with null for the feeder side.
        this(drive, strafe, turn, null, description);
    }

    public double driveInches() {
        return driveInches;
    }

    public double strafeInches() {
        return strafeInches;
    }

    public double turnDegrees() {
        return turnDegrees;
    }

    public FeederSide feederSideToUse() {
        return feederSideToUse;
    }

    public String description() {
        return description;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) return true;
        if (obj == null || obj.getClass() != this.getClass()) return false;
        AutoAction that = (AutoAction) obj;
        return Double.doubleToLongBits(this.driveInches) == Double.doubleToLongBits(that.driveInches) &&
                Double.doubleToLongBits(this.strafeInches) == Double.doubleToLongBits(that.strafeInches) &&
                Double.doubleToLongBits(this.turnDegrees) == Double.doubleToLongBits(that.turnDegrees) &&
                Objects.equals(this.feederSideToUse, that.feederSideToUse) &&
                Objects.equals(this.description, that.description);
    }

    @Override
    public int hashCode() {
        return Objects.hash(driveInches, strafeInches, turnDegrees, feederSideToUse, description);
    }

    @Override
    public String toString() {
        return "AutoAction[" +
                "driveInches=" + driveInches + ", " +
                "strafeInches=" + strafeInches + ", " +
                "turnDegrees=" + turnDegrees + ", " +
                "feederSideToUse=" + feederSideToUse + ", " +
                "description=" + description + ']';
    }

}
