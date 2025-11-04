package org.firstinspires.ftc.teamcode.utilities.Common;


import java.util.Map;
import java.util.TreeMap;

/**
 * A lookup table that uses linear interpolation to find values between
 * known data points. This is perfect for tuning things like flywheel
 * velocity based on distance.
 */
public class InterpolatingLookupTable {

    // TreeMap is the core of this. It keeps keys (distance) sorted.
    private final TreeMap<Double, Double> table;

    public InterpolatingLookupTable() {
        this.table = new TreeMap<>();
    }

    /**
     * Adds a known data point to the lookup table.
     * @param key   The "input" value (e.g., distance in cm).
     * @param value The "output" value (e.g., motor velocity in ticks/sec).
     */
    public void add(double key, double value) {
        table.put(key, value);
    }

    /**
     * Gets the interpolated value for a given key.
     * @param key The current input (e.g., your robot's current distance).
     * @return The interpolated output (the calculated motor velocity).
     */
    public double get(double key) {
        // If the table is empty, return 0.0
        if (table.isEmpty()) {
            return 0.0;
        }

        // Check for an exact match
        if (table.containsKey(key)) {
            return table.get(key);
        }

        // Get the entries just below (floor) and just above (ceiling) our key
        Map.Entry<Double, Double> floorEntry = table.floorEntry(key);
        Map.Entry<Double, Double> ceilingEntry = table.ceilingEntry(key);

        // --- Handle Edge Cases ---

        // 1. Key is "off the left end" (smaller than all known points)
        //    -> Return the value of the smallest known point. (Clamping)
        if (floorEntry == null) {
            return ceilingEntry.getValue();
        }

        // 2. Key is "off the right end" (larger than all known points)
        //    -> Return the value of the largest known point. (Clamping)
        if (ceilingEntry == null) {
            return floorEntry.getValue();
        }

        // --- Perform Linear Interpolation ---

        // Get the (x, y) values for the lower and upper points
        double x1 = floorEntry.getKey();
        double y1 = floorEntry.getValue();
        double x2 = ceilingEntry.getKey();
        double y2 = ceilingEntry.getValue();

        // Calculate the percentage (or "t" value) of how far our key
        // is between x1 and x2.
        double t = (key - x1) / (x2 - x1);

        // Apply the formula: y = y1 + t * (y2 - y1)
        double interpolatedValue = y1 + t * (y2 - y1);

        return interpolatedValue;
    }
}