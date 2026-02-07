package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;



import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Manages an array of color/distance sensors in the intake to provide a complete
 * picture of all artifacts currently held by the robot. This class handles sensor fusion,
 * color averaging, and state management for each sensor slot.
 *
 * UPDATED VERSION 1.1: Now supports THREE slots:
 * - LEFT slot: Dual sensors (sensor1 + sensor2) with fusion
 * - CENTER slot: Single sensor
 * - RIGHT slot: Dual sensors (sensor1 + sensor2) with fusion
 */

public class IntakeSensorFusion001 {

    // Public enums for easy access from other classes like your OpMode.
    public enum ArtifactColor { PURPLE, GREEN, UNKNOWN }
    public enum IntakeSlot { LEFT, CENTER, RIGHT }  // UPDATED: Added CENTER

    /**
     * A private inner class that manages the state and logic for a single physical color sensor.
     * This contains the core logic from your ColorAveragingSensor.java.
     */
    private static class ArtifactSensor {
        private enum DetectionState { WAITING, SAMPLING, DETERMINED }
        private DetectionState currentState = DetectionState.WAITING;

        // Hardware and Configuration
        private final NormalizedColorSensor colorSensor;
        private final DistanceSensor distanceSensor;
        private final String name;
        private final Telemetry telemetry;

        // State and Averaging Variables
        private final ElapsedTime samplingTimer = new ElapsedTime();
        private double totalRed = 0, totalGreen = 0, totalBlue = 0;
        private int sampleCount = 0;

        // Output variables
        private ArtifactColor determinedColor = ArtifactColor.UNKNOWN;
        private boolean isOccupied = false;

        // Constants (can be moved to a separate GGRobotConstants file if desired)
        private static final double DISTANCE_THRESHOLD_CM = 7.0; // How close an artifact must be to be "detected".
        private static final double SAMPLING_DURATION_MSEC = 300; // How long to sample colors.

        /**
         * Constructor for a single sensor.
         * @param sensorName The name of the sensor in the robot configuration (e.g., "color_sensor_left1").
         */
        public ArtifactSensor(String sensorName, HardwareMap hardwareMap, Telemetry telemetry) {
            this.name = sensorName;
            this.telemetry = telemetry;
            try {
                colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorName);
                distanceSensor = (DistanceSensor) colorSensor;
                // Set a higher gain for better color sensitivity, especially for purple.
                colorSensor.setGain(20);
            } catch (Exception e) {
                // Throw a runtime exception to fail fast if a sensor is misconfigured.
                throw new RuntimeException("Could not find or initialize sensor: " + sensorName);
            }
        }

        /**
         * This method is called on every loop cycle. It runs the state machine for this sensor.
         */
        public void update() {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            isOccupied = (distance < DISTANCE_THRESHOLD_CM);

            switch (currentState) {
                case WAITING:
                    if (isOccupied) {
                        // An artifact has entered the slot. Reset variables and start sampling.
                        totalRed = 0; totalGreen = 0; totalBlue = 0;
                        sampleCount = 0;
                        samplingTimer.reset();
                        currentState = DetectionState.SAMPLING;
                    } else {
                        // If the slot is not occupied, ensure its color is marked as UNKNOWN.
                        determinedColor = ArtifactColor.UNKNOWN;
                    }
                    break;

                case SAMPLING:
                    if (!isOccupied) {
                        // Artifact was removed in the middle of sampling. Reset immediately.
                        // FIX: Clear the color here too to prevent freeze!
                        determinedColor = ArtifactColor.UNKNOWN;
                        currentState = DetectionState.WAITING;
                        break;
                    }

                    if (samplingTimer.milliseconds() < SAMPLING_DURATION_MSEC) {
                        // While the timer is running, accumulate color data.
                        NormalizedRGBA colors = colorSensor.getNormalizedColors();
                        totalRed += colors.red;
                        totalGreen += colors.green;
                        totalBlue += colors.blue;
                        sampleCount++;
                    } else {
                        // Sampling time is over. Analyze the collected data.
                        determineColor();
                        currentState = DetectionState.DETERMINED;
                    }
                    break;

                case DETERMINED:
                    if (!isOccupied) {
                        // The artifact has been removed. Reset the state machine for the next one.
                        // FIX: Clear the color here to prevent freeze!
                        determinedColor = ArtifactColor.UNKNOWN;
                        currentState = DetectionState.WAITING;
                    }
                    // Otherwise, do nothing. The color is determined and will remain locked
                    // until the artifact is removed.
                    break;
            }
        }

        /**
         * Calculates the average color and makes a final decision. This is the core color logic.
         */
        private void determineColor() {
            if (sampleCount == 0) {
                this.determinedColor = ArtifactColor.UNKNOWN;
                return;
            }
            double avgRed = totalRed / sampleCount;
            double avgGreen = totalGreen / sampleCount;
            double avgBlue = totalBlue / sampleCount;

            // This logic needs to be tuned with real sensor values from your artifacts!
            // A common way to detect purple is to see if Red and Blue are both stronger than Green.
            // A common way to detect green is to see if Green is much stronger than Red and Blue.
            if (avgGreen > avgRed && avgGreen > avgBlue) {
                this.determinedColor = ArtifactColor.GREEN;
            } else if ((avgBlue > avgRed) && (avgBlue > avgGreen)) {
                this.determinedColor = ArtifactColor.PURPLE;
            } else {
                this.determinedColor = ArtifactColor.UNKNOWN;
            }
        }

        // Public "getter" methods for other classes to use.
        public ArtifactColor getColor() { return determinedColor; }
        public boolean isOccupied() { return isOccupied; }
        public String getName() { return name; }

        // Telemetry for debugging this specific sensor.
        public void addTelemetry() {
            telemetry.addData(name, "Occ: %s, Color: %s, State: %s",
                    isOccupied(), getColor(), currentState);
        }
    }

    /**
     * A private inner class that manages a PAIR of sensors for a single intake slot.
     * This provides redundancy - if one sensor sees the hole in the artifact,
     * the other sensor can still get a good reading.
     */
    private static class ArtifactSensorPair {
        private final ArtifactSensor sensor1;
        private final ArtifactSensor sensor2;
        private final String slotName;
        private final Telemetry telemetry;

        /**
         * Constructor for a sensor pair.
         * @param slotName The name of this slot (e.g., "LEFT" or "RIGHT").
         * @param sensor1Name The name of the first sensor in the robot configuration.
         * @param sensor2Name The name of the second sensor in the robot configuration.
         */
        public ArtifactSensorPair(String slotName, String sensor1Name, String sensor2Name,
                                  HardwareMap hardwareMap, Telemetry telemetry) {
            this.slotName = slotName;
            this.telemetry = telemetry;
            this.sensor1 = new ArtifactSensor(sensor1Name, hardwareMap, telemetry);
            this.sensor2 = new ArtifactSensor(sensor2Name, hardwareMap, telemetry);
        }

        /**
         * Updates both sensors in the pair.
         */
        public void update() {
            sensor1.update();
            sensor2.update();
        }

        /**
         * Determines if the slot is occupied based on EITHER sensor detecting an artifact.
         * If either sensor sees something close, we consider the slot occupied.
         * @return true if at least one sensor detects an artifact.
         */
        public boolean isOccupied() {
            return sensor1.isOccupied() || sensor2.isOccupied();
        }

        /**
         * Fuses the color readings from both sensors using intelligent logic:
         * 1. If both sensors agree, use that color.
         * 2. If they disagree:
         *    - Prefer any non-UNKNOWN reading (one sensor might see the hole).
         *    - If both have non-UNKNOWN but different colors, prefer sensor1 (or you can add more logic).
         * 3. If both are UNKNOWN, return UNKNOWN.
         * @return The fused ArtifactColor for this slot.
         */
        public ArtifactColor getFusedColor() {
            ArtifactColor color1 = sensor1.getColor();
            ArtifactColor color2 = sensor2.getColor();

            // If both sensors agree, that's our answer!
            if (color1 == color2) {
                return color1;
            }

            // If they disagree, prioritize non-UNKNOWN readings
            // (One sensor probably sees the hole in the artifact)
            if (color1 != ArtifactColor.UNKNOWN && color2 == ArtifactColor.UNKNOWN) {
                return color1;
            }
            if (color2 != ArtifactColor.UNKNOWN && color1 == ArtifactColor.UNKNOWN) {
                return color2;
            }

            // If both sensors have different non-UNKNOWN readings (unusual but possible),
            // we need a tiebreaker. Options:
            // - Return sensor1's reading (simple default)
            // - Return UNKNOWN (conservative approach)
            // - Add more sophisticated logic based on confidence/sample counts
            // For now, let's be conservative and return UNKNOWN in this edge case.
            if (color1 != ArtifactColor.UNKNOWN && color2 != ArtifactColor.UNKNOWN) {
                // Log this unusual situation
                telemetry.addData(slotName + " CONFLICT", "%s vs %s - using UNKNOWN", color1, color2);
                return ArtifactColor.UNKNOWN;
            }

            // Fallback (should never reach here, but just in case)
            return ArtifactColor.UNKNOWN;
        }

        /**
         * Adds detailed telemetry for both sensors in this pair.
         */
        public void addTelemetry() {
            telemetry.addLine("--- " + slotName + " Slot ---");
            sensor1.addTelemetry();
            sensor2.addTelemetry();
            telemetry.addData(slotName + " FUSED", "Occupied: %s, Color: %s",
                    isOccupied(), getFusedColor());
        }
    }


    // =================================================================================
    // --- IntakeSensorFusion Manager Class ---
    // This is the public-facing part of the utility.
    // =================================================================================

    private final ArtifactSensorPair leftSlot;
    private final ArtifactSensor centerSlot;  // UPDATED: Single sensor for center
    private final ArtifactSensorPair rightSlot;
    private final List<Object> allSlots = new ArrayList<>();  // Can hold both types
    private final Telemetry telemetry;

    /**
     * Constructor for the main sensor fusion manager.
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry The telemetry object from the OpMode.
     */
    public IntakeSensorFusion001(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize sensor pairs for LEFT and RIGHT slots.
        // The names MUST match your robot configuration:
        // - color_sensor_left1
        // - color_sensor_left2
        // - color_sensor_right1
        // - color_sensor_right2
        leftSlot = new ArtifactSensorPair("LEFT",
                "color_sensor_left1", "color_sensor_left2", hardwareMap, telemetry);
        rightSlot = new ArtifactSensorPair("RIGHT",
                "color_sensor_right1", "color_sensor_right2", hardwareMap, telemetry);

        // UPDATED: Initialize single sensor for CENTER slot
        // The name MUST match your robot configuration: color_sensor_center
        centerSlot = new ArtifactSensor("color_sensor_center", hardwareMap, telemetry);

        allSlots.add(leftSlot);
        allSlots.add(centerSlot);
        allSlots.add(rightSlot);
    }

    /**
     * This method MUST be called in every loop of your OpMode.
     * It updates the state of all sensors (5 physical sensors total: 2+1+2).
     */
    public void update() {
        leftSlot.update();
        centerSlot.update();  // UPDATED: Update center sensor
        rightSlot.update();
    }

    /**
     * Gets the fused/determined color of the artifact in a specific slot.
     * This uses intelligent sensor fusion for LEFT/RIGHT slots to handle cases where one sensor sees the hole.
     * For CENTER slot, returns the single sensor's reading.
     * @param slot The IntakeSlot to check (IntakeSlot.LEFT, IntakeSlot.CENTER, or IntakeSlot.RIGHT).
     * @return The ArtifactColor (PURPLE, GREEN, or UNKNOWN if empty/undetermined).
     */
    public ArtifactColor getColorOfSlot(IntakeSlot slot) {
        switch (slot) {
            case LEFT: return leftSlot.getFusedColor();
            case CENTER: return centerSlot.getColor();  // UPDATED: Direct read from single sensor
            case RIGHT: return rightSlot.getFusedColor();
            default: return ArtifactColor.UNKNOWN;
        }
    }

    /**
     * Checks if a specific slot is currently occupied by an artifact.
     * For LEFT/RIGHT: A slot is considered occupied if EITHER of its sensors detects something.
     * For CENTER: Direct reading from the single sensor.
     * @param slot The IntakeSlot to check.
     * @return true if an artifact is detected, false otherwise.
     */
    public boolean isSlotOccupied(IntakeSlot slot) {
        switch (slot) {
            case LEFT: return leftSlot.isOccupied();
            case CENTER: return centerSlot.isOccupied();  // UPDATED: Direct read from single sensor
            case RIGHT: return rightSlot.isOccupied();
            default: return false;
        }
    }

    /**
     * Provides a list of all currently held artifacts, useful for quick inventory checks.
     * @return A List of ArtifactColor representing the current inventory from all 3 slots.
     */
    public List<ArtifactColor> getInventory() {
        List<ArtifactColor> inventory = new ArrayList<>();

        // Check LEFT slot
        if (leftSlot.isOccupied()) {
            inventory.add(leftSlot.getFusedColor());
        }

        // UPDATED: Check CENTER slot
        if (centerSlot.isOccupied()) {
            inventory.add(centerSlot.getColor());
        }

        // Check RIGHT slot
        if (rightSlot.isOccupied()) {
            inventory.add(rightSlot.getFusedColor());
        }

        return inventory;
    }

    /**
     * Adds detailed telemetry for all sensor slots to the Driver Station.
     * Shows individual sensor readings AND the fused result for each slot.
     */
    public void addTelemetry() {
        telemetry.addLine("=== INTAKE INVENTORY ===");
        leftSlot.addTelemetry();
        telemetry.addLine(); // Blank line for readability

        // UPDATED: Add CENTER slot telemetry
        telemetry.addLine("--- CENTER Slot ---");
        centerSlot.addTelemetry();
        telemetry.addLine();

        rightSlot.addTelemetry();
        telemetry.addLine();
        telemetry.addData("Total Artifacts", getInventory().size());
        telemetry.addData("Inventory", getInventory().toString());
    }

    /**
     * Optional: Get detailed debug information about a specific slot.
     * Useful for testing and calibration.
     */
    public void addDetailedTelemetryForSlot(IntakeSlot slot) {
        switch (slot) {
            case LEFT:
                leftSlot.addTelemetry();
                break;
            case CENTER:  // UPDATED: Add CENTER case
                telemetry.addLine("--- CENTER Slot ---");
                centerSlot.addTelemetry();
                break;
            case RIGHT:
                rightSlot.addTelemetry();
                break;
        }
    }
}