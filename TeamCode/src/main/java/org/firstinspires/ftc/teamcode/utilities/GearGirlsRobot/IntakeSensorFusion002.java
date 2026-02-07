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
 * picture of all artifacts currently held by the robot.
 *
 * VERSION 003: Added hysteresis to prevent ghost readings after firing
 * and force-clear capability for manual slot clearing.
 */
public class IntakeSensorFusion002 {

    // Public enums for easy access from other classes
    public enum ArtifactColor {
        PURPLE,   // Definitely purple
        GREEN,    // Definitely green
        UNKNOWN,  // Artifact present but can't determine color (e.g., seeing hole)
        EMPTY     // No artifact present (slot is empty)
    }
    public enum IntakeSlot { LEFT, CENTER, RIGHT }

    /**
     * A private inner class that manages a single physical color sensor.
     * Uses direct color reads with hysteresis to prevent ghost readings.
     */
    private static class ArtifactSensor {
        private enum DetectionState { EMPTY, OCCUPIED }
        private DetectionState currentState = DetectionState.EMPTY;

        // Hardware and Configuration
        private final NormalizedColorSensor colorSensor;
        private final DistanceSensor distanceSensor;
        private final String name;
        private final Telemetry telemetry;

        // Stabilization tracking
        private final ElapsedTime stabilizationTimer = new ElapsedTime();
        private boolean needsStabilization = false;

        // Output variables
        private ArtifactColor determinedColor = ArtifactColor.EMPTY;

        // Constants with HYSTERESIS to prevent flickering
        private static final double DISTANCE_OCCUPIED_THRESHOLD_CM = 6.0;  // Ball must be this close to register as arrived
        private static final double DISTANCE_EMPTY_THRESHOLD_CM = 8.0;     // Ball must be this far to register as gone
        private static final double STABILIZATION_DELAY_MS = 50;

        /**
         * Constructor for a single sensor.
         */
        public ArtifactSensor(String sensorName, HardwareMap hardwareMap, Telemetry telemetry) {
            this.name = sensorName;
            this.telemetry = telemetry;
            try {
                colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorName);
                distanceSensor = (DistanceSensor) colorSensor;
                colorSensor.setGain(20); // Higher gain for better color sensitivity
            } catch (Exception e) {
                throw new RuntimeException("Could not find or initialize sensor: " + sensorName);
            }
        }

        /**
         * This method is called on every loop cycle.
         */
        public void update() {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            switch (currentState) {
                case EMPTY:
                    // Use stricter threshold to register arrival
                    if (distance < DISTANCE_OCCUPIED_THRESHOLD_CM) {
                        // Artifact just arrived - start stabilization timer
                        needsStabilization = true;
                        stabilizationTimer.reset();
                        currentState = DetectionState.OCCUPIED;
                        // Don't read color yet, wait for stabilization
                    } else {
                        // Slot is empty - make sure we report EMPTY, not UNKNOWN
                        determinedColor = ArtifactColor.EMPTY;
                    }
                    break;

                case OCCUPIED:
                    // Use looser threshold to register departure (hysteresis)
                    if (distance > DISTANCE_EMPTY_THRESHOLD_CM) {
                        // Artifact removed - reset to EMPTY
                        determinedColor = ArtifactColor.EMPTY;
                        currentState = DetectionState.EMPTY;
                        needsStabilization = false;
                    } else {
                        // Artifact still present - read color
                        if (needsStabilization) {
                            // Optional: Wait for brief stabilization period
                            if (stabilizationTimer.milliseconds() >= STABILIZATION_DELAY_MS) {
                                needsStabilization = false;
                            }
                        }

                        // Read color directly (even during stabilization for faster response)
                        readColorNow();
                    }
                    break;
            }
        }

        /**
         * Reads the color sensor directly and determines artifact color.
         * No averaging - instant read of current state.
         */
        private void readColorNow() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Color detection logic - tuned based on actual artifact data
            if (colors.green > colors.red && colors.green > colors.blue) {
                this.determinedColor = ArtifactColor.GREEN;
            } else if (colors.blue > colors.red && colors.blue > colors.green) {
                this.determinedColor = ArtifactColor.PURPLE;
            } else {
                // If no clear winner, mark as UNKNOWN (artifact present but ambiguous)
                // This happens when sensor sees the hole or ambiguous colors
                this.determinedColor = ArtifactColor.UNKNOWN;
            }
        }

        /**
         * Force this sensor to register as empty.
         * Used after firing when we know the ball is gone but sensor hasn't caught up.
         */
        public void forceEmpty() {
            this.determinedColor = ArtifactColor.EMPTY;
            this.currentState = DetectionState.EMPTY;
            this.needsStabilization = false;
        }

        // Public getters
        public ArtifactColor getColor() { return determinedColor; }

        public boolean isOccupied() {
            // Use the stricter threshold for occupancy check
            return distanceSensor.getDistance(DistanceUnit.CM) < DISTANCE_OCCUPIED_THRESHOLD_CM;
        }

        public String getName() { return name; }

        // Telemetry for debugging
        public void addTelemetry() {
            double dist = distanceSensor.getDistance(DistanceUnit.CM);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            telemetry.addData(name, "Dist: %.1fcm, Color: %s, State: %s",
                    dist, determinedColor, currentState);
            telemetry.addData(name + " RGB", "R:%.2f G:%.2f B:%.2f",
                    colors.red, colors.green, colors.blue);
        }
    }

    /**
     * A private inner class that manages a PAIR of sensors for a single intake slot.
     * Provides redundancy when one sensor sees the hole in the artifact.
     */
    private static class ArtifactSensorPair {
        private final ArtifactSensor sensor1;
        private final ArtifactSensor sensor2;
        private final String slotName;
        private final Telemetry telemetry;

        public ArtifactSensorPair(String slotName, String sensor1Name, String sensor2Name,
                                  HardwareMap hardwareMap, Telemetry telemetry) {
            this.slotName = slotName;
            this.telemetry = telemetry;
            this.sensor1 = new ArtifactSensor(sensor1Name, hardwareMap, telemetry);
            this.sensor2 = new ArtifactSensor(sensor2Name, hardwareMap, telemetry);
        }

        public void update() {
            sensor1.update();
            sensor2.update();
        }

        /**
         * Slot is occupied if EITHER sensor detects something.
         */
        public boolean isOccupied() {
            return sensor1.isOccupied() || sensor2.isOccupied();
        }

        /**
         * Fuses color readings from both sensors with PURPLE priority.
         *
         * Priority order when sensors disagree:
         * 1. PURPLE > GREEN (prefer purple when conflict)
         * 2. Definite color > UNKNOWN > EMPTY
         *
         * This ensures we always give a decisive answer and never get stuck.
         */
        public ArtifactColor getFusedColor() {
            ArtifactColor color1 = sensor1.getColor();
            ArtifactColor color2 = sensor2.getColor();

            // Both say empty - slot is definitely empty
            if (color1 == ArtifactColor.EMPTY && color2 == ArtifactColor.EMPTY) {
                return ArtifactColor.EMPTY;
            }

            // Both agree on the same color - easy case
            if (color1 == color2) {
                return color1;
            }

            // PURPLE PRIORITY: If either sensor sees purple, trust it
            if (color1 == ArtifactColor.PURPLE) {
                return ArtifactColor.PURPLE;
            }
            if (color2 == ArtifactColor.PURPLE) {
                return ArtifactColor.PURPLE;
            }

            // GREEN: If either sensor sees green (and neither saw purple), trust it
            if (color1 == ArtifactColor.GREEN) {
                return ArtifactColor.GREEN;
            }
            if (color2 == ArtifactColor.GREEN) {
                return ArtifactColor.GREEN;
            }

            // UNKNOWN: If either sensor sees something ambiguous, trust that
            if (color1 == ArtifactColor.UNKNOWN) {
                return ArtifactColor.UNKNOWN;
            }
            if (color2 == ArtifactColor.UNKNOWN) {
                return ArtifactColor.UNKNOWN;
            }

            // One sensor sees EMPTY, the other doesn't - trust the occupied sensor
            if (color1 == ArtifactColor.EMPTY) {
                return color2;
            }
            if (color2 == ArtifactColor.EMPTY) {
                return color1;
            }

            // Shouldn't get here, but default to UNKNOWN
            telemetry.addData(slotName + " EDGE CASE", "%s vs %s", color1, color2);
            return ArtifactColor.UNKNOWN;
        }

        /**
         * Force both sensors in this pair to register as empty.
         */
        public void forceEmpty() {
            sensor1.forceEmpty();
            sensor2.forceEmpty();
        }

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
    // =================================================================================

    private final ArtifactSensorPair leftSlot;
    private final ArtifactSensor centerSlot;
    private final ArtifactSensorPair rightSlot;
    private final Telemetry telemetry;

    /**
     * Constructor for the main sensor fusion manager.
     */
    public IntakeSensorFusion002(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize sensor pairs for LEFT and RIGHT slots
        leftSlot = new ArtifactSensorPair("LEFT",
                "color_sensor_left1", "color_sensor_left2", hardwareMap, telemetry);
        rightSlot = new ArtifactSensorPair("RIGHT",
                "color_sensor_right1", "color_sensor_right2", hardwareMap, telemetry);

        // Initialize single sensor for CENTER slot
        centerSlot = new ArtifactSensor("color_sensor_center", hardwareMap, telemetry);
    }

    /**
     * MUST be called in every loop of your OpMode.
     */
    public void update() {
        leftSlot.update();
        centerSlot.update();
        rightSlot.update();
    }

    /**
     * Gets the color of the artifact in a specific slot.
     * Returns EMPTY if no artifact is present, UNKNOWN if artifact is present but color is ambiguous.
     */
    public ArtifactColor getColorOfSlot(IntakeSlot slot) {
        switch (slot) {
            case LEFT: return leftSlot.getFusedColor();
            case CENTER: return centerSlot.getColor();
            case RIGHT: return rightSlot.getFusedColor();
            default: return ArtifactColor.EMPTY;
        }
    }

    /**
     * Checks if a specific slot is currently occupied.
     * Returns true if an artifact is physically present (regardless of whether we know its color).
     */
    public boolean isSlotOccupied(IntakeSlot slot) {
        switch (slot) {
            case LEFT: return leftSlot.isOccupied();
            case CENTER: return centerSlot.isOccupied();
            case RIGHT: return rightSlot.isOccupied();
            default: return false;
        }
    }

    /**
     * Force a slot to register as empty.
     * Used after firing when sensors haven't caught up yet.
     * This prevents "ghost readings" where sensors still see color from a ball that was just fired.
     */
    public void forceSlotEmpty(IntakeSlot slot) {
        switch (slot) {
            case LEFT:
                leftSlot.forceEmpty();
                break;
            case CENTER:
                centerSlot.forceEmpty();
                break;
            case RIGHT:
                rightSlot.forceEmpty();
                break;
        }
    }

    /**
     * Provides a list of all currently held artifacts (only includes slots with known colors).
     * EMPTY and UNKNOWN artifacts are excluded from inventory.
     * @return A List of ArtifactColor representing only GREEN and PURPLE artifacts.
     */
    public List<ArtifactColor> getInventory() {
        List<ArtifactColor> inventory = new ArrayList<>();

        // Only add artifacts with determined colors to inventory
        ArtifactColor leftColor = leftSlot.getFusedColor();
        if (leftColor == ArtifactColor.GREEN || leftColor == ArtifactColor.PURPLE) {
            inventory.add(leftColor);
        }

        ArtifactColor centerColor = centerSlot.getColor();
        if (centerColor == ArtifactColor.GREEN || centerColor == ArtifactColor.PURPLE) {
            inventory.add(centerColor);
        }

        ArtifactColor rightColor = rightSlot.getFusedColor();
        if (rightColor == ArtifactColor.GREEN || rightColor == ArtifactColor.PURPLE) {
            inventory.add(rightColor);
        }

        return inventory;
    }

    /**
     * Counts how many purple artifacts are currently held.
     */
    public int getPurpleCount() {
        int count = 0;
        if (leftSlot.getFusedColor() == ArtifactColor.PURPLE) count++;
        if (centerSlot.getColor() == ArtifactColor.PURPLE) count++;
        if (rightSlot.getFusedColor() == ArtifactColor.PURPLE) count++;
        return count;
    }

    /**
     * Counts how many green artifacts are currently held.
     */
    public int getGreenCount() {
        int count = 0;
        if (leftSlot.getFusedColor() == ArtifactColor.GREEN) count++;
        if (centerSlot.getColor() == ArtifactColor.GREEN) count++;
        if (rightSlot.getFusedColor() == ArtifactColor.GREEN) count++;
        return count;
    }

    /**
     * Checks if the intake is completely full (all 3 slots occupied with identified artifacts).
     */
    public boolean isFull() {
        return getInventory().size() == 3;
    }

    /**
     * Checks if the intake is completely empty (no artifacts in any slot).
     */
    public boolean isEmpty() {
        return leftSlot.getFusedColor() == ArtifactColor.EMPTY &&
                centerSlot.getColor() == ArtifactColor.EMPTY &&
                rightSlot.getFusedColor() == ArtifactColor.EMPTY;
    }

    /**
     * Adds detailed telemetry for all sensor slots.
     */
    public void addTelemetry() {
        telemetry.addLine("=== INTAKE INVENTORY ===");
        leftSlot.addTelemetry();
        telemetry.addLine();

        telemetry.addLine("--- CENTER Slot ---");
        centerSlot.addTelemetry();
        telemetry.addLine();

        rightSlot.addTelemetry();
        telemetry.addLine();
        telemetry.addData("Total Identified", getInventory().size());
        telemetry.addData("Purple Count", getPurpleCount());
        telemetry.addData("Green Count", getGreenCount());
        telemetry.addData("Inventory", getInventory().toString());
        telemetry.addData("Full?", isFull());
        telemetry.addData("Empty?", isEmpty());
    }

    /**
     * Get detailed debug information about a specific slot.
     */
    public void addDetailedTelemetryForSlot(IntakeSlot slot) {
        switch (slot) {
            case LEFT:
                leftSlot.addTelemetry();
                break;
            case CENTER:
                telemetry.addLine("--- CENTER Slot ---");
                centerSlot.addTelemetry();
                break;
            case RIGHT:
                rightSlot.addTelemetry();
                break;
        }
    }
}