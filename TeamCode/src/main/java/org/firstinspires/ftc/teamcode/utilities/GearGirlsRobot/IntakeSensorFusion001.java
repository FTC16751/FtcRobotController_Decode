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
 * color averaging, and state management for each individual sensor slot.
 */

public class IntakeSensorFusion001 {

    // Public enums for easy access from other classes like your OpMode.
    public enum ArtifactColor { PURPLE, GREEN, UNKNOWN }
    public enum IntakeSlot {LEFT, RIGHT, CENTER }

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
         * Constructor for a single sensor slot.
         * @param sensorName The name of the sensor in the robot configuration (e.g., "sensor_L1").
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
        public ArtifactColor getColor() { return isOccupied ? determinedColor : ArtifactColor.UNKNOWN; }
        public boolean isOccupied() { return isOccupied; }

        // Telemetry for debugging this specific sensor.
        public void addTelemetry() {
            telemetry.addData(name, "Occupied: %s, Color: %s", isOccupied(), getColor());
        }
    }


    // =================================================================================
    // --- IntakeSensorFusion Manager Class ---
    // This is the public-facing part of the utility.
    // =================================================================================

    private final ArtifactSensor leftSlot;
    private final ArtifactSensor rightSlot;
//    private final ArtifactSensor rightSlot1;
//    private final ArtifactSensor rightSlot2;
    private final List<ArtifactSensor> allSensors = new ArrayList<>();
    private final Telemetry telemetry;

    /**
     * Constructor for the main sensor fusion manager.
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry The telemetry object from the OpMode.
     */
    public IntakeSensorFusion001(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Initialize one ArtifactSensor for each physical sensor on the robot.
        // The names MUST match your robot configuration.
        leftSlot = new ArtifactSensor("sensor_color", hardwareMap, telemetry);
        rightSlot = new ArtifactSensor("sensor_color2", hardwareMap, telemetry);

        allSensors.add(leftSlot);
        allSensors.add(rightSlot);
    }

    /**
     * This method MUST be called in every loop of your OpMode.
     * It updates the state of all four individual sensors.
     */
    public void update() {
        for (ArtifactSensor sensor : allSensors) {
            sensor.update();
        }
    }

    /**
     * Gets the determined color of the artifact in a specific slot.
     * @param slot The IntakeSlot to check (e.g., IntakeSlot.LEFT_1).
     * @return The ArtifactColor (PURPLE, GREEN, or UNKNOWN if empty/undetermined).
     */
    public ArtifactColor getColorOfSlot(IntakeSlot slot) {
        switch (slot) {
            case LEFT: return leftSlot.getColor();
            case RIGHT: return rightSlot.getColor();
            default: return ArtifactColor.UNKNOWN;
        }
    }

    /**
     * Checks if a specific slot is currently occupied by an artifact.
     * @param slot The IntakeSlot to check.
     * @return true if an artifact is detected, false otherwise.
     */
    public boolean isSlotOccupied(IntakeSlot slot) {
        switch (slot) {
            case LEFT: return leftSlot.isOccupied();
            case RIGHT: return rightSlot.isOccupied();
            default: return false;
        }
    }

    /**
     * Provides a list of all currently held artifacts, useful for quick inventory checks.
     * @return A List of ArtifactColor representing the current inventory.
     */
    public List<ArtifactColor> getInventory() {
        List<ArtifactColor> inventory = new ArrayList<>();
        for (ArtifactSensor sensor : allSensors) {
            if (sensor.isOccupied()) {
                inventory.add(sensor.getColor());
            }
        }
        return inventory;
    }

    /**
     * Adds detailed telemetry for all four sensor slots to the Driver Station.
     */
    public void addTelemetry() {
        telemetry.addLine("--- Intake Inventory ---");
        leftSlot.addTelemetry();
        rightSlot.addTelemetry();
        telemetry.addData("Total Artifacts", getInventory().size());
    }
}

