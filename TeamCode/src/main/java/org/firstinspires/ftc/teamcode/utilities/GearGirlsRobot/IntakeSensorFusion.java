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
 * This class handles sensor fusion, color averaging, and state management for each slot.
 */
public class IntakeSensorFusion {

    // Public enums for easy access from other classes
    public enum ArtifactColor { PURPLE, GREEN, UNKNOWN }
    public enum IntakeSlot { LEFT_1, LEFT_2, RIGHT_1, RIGHT_2 }

    // The core worker class that manages a single sensor
    private static class ArtifactSensor {
        private enum DetectionState { WAITING, SAMPLING, DETERMINED }
        private DetectionState currentState = DetectionState.WAITING;

        private final NormalizedColorSensor colorSensor;
        private final DistanceSensor distanceSensor;
        private final String name;
        private final Telemetry telemetry;

        private final ElapsedTime samplingTimer = new ElapsedTime();
        private double totalRed = 0, totalGreen = 0, totalBlue = 0;
        private int sampleCount = 0;

        private ArtifactColor determinedColor = ArtifactColor.UNKNOWN;
        private boolean isOccupied = false;

        // Constants (can be moved to a separate file)
        private static final double DISTANCE_THRESHOLD_CM = 5.0; // How close to detect
        private static final double SAMPLING_DURATION_MSEC = 300; // How long to sample

        public ArtifactSensor(String sensorName, HardwareMap hardwareMap, Telemetry telemetry) {
            this.name = sensorName;
            this.telemetry = telemetry;
            try {
                colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorName);
                distanceSensor = (DistanceSensor) colorSensor;
                colorSensor.setGain(20);
            } catch (Exception e) {
                throw new RuntimeException("Could not find sensor: " + sensorName);
            }
        }

        public void update() {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            isOccupied = (distance < DISTANCE_THRESHOLD_CM);

            switch (currentState) {
                case WAITING:
                    if (isOccupied) {
                        // An artifact has entered the slot. Start sampling.
                        totalRed = 0; totalGreen = 0; totalBlue = 0;
                        sampleCount = 0;
                        samplingTimer.reset();
                        currentState = DetectionState.SAMPLING;
                    } else {
                        // If the slot is empty, ensure the color is UNKNOWN.
                        determinedColor = ArtifactColor.UNKNOWN;
                    }
                    break;

                case SAMPLING:
                    if (!isOccupied) {
                        // Artifact was removed mid-sample. Reset.
                        currentState = DetectionState.WAITING;
                        break;
                    }

                    if (samplingTimer.milliseconds() < SAMPLING_DURATION_MSEC) {
                        // Collect data
                        NormalizedRGBA colors = colorSensor.getNormalizedColors();
                        totalRed += colors.red;
                        totalGreen += colors.green;
                        totalBlue += colors.blue;
                        sampleCount++;
                    } else {
                        // Sampling is complete. Determine the color.
                        determineColor();
                        currentState = DetectionState.DETERMINED;
                    }
                    break;

                case DETERMINED:
                    if (!isOccupied) {
                        // Artifact has been removed. Reset the state machine.
                        currentState = DetectionState.WAITING;
                    }
                    // Otherwise, do nothing. The color is determined and will remain
                    // until the artifact is removed.
                    break;
            }
        }

        private void determineColor() {
            if (sampleCount == 0) {
                this.determinedColor = ArtifactColor.UNKNOWN;
                return;
            }
            double avgRed = totalRed / sampleCount;
            double avgGreen = totalGreen / sampleCount;
            double avgBlue = totalBlue / sampleCount;

            // Simplified logic: Purple is a mix of Red and Blue.
            // We check if Green is the lowest value, and Red/Blue are strong.
            // This needs to be tuned with real sensor values!
            if (avgGreen > avgRed && avgGreen > avgBlue) {
                this.determinedColor = ArtifactColor.GREEN;
            } else if ((avgRed + avgBlue) > avgGreen * 1.5) { // Check if Red+Blue is significantly stronger than Green
                this.determinedColor = ArtifactColor.PURPLE;
            } else {
                this.determinedColor = ArtifactColor.UNKNOWN;
            }
        }

        public ArtifactColor getColor() { return isOccupied ? determinedColor : ArtifactColor.UNKNOWN; }
        public boolean isOccupied() { return isOccupied; }

        public void addTelemetry() {
            telemetry.addData(name, "Occupied: %s, Color: %s", isOccupied(), getColor());
        }
    }

    // --- IntakeSensorFusion Manager Class ---

    private final ArtifactSensor leftSlot1;
    private final ArtifactSensor leftSlot2;
    private final ArtifactSensor rightSlot1;
    private final ArtifactSensor rightSlot2;
    private final List<ArtifactSensor> allSensors = new ArrayList<>();
    private final Telemetry telemetry;

    public IntakeSensorFusion(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Initialize one ArtifactSensor for each physical sensor on the robot
        leftSlot1 = new ArtifactSensor("sensor_L1", hardwareMap, telemetry);
        leftSlot2 = new ArtifactSensor("sensor_L2", hardwareMap, telemetry);
        rightSlot1 = new ArtifactSensor("sensor_R1", hardwareMap, telemetry);
        rightSlot2 = new ArtifactSensor("sensor_R2", hardwareMap, telemetry);

        allSensors.add(leftSlot1);
        allSensors.add(leftSlot2);
        allSensors.add(rightSlot1);
        allSensors.add(rightSlot2);
    }

    /**
     * This method MUST be called in every loop of the OpMode.
     * It updates the state of all individual sensors.
     */
    public void update() {
        for (ArtifactSensor sensor : allSensors) {
            sensor.update();
        }
    }

    /**
     * Gets the determined color of the artifact in a specific slot.
     * @param slot The IntakeSlot to check.
     * @return The ArtifactColor (PURPLE, GREEN, or UNKNOWN if empty/undetermined).
     */
    public ArtifactColor getColorOfSlot(IntakeSlot slot) {
        switch (slot) {
            case LEFT_1: return leftSlot1.getColor();
            case LEFT_2: return leftSlot2.getColor();
            case RIGHT_1: return rightSlot1.getColor();
            case RIGHT_2: return rightSlot2.getColor();
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
            case LEFT_1: return leftSlot1.isOccupied();
            case LEFT_2: return leftSlot2.isOccupied();
            case RIGHT_1: return rightSlot1.isOccupied();
            case RIGHT_2: return rightSlot2.isOccupied();
            default: return false;
        }
    }

    /**
     * Provides a list of all currently held artifacts.
     * @return A List of ArtifactColor representing the inventory.
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
     * Adds detailed telemetry for all four sensor slots.
     */
    public void addTelemetry() {
        telemetry.addLine("--- Intake Inventory ---");
        leftSlot1.addTelemetry();
        leftSlot2.addTelemetry();
        rightSlot1.addTelemetry();
        rightSlot2.addTelemetry();
        telemetry.addData("Total Artifacts", getInventory().size());
    }
}
