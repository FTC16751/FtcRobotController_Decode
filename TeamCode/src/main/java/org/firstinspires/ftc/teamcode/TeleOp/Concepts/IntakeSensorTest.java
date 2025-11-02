package org.firstinspires.ftc.teamcode.TeleOp.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

/**
 * IntakeSensorTest is a simple TeleOp OpMode for debugging and verifying the
 * four color/distance sensors in the robot's intake system.
 *
 * PURPOSE:
 * 1.  Verify that all four sensors (`sensor_L1`, `sensor_L2`, `sensor_R1`, `sensor_R2`) are
 *     correctly configured and connected.
 * 2.  Display real-time Distance, RGB, and Gain values for each sensor.
 * 3.  Allow for easy tuning of distance thresholds by placing an artifact in each slot
 *     and observing the reported distance.
 * 4.  Allow for easy tuning of color detection logic by observing the RGB values
 *     for both PURPLE and GREEN artifacts under competition lighting.
 *
 * USAGE:
 * - Run this OpMode.
 * - The Driver Station will display a block of telemetry for each of the four sensors.
 * - Place an artifact in one of the intake slots to see its color and distance values.
 *
 * @author Your Team Name
 */
@TeleOp(name = "Test: Intake Sensors", group = "Test")
public class IntakeSensorTest extends OpMode {

    // A map to hold our sensor objects, pairing the configuration name with the sensor object.
    // This makes it easy to loop through them without writing repetitive code.
    private final Map<String, NormalizedColorSensor> colorSensors = new HashMap<>();
    private final Map<String, DistanceSensor> distanceSensors = new HashMap<>();

    // The names of the sensors as configured on the Robot Controller.
    private final String[] SENSOR_NAMES = {"sensor_L1", "sensor_L2", "sensor_R1", "sensor_R2"};

    /**
     * Code to run ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        telemetry.addLine("Initializing Intake Sensors...");
        telemetry.update();

        // Loop through the sensor names to initialize each one.
        for (String name : SENSOR_NAMES) {
            try {
                // Get the color sensor object from the hardware map.
                NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, name);

                // Set a gain value. Higher gain is better for detecting color, but can
                // saturate (max out) if the object is too close or reflective. 20 is a good starting point.
                sensor.setGain(20.0f);

                // Add the sensor object to our maps.
                colorSensors.put(name, sensor);
                distanceSensors.put(name, (DistanceSensor) sensor);

                telemetry.addData(name, "Initialized successfully!");
            } catch (Exception e) {
                // If a sensor is not found, report the error clearly.
                telemetry.addData("ERROR", "Could not find or initialize sensor: " + name);
            }
        }
        telemetry.addLine("Initialization Complete. Ready for Start.");
        telemetry.update();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START.
     */
    @Override
    public void loop() {
        // Loop through each sensor on every cycle to get and display its data.
        for (String name : SENSOR_NAMES) {
            NormalizedColorSensor colorSensor = colorSensors.get(name);
            DistanceSensor distanceSensor = distanceSensors.get(name);

            // Add a separator line for readability.
            telemetry.addLine("--- " + name.toUpperCase() + " ---");

            if (colorSensor != null && distanceSensor != null) {
                // Get the normalized color values.
                // Normalized colors return a value between 0.0 and 1.0 for each channel.
                NormalizedRGBA colors = colorSensor.getNormalizedColors();

                // Get the distance reading in centimeters.
                double distance = distanceSensor.getDistance(DistanceUnit.CM);

                // --- Display all valuable output ---

                // 1. Distance: The most important value for detecting presence.
                telemetry.addData("Distance (cm)", "%.2f", distance);

                // 2. Occupancy Check: A simple true/false based on a threshold.
                //    This helps you tune your DISTANCE_THRESHOLD_CM constant.
                boolean isOccupied = (distance < 5.0); // Using 5.0cm as a sample threshold.
                telemetry.addData("Is Occupied?", isOccupied);

                // 3. Raw RGB Values: Essential for tuning your color logic.
                telemetry.addData("Red", "%.3f", colors.red);
                telemetry.addData("Green", "%.3f", colors.green);
                telemetry.addData("Blue", "%.3f", colors.blue);

                // 4. Alpha (Brightness): Useful for checking if the sensor is saturated (reading is too bright).
                //    If Alpha is 1.0, you might need to lower the gain.
                telemetry.addData("Alpha (Brightness)", "%.3f", colors.alpha);

                // 5. Gain: Display the current gain setting to confirm it was set correctly.
                telemetry.addData("Gain", "%.1f", colorSensor.getGain());

            } else {
                // If the sensor failed to initialize, show an error message.
                telemetry.addData("Status", "ERROR - Sensor not found");
            }
        }
    }
}
