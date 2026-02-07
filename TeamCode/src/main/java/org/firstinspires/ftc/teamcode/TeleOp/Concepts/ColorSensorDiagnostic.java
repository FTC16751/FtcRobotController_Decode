package org.firstinspires.ftc.teamcode.TeleOp.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * ColorSensorDiagnostic - A diagnostic tool to help you tune your color detection thresholds.
 *
 * PURPOSE:
 * This OpMode shows you the RAW RGB values from each of your FIVE color sensors in real-time.
 * Use this to collect data about what your purple and green artifacts actually look like
 * to the sensors, so you can tune the color detection logic in IntakeSensorFusion002.java.
 *
 * UPDATED for 3-slot system: LEFT (dual), CENTER (single), RIGHT (dual)
 *
 * USAGE:
 * 1. Run this OpMode on your robot.
 * 2. Hold a PURPLE artifact in front of each slot and write down the RGB values.
 * 3. Hold a GREEN artifact in front of each slot and write down the RGB values.
 * 4. Look for patterns:
 *    - What is the typical Red/Green/Blue ratio for purple? (e.g., R:0.3, G:0.1, B:0.4)
 *    - What is the typical Red/Green/Blue ratio for green? (e.g., R:0.1, G:0.5, B:0.1)
 * 5. Use those patterns to update the readColorNow() method in IntakeSensorFusion002.java
 *
 * TIPS:
 * - The values are NORMALIZED (0.0 to 1.0), not raw (0-255).
 * - Distance shows how close the artifact is in centimeters.
 * - Alpha is the overall brightness - you can usually ignore it.
 * - Test in the same lighting conditions you'll compete in!
 *
 * @author Your Team Name
 */
@TeleOp(name = "Diagnostic: Color Sensor Tuning", group = "Diagnostic")
public class ColorSensorDiagnostic extends OpMode {

    // Hardware references
    private NormalizedColorSensor colorSensorLeft1;
    private NormalizedColorSensor colorSensorLeft2;
    private NormalizedColorSensor colorSensorCenter;  // NEW
    private NormalizedColorSensor colorSensorRight1;
    private NormalizedColorSensor colorSensorRight2;

    private DistanceSensor distanceSensorLeft1;
    private DistanceSensor distanceSensorLeft2;
    private DistanceSensor distanceSensorCenter;  // NEW
    private DistanceSensor distanceSensorRight1;
    private DistanceSensor distanceSensorRight2;

    @Override
    public void init() {
        telemetry.addLine("Initializing Color Sensors for Diagnostic...");
        telemetry.update();

        try {
            // Initialize all FIVE sensors
            // IMPORTANT: These names must match your robot configuration!
            colorSensorLeft1 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_left1");
            colorSensorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_left2");
            colorSensorCenter = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_center");  // NEW
            colorSensorRight1 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_right1");
            colorSensorRight2 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_right2");

            // Cast to distance sensors
            distanceSensorLeft1 = (DistanceSensor) colorSensorLeft1;
            distanceSensorLeft2 = (DistanceSensor) colorSensorLeft2;
            distanceSensorCenter = (DistanceSensor) colorSensorCenter;  // NEW
            distanceSensorRight1 = (DistanceSensor) colorSensorRight1;
            distanceSensorRight2 = (DistanceSensor) colorSensorRight2;

            // Set the same gain as used in IntakeSensorFusion002
            colorSensorLeft1.setGain(20);
            colorSensorLeft2.setGain(20);
            colorSensorCenter.setGain(20);  // NEW
            colorSensorRight1.setGain(20);
            colorSensorRight2.setGain(20);

            telemetry.addLine("All 5 sensors initialized successfully!");
            telemetry.addLine();
            telemetry.addLine("Instructions:");
            telemetry.addLine("1. Hold artifacts in front of each slot");
            telemetry.addLine("2. Record the RGB values shown below");
            telemetry.addLine("3. Use these values to tune color detection");
            telemetry.addLine();
            telemetry.addLine("Ready to start!");

        } catch (Exception e) {
            telemetry.addData("FATAL ERROR", "Could not initialize sensors!");
            telemetry.addData("Reason", e.getMessage());
            telemetry.addLine();
            telemetry.addLine("Check your robot configuration!");
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        // Read all FIVE sensors
        NormalizedRGBA colorsLeft1 = colorSensorLeft1.getNormalizedColors();
        NormalizedRGBA colorsLeft2 = colorSensorLeft2.getNormalizedColors();
        NormalizedRGBA colorsCenter = colorSensorCenter.getNormalizedColors();  // NEW
        NormalizedRGBA colorsRight1 = colorSensorRight1.getNormalizedColors();
        NormalizedRGBA colorsRight2 = colorSensorRight2.getNormalizedColors();

        double distLeft1 = distanceSensorLeft1.getDistance(DistanceUnit.CM);
        double distLeft2 = distanceSensorLeft2.getDistance(DistanceUnit.CM);
        double distCenter = distanceSensorCenter.getDistance(DistanceUnit.CM);  // NEW
        double distRight1 = distanceSensorRight1.getDistance(DistanceUnit.CM);
        double distRight2 = distanceSensorRight2.getDistance(DistanceUnit.CM);

        // Display LEFT SLOT sensors
        telemetry.addLine("=== LEFT SLOT ===");
        telemetry.addLine();

        telemetry.addLine("Sensor Left1:");
        telemetry.addData("  Distance (cm)", "%.1f", distLeft1);
        telemetry.addData("  Red", "%.3f", colorsLeft1.red);
        telemetry.addData("  Green", "%.3f", colorsLeft1.green);
        telemetry.addData("  Blue", "%.3f", colorsLeft1.blue);
        telemetry.addData("  Alpha", "%.3f", colorsLeft1.alpha);
        addColorSuggestion(colorsLeft1);
        telemetry.addLine();

        telemetry.addLine("Sensor Left2:");
        telemetry.addData("  Distance (cm)", "%.1f", distLeft2);
        telemetry.addData("  Red", "%.3f", colorsLeft2.red);
        telemetry.addData("  Green", "%.3f", colorsLeft2.green);
        telemetry.addData("  Blue", "%.3f", colorsLeft2.blue);
        telemetry.addData("  Alpha", "%.3f", colorsLeft2.alpha);
        addColorSuggestion(colorsLeft2);
        telemetry.addLine();

        // Display CENTER SLOT sensor (NEW)
        telemetry.addLine("=== CENTER SLOT ===");
        telemetry.addLine();

        telemetry.addLine("Sensor Center:");
        telemetry.addData("  Distance (cm)", "%.1f", distCenter);
        telemetry.addData("  Red", "%.3f", colorsCenter.red);
        telemetry.addData("  Green", "%.3f", colorsCenter.green);
        telemetry.addData("  Blue", "%.3f", colorsCenter.blue);
        telemetry.addData("  Alpha", "%.3f", colorsCenter.alpha);
        addColorSuggestion(colorsCenter);
        telemetry.addLine();

        // Display RIGHT SLOT sensors
        telemetry.addLine("=== RIGHT SLOT ===");
        telemetry.addLine();

        telemetry.addLine("Sensor Right1:");
        telemetry.addData("  Distance (cm)", "%.1f", distRight1);
        telemetry.addData("  Red", "%.3f", colorsRight1.red);
        telemetry.addData("  Green", "%.3f", colorsRight1.green);
        telemetry.addData("  Blue", "%.3f", colorsRight1.blue);
        telemetry.addData("  Alpha", "%.3f", colorsRight1.alpha);
        addColorSuggestion(colorsRight1);
        telemetry.addLine();

        telemetry.addLine("Sensor Right2:");
        telemetry.addData("  Distance (cm)", "%.1f", distRight2);
        telemetry.addData("  Red", "%.3f", colorsRight2.red);
        telemetry.addData("  Green", "%.3f", colorsRight2.green);
        telemetry.addData("  Blue", "%.3f", colorsRight2.blue);
        telemetry.addData("  Alpha", "%.3f", colorsRight2.alpha);
        addColorSuggestion(colorsRight2);
    }

    /**
     * Helper method that uses the CURRENT (simple) color detection logic
     * to show you what the sensor would classify the current reading as.
     * This helps you verify if your logic is working correctly.
     */
    private void addColorSuggestion(NormalizedRGBA colors) {
        String suggestion;
        if (colors.green > colors.red && colors.green > colors.blue) {
            suggestion = "→ Looks GREEN";
        } else if (colors.blue > colors.red && colors.blue > colors.green) {
            suggestion = "→ Looks PURPLE";
        } else {
            suggestion = "→ UNKNOWN/Ambiguous";
        }
        telemetry.addData("  Current Logic Says", suggestion);
    }
}