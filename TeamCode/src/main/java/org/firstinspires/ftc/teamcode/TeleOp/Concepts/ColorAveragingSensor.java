/*
 * This OpMode demonstrates a robust method for detecting the color of a game element
 * that may have an uneven surface (like holes). It uses a state machine and
 * averages multiple color sensor readings over a short period to get a stable result.
 *
 * It assumes a color sensor configured as "sensor_color" that also supports
 * distance sensing.
 */
package org.firstinspires.ftc.teamcode.TeleOp.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: Color Averaging", group = "Sensor")
@Disabled
public class ColorAveragingSensor extends LinearOpMode {

    // Define the states for our detection process
    enum DetectionState {
        WAITING_FOR_ELEMENT, // State 1: Looking for a ball to enter the sensor's range
        SAMPLING_COLOR,      // State 2: Ball is present, collect color data
        DETERMINED,          // State 3: Analysis is done, display the result
        WAITING_FOR_REMOVAL  // State 4: Wait for the ball to be removed before restarting
    }

    private DetectionState currentState = DetectionState.WAITING_FOR_ELEMENT;

    // The sensor object
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;

    // Timer for the sampling period
    ElapsedTime samplingTimer = new ElapsedTime();

    // Constants for our logic (these should be tuned for your robot and lighting)
    final double DISTANCE_THRESHOLD_CM = 4.0; // How close the ball must be to start sampling
    final double SAMPLING_DURATION_MSEC = 500; // How long to collect samples (in milliseconds)
    final int REQUIRED_SAMPLES = 20; // How many samples to try and get in that time

    // Variables to store the running total of color values
    double totalRed = 0;
    double totalGreen = 0;
    double totalBlue = 0;
    int sampleCount = 0;

    // Enum to store the final color result
    enum DeterminedColor {
        RED,
        BLUE,
        GREEN,
        UNKNOWN
    }
    DeterminedColor determinedColor = DeterminedColor.UNKNOWN;


    @Override
    public void runOpMode() {
        // --- Initialization ---
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_L2");
            // Most color sensors with distance are also castable to DistanceSensor
            distanceSensor = (DistanceSensor) colorSensor;
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find 'sensor_color'. Check configuration.");
            telemetry.update();
            sleep(5000);
            return;
        }

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.addData(">", "Point sensor at game element to begin.");
        telemetry.update();

        waitForStart();

        // --- Main Loop ---
        while (opModeIsActive()) {
            // Always get the latest sensor data at the top of the loop
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            colorSensor.setGain(20);
            // The state machine logic
            switch (currentState) {
                case WAITING_FOR_ELEMENT:
                    telemetry.addData("State", "Waiting for Element");
                    telemetry.addData("Distance (cm)", "%.2f", distance);

                    // If a game element is close enough, start the sampling process
                    if (distance < DISTANCE_THRESHOLD_CM) {
                        // Reset all our averaging variables
                        totalRed = 0;
                        totalGreen = 0;
                        totalBlue = 0;
                        sampleCount = 0;
                        samplingTimer.reset(); // Start the timer for the sampling duration
                        currentState = DetectionState.SAMPLING_COLOR;
                    }
                    break;

                case SAMPLING_COLOR:
                    telemetry.addData("State", "Sampling Color...");
                    telemetry.addData("Time Elapsed (ms)", "%.0f", samplingTimer.milliseconds());
                    telemetry.addData("Samples Collected", sampleCount);

                    // Collect samples as long as we are within the sampling duration
                    if (samplingTimer.milliseconds() < SAMPLING_DURATION_MSEC) {
                        totalRed += colors.red;
                        totalGreen += colors.green;
                        totalBlue += colors.blue;
                        sampleCount++;
                    } else {
                        // The sampling time is over. Now, analyze the result.
                        determineColor();
                        currentState = DetectionState.DETERMINED;
                    }
                    break;

                case DETERMINED:
                    telemetry.addData("State", "Color Determined!");
                    telemetry.addData("FINAL COLOR", determinedColor);
                    telemetry.addLine("Waiting 2 seconds before checking for removal...");

                    // Display the result for a moment before moving on
                    if (samplingTimer.seconds() > 2.0) {
                        currentState = DetectionState.WAITING_FOR_REMOVAL;
                    }
                    break;

                case WAITING_FOR_REMOVAL:
                    telemetry.addData("State", "Waiting for Element Removal");
                    telemetry.addData("Distance (cm)", "%.2f", distance);

                    // If the game element is moved away, reset the state machine
                    if (distance > DISTANCE_THRESHOLD_CM) {
                        telemetry.addLine("Element removed. Ready for next one.");
                        currentState = DetectionState.WAITING_FOR_ELEMENT;
                    }
                    break;
            }

            telemetry.update();
        }
    }

    /**
     * This method is called once the sampling period is over. It calculates the
     * average color and makes a final decision.
     */
    void determineColor() {
        if (sampleCount == 0) {
            determinedColor = DeterminedColor.UNKNOWN;
            return;
        }

        // Calculate the average of each color component
        double avgRed = totalRed / sampleCount;
        double avgGreen = totalGreen / sampleCount;
        double avgBlue = totalBlue / sampleCount;

        telemetry.addData("Avg Red", "%.3f", avgRed);
        telemetry.addData("Avg Green", "%.3f", avgGreen);
        telemetry.addData("Avg Blue", "%.3f", avgBlue);

        // --- Simple Color Logic ---
        // This is the part you will need to tune based on your lighting conditions.
        // Look at the telemetry values for your red and blue game elements and adjust these thresholds.
        // A common method is to check which color component is strongest.
        if (avgGreen > avgBlue && avgGreen > avgRed) {
            determinedColor = DeterminedColor.GREEN;
        } else if (avgBlue > avgRed && avgBlue > avgGreen) {
            determinedColor = DeterminedColor.BLUE;
        } else {
            determinedColor = DeterminedColor.UNKNOWN;
        }
    }
}
