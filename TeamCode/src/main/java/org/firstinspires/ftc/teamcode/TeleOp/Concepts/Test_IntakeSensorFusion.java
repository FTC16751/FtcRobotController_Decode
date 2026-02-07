package org.firstinspires.ftc.teamcode.TeleOp.Concepts;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.IntakeSensorFusion001;

import java.util.List;

/**
 * Test_IntakeSensorFusion is a simple TeleOp OpMode designed to verify the
 * functionality of the high-level IntakeSensorFusion001 utility class.
 *
 * PURPOSE:
 * 1.  Confirm that the IntakeSensorFusion001 class initializes correctly without errors.
 * 2.  Verify that the internal state machines for each sensor are running by calling the main update() method.
 * 3.  Display the clean, fused telemetry output from the utility class, showing occupancy and determined color for each slot.
 * 4.  Test the high-level API methods like getInventory() and getColorOfSlot().
 *
 * USAGE:
 * - Run this OpMode.
 * - The Driver Station will display a summary of the intake's inventory.
 * - Slowly feed artifacts into your intake system one by one.
 * - As an artifact enters a sensor's range, you should see its "Occupied" status change to 'true'
 *   and its "Color" change from 'UNKNOWN' to 'SAMPLING' and then to 'PURPLE' or 'GREEN'.
 * - As the artifact leaves the sensor, its status should return to "Occupied: false, Color: UNKNOWN".
 *
 * @author Your Team Name
 */
@TeleOp(name = "Test: Intake Sensor FUSION", group = "Test")

public class Test_IntakeSensorFusion extends OpMode {

    // The single instance of our sensor fusion utility class.
    private IntakeSensorFusion001 intakeSensors;

    /**
     * Code to run ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        telemetry.addLine("Initializing IntakeSensorFusion Utility...");
        telemetry.update();

        try {
            // Create an instance of the class we want to test.
            // The constructor will handle finding and setting up all four sensors.
            intakeSensors = new IntakeSensorFusion001(hardwareMap, telemetry);
            telemetry.addLine("IntakeSensorFusion Initialized Successfully!");
        } catch (Exception e) {
            // If any sensor is misconfigured, the constructor will throw an error.
            telemetry.addData("FATAL ERROR", "Could not initialize IntakeSensorFusion!");
            telemetry.addData("Reason", e.getMessage());
        }

        telemetry.addLine("Ready for Start.");
        telemetry.update();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START.
     */
    @Override
    public void loop() {
        // If initialization failed, don't do anything else.
        if (intakeSensors == null) {
            telemetry.addData("ERROR", "IntakeSensorFusion is not available. Check configuration.");
            return;
        }

        // CRITICAL: You must call the update() method of the fusion class in every
        // loop cycle. This is what drives the internal state machines for all four sensors.
        intakeSensors.update();

        // --- Display the Fused Telemetry ---
        // Instead of printing raw data, we now call the clean telemetry method
        // from our utility class. This directly tests the output of your fusion logic.
        intakeSensors.addTelemetry();

        // --- Test High-Level API Methods ---
        // We can add extra telemetry to test the other public methods of our class.
        telemetry.addLine();
        telemetry.addLine("--- API Test Results ---");

        // Test the getInventory() method
        List<IntakeSensorFusion001.ArtifactColor> inventory = intakeSensors.getInventory();
        telemetry.addData("getInventory() size", inventory.size());
        telemetry.addData("Inventory Contents", inventory.toString());

        // Test the isSlotOccupied() method for each slot
        boolean leftOccupied = intakeSensors.isSlotOccupied(IntakeSensorFusion001.IntakeSlot.LEFT);
        boolean rightOccupied = intakeSensors.isSlotOccupied(IntakeSensorFusion001.IntakeSlot.RIGHT);
        telemetry.addData("isSlotOccupied(LEFT)", leftOccupied);
        telemetry.addData("isSlotOccupied(RIGHT)", rightOccupied);

        // Test the getColorOfSlot() method for each slot
        IntakeSensorFusion001.ArtifactColor leftColor = intakeSensors.getColorOfSlot(IntakeSensorFusion001.IntakeSlot.LEFT);
        IntakeSensorFusion001.ArtifactColor rightColor = intakeSensors.getColorOfSlot(IntakeSensorFusion001.IntakeSlot.RIGHT);
        telemetry.addData("getColorOfSlot(LEFT)", leftColor);
        telemetry.addData("getColorOfSlot(RIGHT)", rightColor);  // FIXED: Was showing "LEFT_2"
    }
}