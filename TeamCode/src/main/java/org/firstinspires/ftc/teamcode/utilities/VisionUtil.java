package org.firstinspires.ftc.teamcode.utilities;

import android.util.Log;
import androidx.annotation.NonNull;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Collections;
import java.util.List;

/**
 * A utility class for managing a Limelight 3A vision sensor.
 * This class encapsulates the initialization and usage of the Limelight,
 * providing a simple and reusable interface for both Autonomous and TeleOp modes.
 *
 * <p><b>Integration:</b>
 * 1. Create an instance of this class in your OpMode's init() method.
 * 2. In every loop (init_loop and loop), you MUST call the {@link #update()} method.
 * 3. Use the provided high-level methods like {@link #getMotifPattern()} or {@linkDetectionByClass}
 *    to get processed vision data.
 * 4. At the end of your OpMode, call the {@link #stop()} method to release resources.
 * </p>
 */
public class VisionUtil {

    private Limelight3A limelight;
    private LLResult lastValidResult;

    /**
     * An enum to represent the detected spike mark position based on AprilTag IDs.
     * This makes the code in the OpMode much more readable.
     */
    public enum MotifPattern {
        GPP21,
        PGP22,
        PPG23,
        UNKNOWN // A default value if no valid tag is seen
    }

    /**
     * Initializes the Limelight 3A vision system.
     * @param hardwareMap The HardwareMap from the OpMode, used to find the Limelight.
     * @param limelightName The name of the Limelight as configured in the robot's configuration file.
     */
    public VisionUtil(@NonNull HardwareMap hardwareMap, @NonNull String limelightName) {
        // Safely initialize the hardware to prevent crashes if it's not configured.
        try {
            limelight = hardwareMap.get(Limelight3A.class, limelightName);
            limelight.pipelineSwitch(0); // Default to pipeline 0
            limelight.start(); // Start polling for data immediately
            Log.i("VisionUtil", "Limelight '" + limelightName + "' initialized successfully.");
        } catch (Exception e) {
            Log.e("VisionUtil", "Could not initialize Limelight '" + limelightName + "'. Check configuration. Error: " + e.getMessage());
            limelight = null; // Ensure limelight is null if initialization fails
        }
    }

    /**
     * This method MUST be called in every loop of your OpMode.
     * It fetches the latest result from the Limelight, making it available
     * for all other methods in this class.
     */
    public void update() {
        if (limelight == null) {
            return;
        }

        LLResult currentResult = limelight.getLatestResult();

        // If we don't have a result yet, get the first valid one.
        if (lastValidResult == null) {
            if (currentResult.isValid()) {
                lastValidResult = currentResult;
            }
            return;
        }

        // After the first assignment, only update if the new result is valid AND newer.
        if (currentResult.isValid() && currentResult.getTimestamp() != lastValidResult.getTimestamp()) {
            lastValidResult = currentResult;
        }
    }

    // =================================================================================
    // GAME-SPECIFIC HIGH-LEVEL METHODS
    // =================================================================================

    /**
     * Analyzes the visible AprilTags to determine the spike mark position for the current FTC game.
     * This method specifically looks for game-specific tags (21 for LEFT, 22 for CENTER, 23 for RIGHT).
     *
     * @return The detected {@link MotifPattern}, or UNKNOWN if no relevant tag is visible.
     */
    public MotifPattern getMotifPattern() {
        List<LLResultTypes.FiducialResult> detections = getFiducialDetections();

        if (detections.isEmpty()) {
            return MotifPattern.UNKNOWN;
        }

        // Iterate through the detected tags to find the one we care about.
        for (LLResultTypes.FiducialResult detection : detections) {
            int tagId = detection.getFiducialId();
            switch (tagId) {
                case 21:
                    return MotifPattern.GPP21;
                case 22:
                    return MotifPattern.PGP22;
                case 23:
                    return MotifPattern.PPG23;
            }
        }

        // If we get here, it means we saw tags, but none of them were 21, 22, or 23.
        return MotifPattern.UNKNOWN;
    }


    // =================================================================================
    // GENERAL-PURPOSE LOW-LEVEL METHODS
    // =================================================================================

    /**
     * Gets the list of all currently detected AprilTags (fiducials).
     *
     * @return A list of {@link LLResultTypes.FiducialResult} objects. The list will be empty if none are detected.
     */
    public List<LLResultTypes.FiducialResult> getFiducialDetections() {
        if (lastValidResult != null && lastValidResult.isValid()) {
            return lastValidResult.getFiducialResults();
        }
        return Collections.emptyList();
    }

    /**
     * Switches the Limelight's active pipeline.
     * @param pipelineIndex The index of the pipeline to switch to (0-9).
     */
    public void setPipeline(int pipelineIndex) {
        if (limelight == null) return;

        if (pipelineIndex >= 0 && pipelineIndex <= 9) {
            limelight.pipelineSwitch(pipelineIndex);
        } else {
            Log.w("VisionUtil", "Invalid pipeline index: " + pipelineIndex + ". Must be between 0 and 9.");
        }
    }

    /**
     * Stops the Limelight polling and releases resources.
     * This MUST be called at the end of your OpMode.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
            Log.i("VisionUtil", "Limelight has been stopped.");
        }
    }

    /* Finds a specific AprilTag by its ID from the list of current detections.

     */
    public LLResultTypes.FiducialResult getFiducialById(int tagId) {
        for (LLResultTypes.FiducialResult detection : getFiducialDetections()) {
            if (detection.getFiducialId() == tagId) {
                return detection;
            }
        }
        return null; // Return null if not found
    }
    /**
     * Gets the robot's calculated 3D position from the Limelight.
     * This is the "botpose" relative to the AprilTag field layout.
     *
     * @return The robot's {@link Pose3D} if available, otherwise null.
     */
    public Pose3D getBotPose() {
        if (lastValidResult != null && lastValidResult.isValid()) {
            return lastValidResult.getBotpose();
        }
        return null;
    }
}
