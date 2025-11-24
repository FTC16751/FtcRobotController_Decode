package org.firstinspires.ftc.teamcode.utilities.Common;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * P3_VisionUtil encapsulates all logic for the Limelight vision camera.
 * It handles initialization, polling for data, and provides simple methods
 * to get results like distance and angle to a target.
 */
public class LimeLightVisionUtil {
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private double lastTagDistanceMeters = -1.0;
    private int lastTagId = -1;
    private double lastTx = 0.0; // Horizontal angle
    private boolean isTargetVisible = false;

    public LimeLightVisionUtil(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetry.addData("Limelight", "Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("Limelight ERROR", "NOT found in config. Check name.");
            throw new RuntimeException("Limelight not found. Check hardware configuration.", e);
        }
    }

    /**
     * This method MUST be called in every loop of an OpMode to get the latest data.
     */
    public void update() {
        if (limelight == null) {
            isTargetVisible = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            resetTracking();
            isTargetVisible = false;
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            isTargetVisible = false;
            resetTracking();
            return;
        }
        isTargetVisible = true;
        // For now, use the first detected tag.
        LLResultTypes.FiducialResult tag = tags.get(0);

        // Extract and store data
        Pose3D robotPoseTagSpace = tag.getRobotPoseTargetSpace();
        double x = robotPoseTagSpace.getPosition().x; // right/left of tag (m)
        double z = robotPoseTagSpace.getPosition().z; // out from tag forward/back (m)

        this.lastTagId = tag.getFiducialId();
        this.lastTagDistanceMeters = Math.hypot(x, z);
        this.lastTx = result.getTx();
        addTelemetry();
    }

    /**
     * Stops the Limelight polling thread. Call this in the OpMode's stop() method.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Adds relevant vision data to the telemetry stream for debugging.
     */
    public void addTelemetry() {
        if (lastTagId != -1) {
            telemetry.addData("LL Tag ID", lastTagId);
            telemetry.addData("LL Dist (in)", "%.1f", lastTagDistanceMeters * 39.3701);
        } else {
            telemetry.addData("LL Status", "No tags visible");
        }
    }

    // --- Public "Getter" Methods ---
    public int getDetectedTagId() {
        return lastTagId;
    }

    public double getDistanceToTagMeters() {
        return lastTagDistanceMeters;
    }
public double getTagAngle() {
        return lastTx;
}

    private void resetTracking() {
        this.lastTagId = -1;
        this.lastTagDistanceMeters = -1.0;
    }
    public boolean isTargetVisible() {
        return isTargetVisible;
    }
}
