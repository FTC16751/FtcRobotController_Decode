package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import android.util.Log;
import androidx.annotation.NonNull;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private final Telemetry telemetry;

    // --- State variables to hold the latest processed data ---
    private boolean isTargetVisible = false;
    private int lastTagId = -1;
    private double lastTagDistanceMeters = -1.0;
    private double lastTx = 0.0; // Horizontal angle
    private LLResult lastValidResult = null;


    public enum MotifPattern {
        GPP21, PGP22, PPG23, UNKNOWN
    }
    double x_meters ;
    double z_meters ;
    double y_meters;
    // --- MegaTag2 field pose state ---
    private boolean hasFieldPose = false;
    private double fieldX_m = 0.0;
    private double fieldY_m = 0.0;
    private double  fieldZ_m = 0.0;
    private double fieldHeadingRad = 0.0;   // yaw
    private double x_meters_rpfs;
    private double z_meters_rpfs;
    private double y_meters_rpfs;
    private double yaw_rpfs;
    private double lastTagDistanceMeters_rpfs;
    private static double normalizeRad(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
    public static final double BLUE_TAG20_X_M = -1.482;
    public static final double BLUE_TAG20_Y_M = -1.413;

    public static final double RED_TAG24_X_M  = -1.482;
    public static final double RED_TAG24_Y_M  =  1.413;
    Pose3D botposeMT2;
    private Pose3D robotPoseInFieldSpace = null;
    private static final int MOTIF_PIPELINE = 0;
    private static final int RED_AIM_PIPELINE = 1;
    private static final int BLUE_AIM_PIPELINE = 2;
    /**
     * Initializes the Limelight 3A vision system.
     * @param hardwareMap The HardwareMap from the OpMode, used to find the Limelight.
     * @param telemetry The Telemetry object used for logging.
     */
    public VisionUtil(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetry.addData("Limelight", "Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("Limelight ERROR", "NOT found in config. Check name.");
            this.limelight = null; // Ensure limelight is null if initialization fails
        }
    }

    /**
     * This method MUST be called in every loop of your OpMode.
     * It fetches the latest result from the Limelight, making it available
     * for all other methods in this class.
     */
    public void update() {

        // Calculate horizontal distance from the robot's pose in the tag's reference frame.
        telemetry.addData("LL PoseTag (x,y,z m)", "%.2f, %.2f, %.2f", x_meters, y_meters, z_meters);
        //telemetry.addData("Debug lastTx", lastTx);
        if (limelight == null) {
            resetTracking();
            return;
        }

        LLResult currentResult = limelight.getLatestResult();

        // 1. Check for a valid result packet from the Limelight
        if (currentResult == null || !currentResult.isValid()) {
            resetTracking();
            return;
        }

        // We have a valid packet, so store it.
        this.lastValidResult = currentResult;
        this.lastTx = currentResult.getTx();

        // 2. Check if the valid packet contains any AprilTags
        List<LLResultTypes.FiducialResult> tags = currentResult.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            resetTracking();
            this.robotPoseInFieldSpace = null;
            return;
        }

        // 3. If we reach here, we have a visible target. Process it.
        this.isTargetVisible = true;

        // For now, use the first detected tag as the primary target.
        // In the future, you could add logic here to find a specific ID.
        LLResultTypes.FiducialResult primaryTag = tags.get(0);
        this.lastTagId = primaryTag.getFiducialId();

        // Calculate horizontal distance from the robot's pose in the tag's reference frame.
        Pose3D robotPoseInTagSpace = primaryTag.getRobotPoseTargetSpace();
        x_meters = robotPoseInTagSpace.getPosition().x; // Side-to-side distance from tag center
        z_meters = robotPoseInTagSpace.getPosition().z; // Forward/backward distance from tag
        y_meters = robotPoseInTagSpace.getPosition().y;
        this.lastTagDistanceMeters = Math.hypot(x_meters, z_meters);

        // --- Robot Post Field Space Data ---
        // Calculate horizontal distance from the robot's pose in the tag's reference frame.
        this.robotPoseInFieldSpace = primaryTag.getRobotPoseFieldSpace();
        x_meters_rpfs = this.robotPoseInFieldSpace.getPosition().x; // Side-to-side distance from tag center (rpfs = robot pose field space)
        z_meters_rpfs = this.robotPoseInFieldSpace.getPosition().z;
        y_meters_rpfs = this.robotPoseInFieldSpace.getPosition().y;
        yaw_rpfs = this.robotPoseInFieldSpace.getOrientation().getYaw();
        this.lastTagDistanceMeters_rpfs = Math.hypot(x_meters_rpfs, z_meters_rpfs);
        // --- Robot Post Field Space Data ---

        // --- FIELD POSE FROM MEGATAG2 ---
        botposeMT2 = currentResult.getBotpose_MT2();
        if (botposeMT2 != null) {
            fieldX_m = botposeMT2.getPosition().x;                 // meters
            fieldY_m = botposeMT2.getPosition().y;                 // meters
            fieldZ_m = botposeMT2.getPosition().z;
            //fieldHeadingRad = botposeMT2.getOrientation().getYaw(); // radians
            hasFieldPose = true;
        } else {
            hasFieldPose = false;
        }

    }

    public void updateRobotOrientation(double imuHeadingDegrees) {
        this.fieldHeadingRad = Math.toRadians(imuHeadingDegrees);
        if (limelight != null) {
            limelight.updateRobotOrientation(imuHeadingDegrees);
        }
    }

    /**
     * Sets the vision system's targeting mode based on the alliance color.
     * This is the primary method OpModes should use to configure vision targeting.
     * @param alliance The alliance color to target.
     */
    public void setTargetingAlliance(GGRobotConstants.Alliance alliance) {
        if (alliance == GGRobotConstants.Alliance.RED) {
            setPipeline(RED_AIM_PIPELINE);
        } else {
            setPipeline(BLUE_AIM_PIPELINE);
        }
    }

    /**
     * Sets the vision system to the generic motif detection mode.
     * This should be used during the autonomous init_loop.
     */
    public void setMotifDetectionMode() {
        setPipeline(MOTIF_PIPELINE);
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
        if (!isTargetVisible) {
            return MotifPattern.UNKNOWN;
        }
        // The getFiducialDetections() method is no longer needed, as we can just check lastTagId
        switch (lastTagId) {
            case 21: return MotifPattern.GPP21;
            case 22: return MotifPattern.PGP22;
            case 23: return MotifPattern.PPG23;
            default: return MotifPattern.UNKNOWN;
        }
    }


    // =================================================================================
    // PUBLIC "GETTER" METHODS - The clean API for your OpModes
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

    public Pose3D getBotPose_MT2() {
        return botposeMT2;
    }

    public Pose3D getRobotPoseFieldSpace() {
        return this.robotPoseInFieldSpace;
    }

    /**
     * Gets the ID of the primary detected AprilTag.
     * @return The tag ID, or -1 if no target is visible.
     */
    public int getDetectedTagId() {
        return lastTagId;
    }

    public double getDistanceToTagMeters() {
        return lastTagDistanceMeters;
    }

    /**
     * Gets the calculated horizontal distance to the primary tag.
     * @return The distance in INCHES, or a negative value if no target is visible.
     */
    public double getDistanceToTagInches() {
        if (!isTargetVisible) {
            return -1.0;
        }
        return lastTagDistanceMeters * 39.3701;
    }

    /**
     * Gets the horizontal angle (tx) to the target. Useful for auto-aiming.
     * @return The angle in degrees. Positive is right, negative is left.
     */
    public double getTargetAngleX() {
        return lastTx;
    }

    private void resetTracking() {
        this.isTargetVisible = false;
        this.lastTagId = -1;
        this.lastTagDistanceMeters = -1.0;
        this.lastTx = 0.0;
        // Do NOT set lastValidResult to null here, so we can still access stale data if needed.
    }
    /**
     * Checks if a valid AprilTag was visible during the last update cycle.
     * @return true if a target is currently visible, false otherwise.
     */
    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    /**
     * Adds relevant vision data to the telemetry stream for debugging.
     */
    public void addTelemetry() {
        telemetry.addLine("--- Limelight Vision ---");
        if (isTargetVisible) {
            telemetry.addData("LL Status", "Target Visible");
            telemetry.addData("LL Tag ID", lastTagId);
            telemetry.addData("LL PoseTag X ", x_meters*39.3701);
            telemetry.addData("LL PoseTag Y ", y_meters*39.3701);
            telemetry.addData("LL PoseTag Z ", z_meters*39.3701);
            telemetry.addData("LL Distance (in)", "%.1f", getDistanceToTagInches());
            telemetry.addData("LL Angle (tx)", "%.2f", lastTx);
        } else {
            telemetry.addData("LL Status", "No Targets Visible");
        }
        telemetry.addData("Heading Error to red Tag: ", getHeadingErrorToRedTag24Deg());
        telemetry.addLine("--- Limelight Vision MT2 Data---");
        telemetry.addData("fieldX_m ", fieldX_m);
        telemetry.addData("fieldY_m ", fieldY_m);
        telemetry.addData("fieldZ_m ", fieldZ_m);
        telemetry.addData("fieldHeading ",Math.toDegrees(fieldHeadingRad));   // yaw

        telemetry.addLine("--- Limelight Vision RPFS Data---");
        telemetry.addData("x_meters_rpfs (in) ", x_meters_rpfs*39.3701);
        telemetry.addData("z_meters_rpfs (in) ", z_meters_rpfs*39.3701);
        telemetry.addData("y_meters_rpfs (in) ", y_meters_rpfs*39.3701);
        telemetry.addData("yaw_rpfs ", yaw_rpfs);
        telemetry.addData("lastTagDistanceMeters_rpfs ", lastTagDistanceMeters_rpfs*39.3701);


    }
    public boolean hasFieldPose() {
        return hasFieldPose;
    }

    // Heading error (radians) from robot's current heading to a field-space target (goalX_m, goalY_m)
    public double getHeadingErrorToFieldPointRad(double goalX_m, double goalY_m) {
        if (!hasFieldPose) return 0.0;

        double dx = goalX_m - fieldX_m;
        double dy = goalY_m - fieldY_m;

        double desiredHeadingRad = Math.atan2(dy, dx);
        double errorRad = normalizeRad(desiredHeadingRad - this.fieldHeadingRad);

        return errorRad;
    }

    public double getHeadingErrorToFieldPointDeg(double goalX_m, double goalY_m) {
        return Math.toDegrees(getHeadingErrorToFieldPointRad(goalX_m, goalY_m));
    }
    // Convenience wrappers
    public double getHeadingErrorToRedTag24Deg() {
        return getHeadingErrorToFieldPointDeg(RED_TAG24_X_M, RED_TAG24_Y_M);
    }

    public double getHeadingErrorToBlueTag20Deg() {
        return getHeadingErrorToFieldPointDeg(BLUE_TAG20_X_M, BLUE_TAG20_Y_M);
    }


}
