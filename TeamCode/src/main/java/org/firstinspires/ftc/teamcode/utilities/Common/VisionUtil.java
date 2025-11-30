package org.firstinspires.ftc.teamcode.utilities.Common;

import android.util.Log;
import androidx.annotation.NonNull;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;

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
    private double robotDistanceToTagInTagSpace = -1.0;
    private double lastTx = 0.0; // Horizontal angle
    private LLResult lastValidResult = null;
    private static final double METERS_TO_INCHES = 39.3701;

    // --- Pose Data ---
    private Pose3D robotPoseInTagSpace;    // Robot's pose relative to the primary tag
    private Pose3D robotPoseInFieldSpace;  // Robot's pose on the field (from a single tag)
    private Pose3D botposeMT2;             // Robot's pose on the field (from MegaTag2)

    // --- MegaTag2 field pose state ---
    private boolean hasMegaTag2FieldPose = false;
    private double fieldHeadingRad = 0.0;   // yaw

    private static double normalizeRad(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    public enum MotifPattern {
        GPP21, PGP22, PPG23, UNKNOWN
    }
    public static final double BLUE_GOAL_X_COORDINATE_METERS = -1.482;
    public static final double BLUE_GOAL_Y_COORDINATE_METERS = -1.413;

    public static final double RED_GOAL_X_COORDINATE_METERS = -1.482;
    public static final double RED_GOAL_Y_COORDINATE_METERS =  1.413;

    private static final int MOTIF_PIPELINE = 0;
    private static final int RED_GOAL_PIPELINE = 1;
    private static final int BLUE_GOAL_PIPELINE = 2;





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

        if (limelight == null) {
            resetTracking();
            return;
        }

        LLResult currentResult = limelight.getLatestResult();
        if (currentResult == null || !currentResult.isValid()) {
            resetTracking();
            return;
        }

        //Check if the valid packet contains any AprilTags
        List<LLResultTypes.FiducialResult> tags = currentResult.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            resetTracking();
            return;
        }

        //If we reach here, we have a visible target. Process it.
        isTargetVisible = true;
        lastValidResult = currentResult;
        lastTx = currentResult.getTx();

        // For now, use the first detected tag as the primary target.
        // In the future, add logic here to find a specific ID.
        LLResultTypes.FiducialResult primaryTag = tags.get(0);
        lastTagId = primaryTag.getFiducialId();

        //i actually don't know which data is best to use, so i'm fetching more than what's needed
        //and sending that to telemetry for analysis
        processTagSpacePose(primaryTag);
        processFieldSpacePose(primaryTag);
        processMegaTagPose(currentResult);

    }


    // =================================================================================
    // PROCESS TAG DATA METHODS
    // =================================================================================
    private void processTagSpacePose(LLResultTypes.FiducialResult tag) {
        robotPoseInTagSpace = tag.getRobotPoseTargetSpace();
    }

    private void processFieldSpacePose(LLResultTypes.FiducialResult tag) {
        robotPoseInFieldSpace = tag.getRobotPoseFieldSpace();
    }

    private void processMegaTagPose(LLResult currentResult) {
        botposeMT2 = currentResult.getBotpose_MT2();
        if (botposeMT2 != null) {
            hasMegaTag2FieldPose = true;
        } else {
            hasMegaTag2FieldPose = false;
        }
    }

    // =================================================================================
    // PUBLIC "GETTER" METHODS
    // =================================================================================
    public Pose3D getRobotPoseInTagSpace() {
        return robotPoseInTagSpace;
    }

    public Pose3D getBotPose_MT2() {
        return botposeMT2;
    }

    public Pose3D getRobotPoseFieldSpace() {
        return this.robotPoseInFieldSpace;
    }
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

    /**
     * Gets the list of all currently detected AprilTags (fiducials).
     *
     * The list will be empty if none are detected.
     */
    public List<LLResultTypes.FiducialResult> getFiducialDetections() {
        if (lastValidResult != null && lastValidResult.isValid()) {
            return lastValidResult.getFiducialResults();
        }
        return Collections.emptyList();
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



    /**
     * Gets the ID of the primary detected AprilTag.
     * @return The tag ID, or -1 if no target is visible.
     */
    public int getDetectedTagId() {

        return lastTagId;
    }

    public double getDistanceToTagMeters() {

        return robotDistanceToTagInTagSpace;
    }

    /**
     * Gets the calculated horizontal distance to the primary tag.
     * @return The distance in INCHES, or a negative value if no target is visible.
     */
    public double getDistanceToTagInches() {
        if (!isTargetVisible) {
            return -1.0;
        }
        // Create a local variable for the pose to work with.
        Pose3D pose = this.robotPoseInTagSpace;
        if (pose == null) {
            return -1.0;
        }
        double x_meters = pose.getPosition().x;
        double z_meters = pose.getPosition().z; // Limelight uses Z for forward distance in tag space
        return Math.hypot(x_meters, z_meters) * METERS_TO_INCHES;
    }

    /**
     * Gets the horizontal angle (tx) to the target. Useful for auto-aiming.
     * @return The angle in degrees. Positive is right, negative is left.
     */
    public double getTargetAngleX() {
        return lastTx;
    }


    /**
     * Calculates the heading error from the robot's current heading to a field-space target.
     * WARNING: This uses the MT2 reading from the apriltag, meaning the robot must have the tag visible
     * for this to work.
     *
     * goalX_m The x-coordinate of the target in meters.
     * fieldY_coordinate_meters The y-coordinate of the target in meters.
     * unit    The desired unit for the returned angle (RADIANS or DEGREES).
     * @return The heading error in the specified unit, or 0.0 if the pose is not available.
     *
     */
    public double calculateHeadingErrorToAFieldPoint(double fieldX_coordinate_meters, double fieldY_coordinate_meters, AngleUnit unit) {
        if (!hasMegaTag2FieldPose) {
            return 0.0;
        }

        Pose3D robotPoseMT2 = getBotPose_MT2();
        if (robotPoseMT2 == null) {
            return 0.0;
        }


        double dx = fieldX_coordinate_meters - robotPoseMT2.getPosition().x;
        double dy = fieldY_coordinate_meters - robotPoseMT2.getPosition().y;

        double desiredHeadingRad = Math.atan2(dy, dx);
        double headingError = normalizeRad(desiredHeadingRad - this.fieldHeadingRad);

        if (unit == AngleUnit.DEGREES) {
            return Math.toDegrees(headingError);
        }

        return headingError;
    }


    // Convenience wrappers
    public double calculateHeadingErrorToRedGoalDegrees() {
        return calculateHeadingErrorToAFieldPoint(RED_GOAL_X_COORDINATE_METERS, RED_GOAL_Y_COORDINATE_METERS, AngleUnit.DEGREES);
    }
    public double calculateHeadingErrorToBlueGoalDegrees() {
        return calculateHeadingErrorToAFieldPoint(BLUE_GOAL_X_COORDINATE_METERS, BLUE_GOAL_Y_COORDINATE_METERS, AngleUnit.DEGREES);
    }

    /**
     * Checks if a valid AprilTag was visible during the last update cycle.
     * @return true if a target is currently visible, false otherwise.
     */
    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    public boolean hasFieldPose() {
        return hasMegaTag2FieldPose;
    }

    // =================================================================================
    // PUBLIC "SETTER" METHODS
    // =================================================================================

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
            setPipeline(RED_GOAL_PIPELINE);
        } else {
            setPipeline(BLUE_GOAL_PIPELINE);
        }
    }

    /**
     * Sets the vision system to the generic motif detection mode.
     * This should be used during the autonomous init_loop.
     */
    public void setMotifDetectionMode() {
        setPipeline(MOTIF_PIPELINE);
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



    private void resetTracking() {
        this.isTargetVisible = false;
        this.lastTagId = -1;
        this.robotDistanceToTagInTagSpace = -1.0;
        this.lastTx = 0.0;
        this.hasMegaTag2FieldPose = false;
        this.robotPoseInFieldSpace = null;
        this.robotPoseInTagSpace = null;
        this.botposeMT2 = null;
    }



    /**
     * Adds relevant vision data to the telemetry stream for debugging.
     */
    public void addTelemetry() {
        telemetry.addLine("--- Limelight Vision ---");
        if (limelight == null) {
            telemetry.addLine("ERROR: Limelight hardware not found!");
            return;
        }
        if (!isTargetVisible()) {
            telemetry.addData("LL Status", "No Targets Visible");
            return;
        }

        telemetry.addData("LL Status", "Target Visible");
        telemetry.addData("LL Tag ID", getDetectedTagId());
        telemetry.addLine();

        telemetry.addLine("--- Target Pose (Camera Relative) ---");
        telemetry.addData("Angle to Target (tx)", "%.2f deg", getTargetAngleX());

        // Safely get the distance. We use a local variable to avoid calling the getter multiple times.
        double distanceInches = getDistanceToTagInches();
        if (distanceInches > 0) {
            telemetry.addData("Distance to Target", "%.1f in", distanceInches);
        } else {
            telemetry.addData("Distance to Target", "N/A");
        }
        telemetry.addLine();

        telemetry.addLine("--- Limelight Vision GetLatestResult ---");
        LLResult lastResult = limelight.getLatestResult();
        telemetry.addData("getTx", lastResult.getTx());
        telemetry.addData("getTy", lastResult.getTy());
        telemetry.addData("getTa", lastResult.getTa());
        telemetry.addData("getTxNC", lastResult.getTxNC());
        telemetry.addLine();

        telemetry.addLine("--- Limelight robot pose in tag space ---");
        Pose3D robotPoseInTagSpace = getRobotPoseInTagSpace();
        if (robotPoseInTagSpace != null) {

            telemetry.addData("LL PoseTag X ", robotPoseInTagSpace.getPosition().x * METERS_TO_INCHES);
            telemetry.addData("LL PoseTag Y ", robotPoseInTagSpace.getPosition().y * METERS_TO_INCHES);
            telemetry.addData("LL PoseTag Z ", robotPoseInTagSpace.getPosition().z * METERS_TO_INCHES);
            telemetry.addData("LL PoseTag Yaw ", robotPoseInTagSpace.getOrientation().getYaw());
            telemetry.addData("LL Distance (in)", "%.1f", getDistanceToTagInches());
            telemetry.addData("LL Angle (tx)", "%.2f", lastTx);
        } else {
            telemetry.addData("LL PoseTag", "N/A");
        }
        telemetry.addLine();

        if (hasFieldPose()) {
            telemetry.addLine("--- Limelight MegaTag2 Data---");
            telemetry.addData("MegaTag2 X ", botposeMT2.getPosition().x * METERS_TO_INCHES);
            telemetry.addData("MegaTag2 Y ", botposeMT2.getPosition().y * METERS_TO_INCHES);
            telemetry.addData("MegaTag2 Z ", botposeMT2.getPosition().z * METERS_TO_INCHES);
            telemetry.addData("MegaTag2 Yaw ", botposeMT2.getOrientation().getYaw(AngleUnit.DEGREES));   // yaw

        } else {
            telemetry.addData("MegaTag2 Data : ", "N/A");
        }
        telemetry.addLine();

        telemetry.addLine("--- Limelight Robot in FieldSpace Data---");
        Pose3D robotPoseInFieldSpace = getRobotPoseFieldSpace();
        if (robotPoseInFieldSpace != null) {
            telemetry.addData("Robot X in Field Space (in) ", robotPoseInFieldSpace.getPosition().x * METERS_TO_INCHES);
            telemetry.addData("Robot Y in Field Space (in) ", robotPoseInFieldSpace.getPosition().y * METERS_TO_INCHES);
            telemetry.addData("Robot Z in Field Space (in) ", robotPoseInFieldSpace.getPosition().z * METERS_TO_INCHES);
            telemetry.addData("Robot Yaw in Field Space (degv2) ", robotPoseInFieldSpace.getOrientation().getYaw(AngleUnit.DEGREES) );

            telemetry.addData("Robot Distance in Field Space (in) ",
                    Math.hypot(robotPoseInFieldSpace.getPosition().x, robotPoseInFieldSpace.getPosition().z) * METERS_TO_INCHES);
        } else {
            telemetry.addData("robotPoseInFieldSpace", "N/A");
        }
        telemetry.addLine();



    }



}
