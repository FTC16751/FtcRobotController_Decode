package org.firstinspires.ftc.teamcode.common;

import android.util.Log;
import androidx.annotation.NonNull;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
public class VisionUtil implements AimTarget, TagSighting {

    private Limelight3A limelight;
    private final Telemetry telemetry;

    // --- State variables to hold the latest processed data ---
    private boolean isTargetVisible = false;
    private int lastTagId = -1;
    private double robotDistanceToTagInTagSpace = -1.0;
    private double lastTx = 0.0; // Horizontal angle
    private LLResult lastValidResult = null;
    private static final double METERS_TO_INCHES = CommonConstants.METERS_TO_INCHES;

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






    /**
     * Initializes the Limelight 3A vision system using the default device name "limelight".
     * Prefer the three-argument constructor with the name from the robot's RobotConfig.
     * @param hardwareMap The HardwareMap from the OpMode, used to find the Limelight.
     * @param telemetry The Telemetry object used for logging.
     */
    public VisionUtil(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "limelight");
    }

    /**
     * Initializes the Limelight 3A vision system.
     * @param hardwareMap   The HardwareMap from the OpMode, used to find the Limelight.
     * @param telemetry     The Telemetry object used for logging.
     * @param limelightName Device name from RobotConfig.hardware.limelight. If null, this robot has
     *                      no Limelight: every query reports "no target" and nothing throws.
     */
    public VisionUtil(@NonNull HardwareMap hardwareMap, Telemetry telemetry, String limelightName) {
        this.telemetry = telemetry;
        if (limelightName == null) {
            this.limelight = null;
            telemetry.addData("Limelight", "none in this robot's config; vision disabled");
            return;
        }
        try {
            limelight = hardwareMap.get(Limelight3A.class, limelightName);
            limelight.pipelineSwitch(CommonConstants.Limelight.MOTIF_PIPELINE);
            limelight.start();
            telemetry.addData("Limelight", "Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("Limelight ERROR", "'" + limelightName + "' NOT found in config. Check name.");
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
        robotDistanceToTagInTagSpace = Math.hypot(x_meters, z_meters);
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
        return calculateHeadingErrorToAFieldPoint(CommonConstants.Field.RED_GOAL_X_M, CommonConstants.Field.RED_GOAL_Y_M, AngleUnit.DEGREES);
    }
    public double calculateHeadingErrorToBlueGoalDegrees() {
        return calculateHeadingErrorToAFieldPoint(CommonConstants.Field.BLUE_GOAL_X_M, CommonConstants.Field.BLUE_GOAL_Y_M, AngleUnit.DEGREES);
    }

    /**
     * Checks if a valid AprilTag was visible during the last update cycle.
     * @return true if a target is currently visible, false otherwise.
     */
    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    // =================================================================================
    // TagSighting: one tag in the robot's frame, for TagApproach
    // =================================================================================
    //
    // Built from the Limelight's "target pose in robot space" for the requested tag id. The four
    // constants below are the ONLY place the Limelight's axis conventions meet ours, and they are
    // the thing to check on a stand (doc/ROBOT_TEST_PLAN.md, test H1). If a number in the
    // TagApproach telemetry moves the wrong way, flip the matching constant here. Never change
    // TagApproach for a sign problem.
    //
    // Limelight robot space as the FTC SDK reports it (checked on the Skyline chassis 2026-09-07,
    // robot square in front of the blue goal): CAMERA axes, X+ to the RIGHT, Y+ DOWN, Z+ FORWARD.
    // So forward distance is Z and lateral offset is X; Y is the tag's height above the camera and
    // is not used. The rotation about the vertical axis arrives as PITCH (rotation about Y-down):
    // turning the robot 30 degrees LEFT read pitch +33 while yaw and roll stayed near 0, so the
    // square-up error is minus pitch. All four signs below were confirmed on the robot 2026-09-07.

    private static final double TAG_FORWARD_SIGN          = 1.0;   // flip if forward error rises as the tag gets closer
    private static final double TAG_RIGHT_SIGN            = 1.0;   // flip if a tag to the robot's right reads negative
    private static final double TAG_SQUARE_SIGN           = -1.0;  // confirmed: robot turned left, pitch went positive
    private static final double TAG_SQUARE_YAW_OFFSET_DEG = 0.0;   // pitch reads 0 when square; no offset needed

    private LLResultTypes.FiducialResult sightedTag = null;   // the tag canSee() last found, this loop

    /** Call this first each loop; the three getters below describe the tag it found. */
    @Override
    public boolean canSee(int tagId) {
        sightedTag = getFiducialById(tagId);
        if (sightedTag != null && sightedTag.getTargetPoseRobotSpace() == null) {
            sightedTag = null;
        }
        return sightedTag != null;
    }

    @Override
    public double forwardInches() {
        if (sightedTag == null) return 0.0;
        return TAG_FORWARD_SIGN * sightedTag.getTargetPoseRobotSpace().getPosition().z * METERS_TO_INCHES;
    }

    @Override
    public double rightInches() {
        if (sightedTag == null) return 0.0;
        return TAG_RIGHT_SIGN * sightedTag.getTargetPoseRobotSpace().getPosition().x * METERS_TO_INCHES;
    }

    /**
     * The raw robot-space pose of the tag canSee() last found, for the stand check: position x y z
     * in inches and orientation yaw pitch roll in degrees, straight from the Limelight. Use it to
     * see which orientation component moves when the tag is angled; that one is the square-up.
     */
    public String sightedTagRaw() {
        if (sightedTag == null) return "none";
        Pose3D t = sightedTag.getTargetPoseRobotSpace();
        return String.format("x %.1f y %.1f z %.1f in | yaw %.1f pitch %.1f roll %.1f deg",
                t.getPosition().x * METERS_TO_INCHES, t.getPosition().y * METERS_TO_INCHES, t.getPosition().z * METERS_TO_INCHES,
                t.getOrientation().getYaw(AngleUnit.DEGREES), t.getOrientation().getPitch(AngleUnit.DEGREES), t.getOrientation().getRoll(AngleUnit.DEGREES));
    }

    @Override
    public double squareUpDegrees() {
        if (sightedTag == null) return 0.0;
        // Rotation about the vertical axis is the PITCH component in the SDK's Limelight pose (Y is down).
        double aboutVerticalDeg = sightedTag.getTargetPoseRobotSpace().getOrientation().getPitch(AngleUnit.DEGREES);
        double error = aboutVerticalDeg - TAG_SQUARE_YAW_OFFSET_DEG;
        while (error > 180)  error -= 360;
        while (error < -180) error += 360;
        return TAG_SQUARE_SIGN * error;
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
    public void setTargetingAlliance(CommonConstants.Alliance alliance) {
        if (alliance == CommonConstants.Alliance.RED) {
            setPipeline(CommonConstants.Limelight.RED_GOAL_PIPELINE);
        } else {
            setPipeline(CommonConstants.Limelight.BLUE_GOAL_PIPELINE);
        }
    }

    /**
     * Sets the vision system to the generic motif detection mode.
     * This should be used during the autonomous init_loop.
     */
    public void setMotifDetectionMode() {
        setPipeline(CommonConstants.Limelight.MOTIF_PIPELINE);
    }

    /**
     * Switch to the pipeline that can see this tag. The Limelight only reports tags its current
     * pipeline is configured for: 21/22/23 (motif) on pipeline 0, the red goal 24 on 1, the blue
     * goal 20 on 2 (CommonConstants.Limelight). A tag approach to a goal tag has to call this
     * first, or the tag is never "seen". Switching takes a moment; do it in init, not per loop.
     */
    public void selectPipelineForTag(int tagId) {
        if (tagId == CommonConstants.Limelight.BLUE_GOAL_TAG)      setPipeline(CommonConstants.Limelight.BLUE_GOAL_PIPELINE);
        else if (tagId == CommonConstants.Limelight.RED_GOAL_TAG)  setPipeline(CommonConstants.Limelight.RED_GOAL_PIPELINE);
        else                                                       setPipeline(CommonConstants.Limelight.MOTIF_PIPELINE);
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
