package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.LaunchIndexer.FeederSide;

import java.util.Objects;

// Using a Java Record is a modern, concise way to create an immutable data class.// It's perfect for representing a single, unchangeable action in a sequence.

// This class represents a single, atomic action in an autonomous sequence.
public final class AutoAction {

    // An enum to clearly define what kind of action this is.
    public enum ActionType {
        DRIVE_TO_POINT, // A Pinpoint-based drive action
        SHOOT,          // A launching action
        CUSTOM          // For other actions like running an intake (for the future)
    }

    public final ActionType type;
    public final String description;

    // --- Fields for different action types ---
    public final Pose2D targetPose; // For DRIVE_TO_POINT actions
    public final FeederSide feederSide; // For SHOOT actions

    // --- Private constructor ---
    private AutoAction(ActionType type, String description, @Nullable Pose2D targetPose, @Nullable FeederSide feederSide) {
        this.type = type;
        this.description = description;
        this.targetPose = targetPose;
        this.feederSide = feederSide;
    }
    /**
     * Constructor for a DRIVE_TO_POINT action.
     */
    public AutoAction(String description, Pose2D targetPose) {
        this.type = ActionType.DRIVE_TO_POINT;
        this.description = description;
        this.targetPose = targetPose;
        this.feederSide = null; // Not used for this action type
    }

    /**
     * Constructor for a SHOOT action.
     */
    public AutoAction(String description, FeederSide feederSide) {
        this.type = ActionType.SHOOT;
        this.description = description;
        this.targetPose = null; // Not used for this action type
        this.feederSide = feederSide;
    }


    /**
     * Creates a DRIVE_TO_POINT action.
     * @param description A human-readable description of the action.
     * @param targetPose The target pose for the robot to drive to.
     * @return A new AutoAction instance for driving.
     */
    public static AutoAction createDriveAction(@NonNull String description, @NonNull Pose2D targetPose) {
        Objects.requireNonNull(description, "Description cannot be null");
        Objects.requireNonNull(targetPose, "targetPose cannot be null for a drive action");
        return new AutoAction(ActionType.DRIVE_TO_POINT, description, targetPose, null);
    }


    /**
     * Creates a SHOOT action.
     * @param description A human-readable description of the action.
     * @param feederSide The side from which to shoot.
     * @return A new AutoAction instance for shooting.
     */
    public static AutoAction createShootAction(@NonNull String description, @NonNull FeederSide feederSide) {
        Objects.requireNonNull(feederSide, "feederSide cannot be null for a shoot action");
        return new AutoAction(ActionType.SHOOT, description, null, feederSide);
    }
}
