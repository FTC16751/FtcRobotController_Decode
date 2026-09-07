package org.firstinspires.ftc.teamcode.common;

/**
 * Constants that are true for EVERY robot because they describe the game, the field, or a
 * contract every team follows. Nothing robot-specific belongs here; that goes in each team's
 * Constants class (GGRobotConstants, P3RobotConstants, ...) or in its RobotConfig.
 *
 * This is the single source of truth for the types below. Do not redeclare Alliance or Location
 * in a team package or inside an OpMode; import this class instead. Two enums with the same values
 * are still different Java types, and a private copy cannot be handed to SharedState or VisionUtil.
 */
public class CommonConstants {

    private CommonConstants() {}

    /** Which alliance the robot is playing for. Selected in auto init, carried to TeleOp by SharedState. */
    public enum Alliance {
        RED,
        BLUE
    }

    /** Which starting position the robot is in for autonomous. */
    public enum Location {
        CLOSE,
        FAR
    }

    /** Unit conversion used wherever Limelight pose data (meters) is shown or compared in inches. */
    public static final double METERS_TO_INCHES = 39.3701;

    /**
     * DECODE field geometry in Limelight field-space coordinates (meters).
     * The goal AprilTags are tag 20 (blue) and tag 24 (red).
     */
    public static final class Field {
        public static final double BLUE_GOAL_X_M = -1.482;
        public static final double BLUE_GOAL_Y_M = -1.413;
        public static final double RED_GOAL_X_M  = -1.482;   // -58.34 in
        public static final double RED_GOAL_Y_M  =  1.413;   //  55.63 in

        private Field() {}
    }

    /**
     * Limelight pipeline contract. EVERY robot's Limelight must be configured with these pipeline
     * indices or VisionUtil will target the wrong thing. Set them in the Limelight web UI.
     */
    public static final class Limelight {
        public static final int MOTIF_PIPELINE     = 0;   // AprilTags 21/22/23, the obelisk motif
        public static final int RED_GOAL_PIPELINE  = 1;   // AprilTag 24
        public static final int BLUE_GOAL_PIPELINE = 2;   // AprilTag 20

        private Limelight() {}
    }
}
