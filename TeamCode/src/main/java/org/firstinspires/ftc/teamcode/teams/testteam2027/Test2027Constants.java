package org.firstinspires.ftc.teamcode.teams.testteam2027;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * HOW test2027bot OPERATES: speeds, timings, waypoints, and the tag it drives to. These change
 * when the game changes or the drivers change their minds. What the robot physically IS lives in
 * Test2027BotConfig.
 *
 * Keep every number an OpMode needs in here, named, so the OpModes read as plain sentences and a
 * student can retune the robot without hunting through them.
 */
public final class Test2027Constants {

    private Test2027Constants() {}

    /** TeleOp driving. */
    public static final class Drive {
        public static final double NORMAL_SPEED   = 0.6;
        public static final double SLOW_SPEED     = 0.25;
        public static final double STICK_DEADBAND = 0.05;
        /** Speeds the beginner auto commands use when no speed is given (driveForward(24) etc.). */
        public static final double AUTO_DRIVE_SPEED = 0.4;
        public static final double AUTO_TURN_SPEED  = 0.3;
    }

    /** The AprilTag approach test. Change TAG_ID to whatever tag is taped to the wall today. */
    public static final class TagTest {
        public static final int    TAG_ID            = 20;
        public static final double STANDOFF_INCHES   = 12.0;
        public static final double HOLD_SECONDS      = 0.5;
    }

    /** Autonomous driving. Power for waypoint moves is Drive.AUTO_DRIVE_SPEED. */
    public static final class Auto {
        /** How long startDriveTo must sit inside tolerance before it counts as arrived. */
        public static final double HOLD_SEC = 0.25;
    }

    /**
     * Waypoints for the drive-a-square auto, relative to where the robot starts (0, 0, heading 0).
     * Pinpoint frame: X forward from the start, Y to the left, heading counter-clockwise positive.
     */
    public static final class Waypoints {
        public static final double SIDE_IN = 24.0;
        public static final Pose2D START    = pose(0,       0,       0);
        public static final Pose2D CORNER_1 = pose(SIDE_IN, 0,       0);
        public static final Pose2D CORNER_2 = pose(SIDE_IN, SIDE_IN, 0);
        public static final Pose2D CORNER_3 = pose(0,       SIDE_IN, 0);
        public static final Pose2D FINISH   = pose(0,       0,       90);   // back at start, turned a quarter turn

        private static Pose2D pose(double xIn, double yIn, double headingDeg) {
            return new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg);
        }
    }
}
