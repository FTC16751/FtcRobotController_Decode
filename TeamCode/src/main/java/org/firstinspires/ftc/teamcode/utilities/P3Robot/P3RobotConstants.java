package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class P3RobotConstants {

    // This private constructor is important! It prevents the class from being instantiated.
    private P3RobotConstants() {}

    public static final class Drive {
        public static final double DRIVE_SPEED = 1.0;

    }

    public static final class Feeder {
        public static final double FEED_TIME_SECONDS = .42;//The feeder servos run this long when a shot is requested.
        public static final double STOP_SPEED = 0.0;//We send this power to the servos when we want them to stop.
        public static final double FULL_SPEED = 1.0;//We send this power to the servos when we want them to stop.
    }

    public static final class Launcher {
        public static final double CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
        public static final double CLOSE_MIN_VELOCITY = 1175;//minimum required to start a shot for close goal.
        public static final double FAR_TARGET_VELOCITY = 1450; //Target velocity for far goal
        public static final double FAR_MIN_VELOCITY = 1325;//minimum required to start a shot for far goal
        public static final double AUTO_TARGET_VELOCITY = 1000; //in ticks/second for the close goal.
        public static final double AUTO_MIN_VELOCITY = AUTO_TARGET_VELOCITY-25;//minimum required to start a shot for close goal.
        public static final double TIME_BETWEEN_SHOTS = 1.0;
    }

    public enum LauncherDistance {
        // Each state now holds its own values, pulling from the constants above.
        CLOSE(Launcher.CLOSE_TARGET_VELOCITY, Launcher.CLOSE_MIN_VELOCITY),
        FAR(Launcher.FAR_TARGET_VELOCITY, Launcher.FAR_MIN_VELOCITY),
        AUTO(Launcher.AUTO_TARGET_VELOCITY, Launcher.AUTO_MIN_VELOCITY);

        // These variables will hold the values for each state.
        public final double targetVelocity;
        public final double minVelocity;

        // This is the "constructor" that sets up each state.
        LauncherDistance(double targetVelocity, double minVelocity) {
            this.targetVelocity = targetVelocity;
            this.minVelocity = minVelocity;
        }
    }


    public static final class Intake {
        public static final double INTAKE_SPEED = 1.0;
        public static final double OUTTAKE_SPEED = -1.0;
    }
    public enum LauncherSystemState {
        IDLE,
        ACTIVE
    }

    public enum LauncherTargetingMode {
        AUTO,    // Use velocity calculated from vision or odometry
        PRESET   // Use the manually toggled CLOSE/FAR presets
    }
    // --- NEW: Public Enums for Autonomous Selection ---
    public enum Alliance { RED, BLUE }
    public enum Location { CLOSE, FAR }

    // --- NEW: Public Waypoints for Autonomous Paths ---
    public static final class Waypoints {

        /* Red Alliance Poses */
        // ---- CLOSE TO GOAL START ----//
        public static final Pose2D START_RED_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_CLOSE_DRIVE_AWAY = new Pose2D(DistanceUnit.INCH,-26, 36, AngleUnit.DEGREES,-50);
        public static final Pose2D RED_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,0, 24, AngleUnit.DEGREES,0);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-39, 19, AngleUnit.DEGREES,90);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,-38, -3.1, AngleUnit.DEGREES,90);

        // ---- FAR FROM GOAL START ----//
        public static final Pose2D START_RED_FAR = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_FAR_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,8.5, 12, AngleUnit.DEGREES,-21);
        public static final Pose2D RED_FAR_PARK = new Pose2D(DistanceUnit.INCH,24, -6, AngleUnit.DEGREES,0);
        public static final Pose2D RED_FAR_DRIVE_AWAY = new Pose2D(DistanceUnit.INCH,24, -6, AngleUnit.DEGREES,0); // CHANGE
        public static final Pose2D RED_FAR_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,24, -6, AngleUnit.DEGREES,0); // CHANGE

        public static final Pose2D RED_FAR_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,24, -6, AngleUnit.DEGREES,0); // CHANGE


        // * Blue Alliance Poses */
        // ---- CLOSE TO GOAL START ----//
        public static final Pose2D START_BLUE_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D BLUE_CLOSE_DRIVE_AWAY = new Pose2D(DistanceUnit.INCH,-26, -36, AngleUnit.DEGREES,50);
        public static final Pose2D BLUE_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,0, -24, AngleUnit.DEGREES,0);

        // ---- FAR FROM GOAL START ----//
        public static final Pose2D START_BLUE_FAR = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D BLUE_FAR_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,8.5, -12, AngleUnit.DEGREES,21);
        public static final Pose2D BLUE_FAR_PARK = new Pose2D(DistanceUnit.INCH,27, 6, AngleUnit.DEGREES,0);


        // -- SPIKE MARK LOCATIONS -- //
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-39, -24, AngleUnit.DEGREES,-90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,-38, 1, AngleUnit.DEGREES,-90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL1 = new Pose2D(DistanceUnit.INCH,-30, -19.5, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL2 = new Pose2D(DistanceUnit.INCH,-30, -14, AngleUnit.DEGREES,90);

        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3a = new Pose2D(DistanceUnit.INCH,-35, -15, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3 = new Pose2D(DistanceUnit.INCH,-35, -8, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_END = new Pose2D(DistanceUnit.INCH,-36, -5, AngleUnit.DEGREES,90);


    }
}
