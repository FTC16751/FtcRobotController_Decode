package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GGRobotConstants {

    // This private constructor is important! It prevents the class from being instantiated.
    private GGRobotConstants() {}

    public static final class Drive {
        public static final double DRIVE_SPEED = 1.0;

    }

    public static final class Feeder {
        public static final double FEED_TIME_SECONDS = .42;//The feeder servos run this long when a shot is requested.
        public static final double STOP_SPEED = 0.0;//We send this power to the servos when we want them to stop.
        public static final double FULL_SPEED = 1.0;//We send this power to the servos when we want them to stop.
    }

    public static final class GoalLocation {
        public static final double BLUE_TAG20_X_M = -1.482;
        public static final double BLUE_TAG20_Y_M = -1.413;

        public static final double RED_TAG24_X_M  = -1.482; // -58.34 in inches
        public static final double RED_TAG24_Y_M  =  1.413; // 55.63 in inches
    }
    public static final class Launcher {
        public static final double CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
        public static final double CLOSE_MIN_VELOCITY = 1175;//minimum required to start a shot for close goal.
        public static final double FAR_TARGET_VELOCITY = 1400; //Target velocity for far goal
        public static final double FAR_MIN_VELOCITY = 1375;//minimum required to start a shot for far goal
        public static final double AUTO_TARGET_VELOCITY = 1130; //in ticks/second for the close goal.
        public static final double AUTO_MIN_VELOCITY = AUTO_TARGET_VELOCITY-25;//minimum required to start a shot for close goal.
        public static final double TIME_BETWEEN_SHOTS = 0.5;
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

    public static final class Diverter {
        public static final double RIGHT_POSITION = 0.2962;
        public static final double LEFT_POSITION = 0.0;
        public static final double CENTER_POSITION = 0.145;
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
        /* START Poses */
        public static final Pose2D START_RED_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,54);
        public static final Pose2D START_RED_FAR = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D START_BLUE_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,-54);
        public static final Pose2D START_BLUE_FAR = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);


        /* Red Alliance Poses */
        /**** RED CLOSE TO GOAL PATHS ****/
        public static final Pose2D RED_CLOSE_DRIVE_AWAY = new Pose2D(DistanceUnit.INCH,-13.7, 17, AngleUnit.DEGREES,-45);
        public static final Pose2D RED_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,10, 15.6, AngleUnit.DEGREES,-45);


        /**** RED FAR PATHS ****/
        public static final Pose2D RED_FAR_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,8, 0, AngleUnit.DEGREES,-20);
        public static final Pose2D RED_FAR_PARK = new Pose2D(DistanceUnit.INCH,24, 8.8, AngleUnit.DEGREES,0);


        /**** RED SPIKE MARK 3 PATHS ****/
        public static final Pose2D RED_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,26, 0, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK3_BALL1 = new Pose2D(DistanceUnit.INCH,26, -17, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK3_BALL2 = new Pose2D(DistanceUnit.INCH,26, -23, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK3_BALL3 = new Pose2D(DistanceUnit.INCH,26, -39, AngleUnit.DEGREES,-90);

        /**** RED SPIKE MARK 2 PATHS ****/
        public static final Pose2D RED_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,47, 0, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK2_BALL1 = new Pose2D(DistanceUnit.INCH,47, -17, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK2_BALL2 = new Pose2D(DistanceUnit.INCH,47, -23, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK2_BALL3 = new Pose2D(DistanceUnit.INCH,47, -39, AngleUnit.DEGREES,-90);
// use+declare



        /* Blue Alliance Poses */
        /**** BLUE CLOSE TO GOAL PATHS ****/
        public static final Pose2D BLUE_CLOSE_DRIVE_AWAY = new Pose2D(DistanceUnit.INCH,-13.7, -17, AngleUnit.DEGREES,45);
        public static final Pose2D BLUE_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,-12, -3, AngleUnit.DEGREES,45);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-30, -31, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL1 = new Pose2D(DistanceUnit.INCH,-30, -19.5, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL2 = new Pose2D(DistanceUnit.INCH,-30, -14, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3a = new Pose2D(DistanceUnit.INCH,-35, -15, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3 = new Pose2D(DistanceUnit.INCH,-35, -8, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_END = new Pose2D(DistanceUnit.INCH,-36, -5, AngleUnit.DEGREES,90);


        /**** BLUE FAR PATHS ****/
        public static final Pose2D BLUE_FAR_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,8, 0, AngleUnit.DEGREES,20);
        public static final Pose2D BLUE_FAR_PARK = new Pose2D(DistanceUnit.INCH,24, -8.8, AngleUnit.DEGREES,0);


        /**** BLUE SPIKE MARK 3 PATHS ****/
        public static final Pose2D BLUE_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,26,0, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK3_BALL1 = new Pose2D(DistanceUnit.INCH,26, 17, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK3_BALL2 = new Pose2D(DistanceUnit.INCH,26, 23, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK3_BALL3 = new Pose2D(DistanceUnit.INCH,26, 39, AngleUnit.DEGREES,90);


        /**** BLUE SPIKE MARK 2 PATHS ****/
        public static final Pose2D BLUE_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,47,0, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_BALL1 = new Pose2D(DistanceUnit.INCH,47, 17, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_BALL2 = new Pose2D(DistanceUnit.INCH,47, 23, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_BALL3 = new Pose2D(DistanceUnit.INCH,47, 39, AngleUnit.DEGREES,90);

    }
}
