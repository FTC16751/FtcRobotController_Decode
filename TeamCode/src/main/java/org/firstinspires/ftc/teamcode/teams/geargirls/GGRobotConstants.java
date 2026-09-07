package org.firstinspires.ftc.teamcode.teams.geargirls;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.LedUtil;

public class GGRobotConstants {

    // This private constructor is important! It prevents the class from being instantiated.
    private GGRobotConstants() {}

    public static final class Drive {
        public static final double DRIVE_SPEED = 1.0;

    }

    public static final class Feeder {
        public static final double FEED_TIME_SECONDS = .5;//The feeder servos run this long when a shot is requested.
        public static final double STOP_SPEED = 0.0;//We send this power to the servos when we want them to stop.
        public static final double FULL_SPEED = 1.0;//We send this power to the servos when we want them to stop.
    }
    public static final class Launcher {
        public static final double CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
        public static final double CLOSE_MIN_VELOCITY = 1150;//minimum required to start a shot for close goal.
        public static final double FAR_TARGET_VELOCITY = 2000; //Target velocity for far goal
        public static final double FAR_MIN_VELOCITY = 1980;//minimum required to start a shot for far goal
        public static final double AUTO_TARGET_VELOCITY = 1850; //in ticks/second for the close goal.
        public static final double AUTO_MIN_VELOCITY = AUTO_TARGET_VELOCITY-50;//minimum required to start a shot for close goal.
        public static final double TIME_BETWEEN_SHOTS = 0.5;

        /**
         * Distance to the goal (inches) -> flywheel velocity (ticks/sec), used by FlywheelVelocityModel.
         * TODO: re-tune for the 72mm wheels; these values are from the 96mm wheel configuration.
         */
        public static final double[][] FLYWHEEL_TABLE = {
                { 30.0, 1290.0},
                { 40.0, 1370.0},
                { 50.0, 1440.0},
                { 60.0, 1500.0},
                { 70.0, 1540.0},
                { 80.0, 1600.0},
                { 90.0, 1700.0},
                {100.0, 1760.0},
                {110.0, 1880.0},
                {120.0, 1950.0},
                {130.0, 2030.0},
                {140.0, 2100.0},
                {160.0, 2250.0},
        };
        /** Velocity to shoot at before the goal has ever been seen (the close-range table value). */
        public static final double FLYWHEEL_INITIAL_FALLBACK = 1290.0;
    }

    /** Aim feedback on the status LED (AimLed). */
    public static final class Aim {
        public static final double LED_TOLERANCE_DEG = 2.0;
        public static final double LED_GOAL_RIGHT = LedUtil.Color.YELLOW;
        public static final double LED_GOAL_LEFT  = LedUtil.Color.BLUE;
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
        PRESET,   // Use the manually toggled CLOSE/FAR presets,
        MANUAL
    }
    // --- NEW: Public Enums for Autonomous Selection ---

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
        public static final Pose2D RED_FAR_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,8, 0, AngleUnit.DEGREES,-22);
        public static final Pose2D RED_FAR_PARK = new Pose2D(DistanceUnit.INCH,24, 8.8, AngleUnit.DEGREES,0);


        /**** RED SPIKE MARK 3 PATHS ****/
        public static final Pose2D RED_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,26, -10, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK3_BALL1 = new Pose2D(DistanceUnit.INCH,26, -17, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK3_BALL2 = new Pose2D(DistanceUnit.INCH,26, -23, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK3_BALL3 = new Pose2D(DistanceUnit.INCH,26, -39, AngleUnit.DEGREES,-90);

        /**** RED SPIKE MARK 2 PATHS ****/
        public static final Pose2D RED_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,47, -10, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK2_BALL1 = new Pose2D(DistanceUnit.INCH,47, -17, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK2_BALL2 = new Pose2D(DistanceUnit.INCH,47, -23, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_FAR_SPIKEMARK2_BALL3 = new Pose2D(DistanceUnit.INCH,47, -39, AngleUnit.DEGREES,-90);
// use+declare

        public static final Pose2D RED_CLOSE_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,-20, 27, AngleUnit.DEGREES,-45);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-33, 28, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_BALL1 = new Pose2D(DistanceUnit.INCH,-33, 18.5, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_BALL2 = new Pose2D(DistanceUnit.INCH,-33, 12, AngleUnit.DEGREES,-90);
        //public static final Pose2D RED_CLOSE_SPIKEMARK1_BALL3a = new Pose2D(DistanceUnit.INCH,-32, 15, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_BALL3 = new Pose2D(DistanceUnit.INCH,-33, 0, AngleUnit.DEGREES,-90);
        //public static final Pose2D RED_CLOSE_SPIKEMARK1_END = new Pose2D(DistanceUnit.INCH,-36, 5, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,-56,28, AngleUnit.DEGREES, -90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_BALL1_COLLECT = new Pose2D(DistanceUnit.INCH,-56,21, AngleUnit.DEGREES, -90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_BALL2_COLLECT = new Pose2D(DistanceUnit.INCH,-56,17, AngleUnit.DEGREES, -90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_BALL3_COLLECT = new Pose2D(DistanceUnit.INCH,-56,0, AngleUnit.DEGREES, -90);


        /* Blue Alliance Poses */
        /**** BLUE CLOSE TO GOAL PATHS ****/
        public static final Pose2D BLUE_CLOSE_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,-32, -34, AngleUnit.DEGREES,45);
        public static final Pose2D BLUE_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,-41, -21, AngleUnit.DEGREES,45);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-30, -31, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL1 = new Pose2D(DistanceUnit.INCH,-30, -19.5, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL2 = new Pose2D(DistanceUnit.INCH,-30, -15, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3a = new Pose2D(DistanceUnit.INCH,-35, -15, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3 = new Pose2D(DistanceUnit.INCH,-35, -8, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_END = new Pose2D(DistanceUnit.INCH,-36, -5, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,-56,-31, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_BALL1_COLLECT = new Pose2D(DistanceUnit.INCH,-56,-21, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_BALL2_COLLECT = new Pose2D(DistanceUnit.INCH,-58,-15, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_BALL3_COLLECT = new Pose2D(DistanceUnit.INCH,-60,-10, AngleUnit.DEGREES, 90);
        //end new

        /**** BLUE FAR PATHS ****/
        public static final Pose2D BLUE_FAR_DRIVE_TO_SCORE = new Pose2D(DistanceUnit.INCH,8, 0, AngleUnit.DEGREES,20);
        public static final Pose2D BLUE_FAR_PARK = new Pose2D(DistanceUnit.INCH,24, -8.8, AngleUnit.DEGREES,0);


        /**** BLUE SPIKE MARK 3 PATHS ****/
        public static final Pose2D BLUE_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,26,10, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK3_BALL1 = new Pose2D(DistanceUnit.INCH,26, 17, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK3_BALL2 = new Pose2D(DistanceUnit.INCH,26, 23, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK3_BALL3 = new Pose2D(DistanceUnit.INCH,26, 39, AngleUnit.DEGREES,90);


        /**** BLUE SPIKE MARK 2 PATHS ****/
        public static final Pose2D BLUE_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,47,10, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_BALL1 = new Pose2D(DistanceUnit.INCH,47, 17, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_BALL2 = new Pose2D(DistanceUnit.INCH,47, 23, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_BALL3 = new Pose2D(DistanceUnit.INCH,47, 39, AngleUnit.DEGREES,90);



    }
}
