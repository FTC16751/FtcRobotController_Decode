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

    public static final class Turret {
        // Manual control speeds
        public static final double MANUAL_SPEED = 0.5;
        public static final double FINE_SPEED = 0.3;
        public static final double COARSE_SPEED = 0.8;

        // Auto-aim parameters
        public static final double AUTO_AIM_KP = 0.02;
        public static final double AIM_TOLERANCE_DEG = 2.0;

        // Preset positions
        public static final double PRESET_LEFT = -45.0;
        public static final double PRESET_RIGHT = 45.0;
        public static final double PRESET_CENTER = 0.0;
    }

    public static final class TeleOp {
        // Drive speed multipliers
        public static final double DRIVE_SPEED_NORMAL = 0.85;
        public static final double DRIVE_SPEED_PRECISION = 0.50;
        public static final double DRIVE_SPEED_TURBO = 1.00;
        public static final double FINE_ADJUSTMENT_SPEED = 0.15;

        // Launcher velocity presets (ticks/sec)
        public static final double NEAR_SHOT_VELOCITY = 2500.0;
        public static final double FAR_SHOT_VELOCITY = 3500.0;
        public static final double VELOCITY_INCREMENT = 100.0;

        // Launch parameters
        public static final double SHOOTER_VELOCITY_TOLERANCE_PERCENT = 0.95;
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

    // --- ROBOT NUMBER 1 (DO NOT EDIT): Public Waypoints for Autonomous Paths ---
    public static final class Waypoints {

        /* Red Alliance Poses */
        // ---- CLOSE TO GOAL START ----//
        public static final Pose2D START_RED_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_CLOSE_DRIVE_AWAY = new Pose2D(DistanceUnit.INCH,-26, 36, AngleUnit.DEGREES,-50);
        public static final Pose2D RED_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,0, 24, AngleUnit.DEGREES,0);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-39, 24, AngleUnit.DEGREES,90);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,-38, -3.4, AngleUnit.DEGREES,90);

        // ---- FAR FROM GOAL START ----//
        public static final Pose2D START_RED_FAR = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_FAR_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH,8.5, 12, AngleUnit.DEGREES,-24);
        public static final Pose2D RED_FAR_PARK_POSITION = new Pose2D(DistanceUnit.INCH,24, -6, AngleUnit.DEGREES,0);
        public static final Pose2D RED_FAR_DRIVE_AWAY = new Pose2D(DistanceUnit.INCH,24, -6, AngleUnit.DEGREES,0); // CHANGE
        public static final Pose2D RED_FAR_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,24, -6, AngleUnit.DEGREES,90); // CHANGE

        public static final Pose2D RED_FAR_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,29, -6, AngleUnit.DEGREES,90); // CHANGE


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
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,-38, 3.4, AngleUnit.DEGREES,-90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL1 = new Pose2D(DistanceUnit.INCH,-30, -19.5, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL2 = new Pose2D(DistanceUnit.INCH,-30, -14, AngleUnit.DEGREES,90);

        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3a = new Pose2D(DistanceUnit.INCH,-35, -15, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_BALL3 = new Pose2D(DistanceUnit.INCH,-35, -8, AngleUnit.DEGREES,90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_END = new Pose2D(DistanceUnit.INCH,-36, -5, AngleUnit.DEGREES,90);


    }
    // --- ROBOT NUMBER 2 (EDIT): Public Waypoints for Autonomous Paths ---
    public static final class Bot2_Waypoints {

        /* Red Alliance Poses */
        // ---- CLOSE TO GOAL START ----//
        public static final Pose2D START_RED_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_CLOSE_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH,34, -26, AngleUnit.DEGREES,-45);
        public static final Pose2D RED_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,-6, -14, AngleUnit.DEGREES,0);


        // ---- FAR FROM GOAL START ----//
        public static final Pose2D RED_FAR_START_POSITION = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_FAR_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH,-8.5, -6, AngleUnit.DEGREES,-30);
        public static final Pose2D RED_FAR_PARK_POSITION = new Pose2D(DistanceUnit.INCH,-24, 0, AngleUnit.DEGREES,0);


        // * RED SPIKEMARK POSITIONS */
        public static final Pose2D RED_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,42, -15, AngleUnit.DEGREES,90);
        public static final Pose2D RED_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,42, 11, AngleUnit.DEGREES,90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,66, -15, AngleUnit.DEGREES,90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH,66, 17, AngleUnit.DEGREES,90);
        public static final Pose2D RED_CLOSE_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,90, -15, AngleUnit.DEGREES,90);
        public static final Pose2D RED_CLOSE_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH,90, 17, AngleUnit.DEGREES,90);


        /** RED FAR **/
        public static final Pose2D RED_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,-28, 14, AngleUnit.DEGREES,90);
        public static final Pose2D RED_FAR_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH,-28, 49, AngleUnit.DEGREES,90);
        public static final Pose2D RED_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,-52, 14, AngleUnit.DEGREES,90);
        public static final Pose2D RED_FAR_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH,-52, 49, AngleUnit.DEGREES,90);

        public static final Pose2D RED_FAR_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH, -76, 14,AngleUnit.DEGREES, 90);
        public static final Pose2D RED_FAR_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH, -76, 42,AngleUnit.DEGREES, 90);



        // ALIGN AND OPEN GATE RED //
        public static final Pose2D RED_ALIGN_GATE = new Pose2D(DistanceUnit.INCH,54, 0, AngleUnit.DEGREES,90);
        public static final Pose2D RED_OPEN_GATE = new Pose2D(DistanceUnit.INCH,54, 11.5, AngleUnit.DEGREES,90);


        // * Blue Alliance Poses */
        // ---- CLOSE TO GOAL START ----//
        public static final Pose2D START_BLUE_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D BLUE_CLOSE_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH,34, 26, AngleUnit.DEGREES,45);
        public static final Pose2D BLUE_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,-6, 14, AngleUnit.DEGREES,0);


        // ---- FAR FROM GOAL START ----//
        public static final Pose2D START_BLUE_FAR = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D BLUE_FAR_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH,-8.5, 4, AngleUnit.DEGREES,22);
        public static final Pose2D BLUE_FAR_PARK_POSITION = new Pose2D(DistanceUnit.INCH,-24, 0, AngleUnit.DEGREES,0);


        // -- BLUE SPIKE MARK LOCATIONS -- //
        //NEAR SIDE
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,42, 18, AngleUnit.DEGREES,-90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,42, -12.0, AngleUnit.DEGREES,-90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,66, 26, AngleUnit.DEGREES,-90); // CHANGE
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH,66, -20.0, AngleUnit.DEGREES,-90); // CHANGE
        public static final Pose2D BLUE_CLOSE_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,90, 26, AngleUnit.DEGREES,-90); // CHANGE
        public static final Pose2D BLUE_CLOSE_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH,90, -20.0, AngleUnit.DEGREES,-90); // CHANGE

        //FAR SIDE:
        public static final Pose2D BLUE_FAR_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-76, -14, AngleUnit.DEGREES,-90); // CHANGED
        public static final Pose2D BLUE_FAR_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,-76, -41, AngleUnit.DEGREES,-90); // CHANGED
        public static final Pose2D BLUE_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,-52, -14, AngleUnit.DEGREES,-90); // CHANGED
        public static final Pose2D BLUE_FAR_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH,-52, -49, AngleUnit.DEGREES,-90); // CHANGED
        public static final Pose2D BLUE_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,-28, -14, AngleUnit.DEGREES,-90); // CHANGED
        public static final Pose2D BLUE_FAR_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH,-28, -49, AngleUnit.DEGREES,-90); // CHANGED


        // ALIGN AND OPEN GATE BLUE //
        public static final Pose2D BLUE_ALIGN_GATE = new Pose2D(DistanceUnit.INCH,52, 4, AngleUnit.DEGREES,-90);
        public static final Pose2D BLUE_OPEN_GATE = new Pose2D(DistanceUnit.INCH,52, -13, AngleUnit.DEGREES,-90);

        // ---- FAR FROM GOAL START ----//
        // blue far shoot
        // x = -8.5 y = 3.7295  heading = 18.3
        //blue align to spikemark 3
        // x = -12.0 y = 10.1 heading = 90
        //blue sm3 collect
        // x = -12.1 y = -25 heading = 90
    }

    public static final class Bot3_Waypoints {

        /* Red Alliance Poses */
        // ---- CLOSE TO GOAL START ----//
        public static final Pose2D START_RED_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_CLOSE_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH,-26, 36, AngleUnit.DEGREES,-40);
        public static final Pose2D RED_CLOSE_PARK = new Pose2D(DistanceUnit.INCH,0, 36, AngleUnit.DEGREES,0);


        // ---- FAR FROM GOAL START ----//
        public static final Pose2D RED_FAR_START_POSITION = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D RED_FAR_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH,8.5, 6, AngleUnit.DEGREES,-30);
        public static final Pose2D RED_FAR_PARK_POSITION = new Pose2D(DistanceUnit.INCH,26, 0, AngleUnit.DEGREES,0);


        // * RED SPIKEMARK POSITIONS */
        public static final Pose2D RED_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH,-30, 26, AngleUnit.DEGREES,-90);

        public static final Pose2D RED_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH,-30,2 , AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH,-54, 26, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH,-54, -3, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH,-78, 26, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_CLOSE_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH,-78, -3, AngleUnit.DEGREES,-90);


        /** RED FAR **/
        public static final Pose2D RED_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH, 28, -12, AngleUnit.DEGREES, -90);

        public static final Pose2D RED_FAR_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH, 28, -45, AngleUnit.DEGREES, -90);
        public static final Pose2D RED_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH, 52, -12, AngleUnit.DEGREES, -90);
        public static final Pose2D RED_FAR_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH, 52, -45, AngleUnit.DEGREES, -90);
        public static final Pose2D RED_FAR_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH, 76, -12, AngleUnit.DEGREES, -90);
        public static final Pose2D RED_FAR_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH, 76, -38, AngleUnit.DEGREES, -90);


        // ALIGN AND OPEN GATE RED //
        public static final Pose2D RED_ALIGN_GATE = new Pose2D(DistanceUnit.INCH,42, 2, AngleUnit.DEGREES,-90);
        public static final Pose2D RED_OPEN_GATE = new Pose2D(DistanceUnit.INCH,42, -2, AngleUnit.DEGREES,-90);


        // * Blue Alliance Poses */
        // ---- CLOSE TO GOAL START ----//
        public static final Pose2D START_BLUE_CLOSE = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D BLUE_CLOSE_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH, -26, -35, AngleUnit.DEGREES, 45);
        public static final Pose2D BLUE_CLOSE_PARK = new Pose2D(DistanceUnit.INCH, 0, -36, AngleUnit.DEGREES, 0);



        // ---- FAR FROM GOAL START ----//
        public static final Pose2D START_BLUE_FAR = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0);
        public static final Pose2D BLUE_FAR_SHOOTING_POSITION = new Pose2D(DistanceUnit.INCH, 8.5, -5, AngleUnit.DEGREES, 22);
        public static final Pose2D BLUE_FAR_PARK_POSITION = new Pose2D(DistanceUnit.INCH, 26, 0, AngleUnit.DEGREES, 0);


        // -- BLUE SPIKE MARK LOCATIONS -- //
        //NEAR SIDE
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH, -30, -25, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH, -30, 0, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH, -54, -25, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH, -54, 5, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH, -78, -25, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_CLOSE_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH, -78, 5, AngleUnit.DEGREES, 90);

        //FAR SIDE:
        public static final Pose2D BLUE_FAR_SPIKEMARK3_ALIGN = new Pose2D(DistanceUnit.INCH, 28, 14, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_FAR_SPIKEMARK3_COLLECT = new Pose2D(DistanceUnit.INCH, 28, 45, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_ALIGN = new Pose2D(DistanceUnit.INCH, 52, 14, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_FAR_SPIKEMARK2_COLLECT = new Pose2D(DistanceUnit.INCH, 52, 45, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_FAR_SPIKEMARK1_ALIGN = new Pose2D(DistanceUnit.INCH, 76, 14, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_FAR_SPIKEMARK1_COLLECT = new Pose2D(DistanceUnit.INCH, 76, 38, AngleUnit.DEGREES, 90);



        // ALIGN AND OPEN GATE BLUE //
        public static final Pose2D BLUE_ALIGN_GATE = new Pose2D(DistanceUnit.INCH, -42, -4, AngleUnit.DEGREES, 90);
        public static final Pose2D BLUE_OPEN_GATE = new Pose2D(DistanceUnit.INCH, -42, 0, AngleUnit.DEGREES, 90);

        // ---- FAR FROM GOAL START ----//
        // blue far shoot
        // x = -8.5 y = 3.7295  heading = 18.3
        //blue align to spikemark 3
        // x = -12.0 y = 10.1 heading = 90
        //blue sm3 collect
        // x = -12.1 y = -25 heading = 90
    }
}