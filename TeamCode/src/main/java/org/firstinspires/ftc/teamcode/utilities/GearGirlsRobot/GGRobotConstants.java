package org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot;

public class GGRobotConstants {

    // This private constructor is important! It prevents the class from being instantiated.
    private GGRobotConstants() {}

    public static final class Drive {
        public static final double DRIVE_SPEED = 1.0;

    }

    public static final class Feeder {
        public static final double FEED_TIME_SECONDS = 0.42;//The feeder servos run this long when a shot is requested.
        public static final double STOP_SPEED = 0.0;//We send this power to the servos when we want them to stop.
        public static final double FULL_SPEED = 1.0;//We send this power to the servos when we want them to stop.
    }

    public static final class Launcher {
        public static final double CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
        public static final double CLOSE_MIN_VELOCITY = 1175;//minimum required to start a shot for close goal.
        public static final double FAR_TARGET_VELOCITY = 1450; //Target velocity for far goal
        public static final double FAR_MIN_VELOCITY = 1325;//minimum required to start a shot for far goal
        public static final double AUTO_TARGET_VELOCITY = 1100; //in ticks/second for the close goal.
        public static final double AUTO_MIN_VELOCITY = 1075;//minimum required to start a shot for close goal.
        public static final double TIME_BETWEEN_SHOTS = 2.0;
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
        public static final double LEFT_POSITION = 0.2962;
        public static final double RIGHT_POSITION = 0.0;
        public static final double CENTER_POSITION = 0.145;
    }

    public static final class Intake {
        public static final double INTAKE_SPEED = 1.0;
        public static final double OUTTAKE_SPEED = -1.0;
    }
}
