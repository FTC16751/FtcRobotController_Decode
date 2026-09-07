package org.firstinspires.ftc.teamcode.teams.skyline;

import org.firstinspires.ftc.teamcode.common.LedUtil;

/**
 * HOW THE SKYLINE ROBOT OPERATES: speeds, timings, presets, and tables that change when the game
 * or the drivers' preferences change. What the robot physically IS lives in SkylineBotConfig.
 *
 * Created 2026-09-07 for the flywheel table and aim settings that were inline in Skyline_Robot.
 * The launcher velocity presets and feed times still live in the TeleOps and autos; moving them
 * here is a good student task.
 */
public class SkylineConstants {

    private SkylineConstants() {}

    public static final class Launcher {
        /** Distance to the goal (inches) -> flywheel velocity (ticks/sec), used by FlywheelVelocityModel. */
        public static final double[][] FLYWHEEL_TABLE = {
                { 30.0, 1200.0*1.045},
                { 40.0, 1200.0*1.045},
                { 50.0, 1230.0*1.045},
                { 60.0, 1260.0*1.05},
                { 70.0, 1285.0*1.05},
                { 80.0, 1340.0*1.045},
                { 90.0, 1420.0*1.04},
                {100.0, 1460.0*1.04},
                {110.0, 1480.0*1.04},
                {120.0, 1560.0*1.04},
                {130.0, 1640.0*1.04},
                {140.0, 1720.0*1.04},
                {150.0, 1760.0},
        };
        /** Velocity to shoot at before the goal has ever been seen (the close-range table value). */
        public static final double FLYWHEEL_INITIAL_FALLBACK = 1200.0*1.045;
    }

    /** Aim feedback on the status LED (AimLed). Skyline shows orange, not yellow, for "goal is right". */
    public static final class Aim {
        public static final double LED_TOLERANCE_DEG = 2.0;
        public static final double LED_GOAL_RIGHT = LedUtil.Color.ORANGE;
        public static final double LED_GOAL_LEFT  = LedUtil.Color.BLUE;
    }
}
