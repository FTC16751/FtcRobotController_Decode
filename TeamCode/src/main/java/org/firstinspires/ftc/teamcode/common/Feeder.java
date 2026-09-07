package org.firstinspires.ftc.teamcode.common;

/**
 * The one robot-specific step in a launch: pushing a game piece into the spinning flywheel.
 *
 * This is the only part of shooting that differs between robots. P3 starts an indexer motor,
 * Skyline runs two feeder servos, a future robot might swing a flipper. Each team implements this
 * in a few lines and hands it to LaunchController, which owns everything else (spin-up, ready
 * check, timeout, stall abort, feed timing, cooldown, shot counting).
 *
 * start() may be called every loop while feeding, so it must be safe to call repeatedly.
 */
public interface Feeder {
    /** Begin pushing a game piece into the flywheel. Called each loop during the feed window. */
    void start();

    /** Stop feeding. Called when the feed window ends or the launch is aborted. */
    void stop();
}
