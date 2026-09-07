package org.firstinspires.ftc.teamcode.common;

/**
 * State that must survive from the Autonomous OpMode into the TeleOp OpMode on the same robot.
 * Static fields live for the life of the Robot Controller app, so an auto can write them in stop()
 * and the TeleOp that follows can read them in init().
 *
 * One copy for all teams. Each robot runs its own Robot Controller, so teams never collide.
 */
public class SharedState {
    /** Alliance chosen during autonomous init; defaults to RED if no auto ran. */
    public static CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;

    private SharedState() {}
}
