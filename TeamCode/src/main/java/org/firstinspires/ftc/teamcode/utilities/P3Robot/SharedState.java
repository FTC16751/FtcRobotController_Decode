package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import org.firstinspires.ftc.teamcode.utilities.GearGirlsRobot.GGRobotConstants;

public class SharedState {
    // This variable belongs to the CLASS and will persist between OpMode runs.
    // We initialize it to a safe default value.
    public static P3RobotConstants.Alliance alliance = P3RobotConstants.Alliance.RED;
    // This class is a utility and should not be instantiated.
    private SharedState() {}
}
