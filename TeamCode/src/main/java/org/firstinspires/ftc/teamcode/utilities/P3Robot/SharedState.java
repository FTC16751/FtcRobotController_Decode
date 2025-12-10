package org.firstinspires.ftc.teamcode.utilities.P3Robot;

import org.firstinspires.ftc.teamcode.utilities.Common.CommonConstants;

public class SharedState {
    // This variable belongs to the CLASS and will persist between OpMode runs.
    // We initialize it to a safe default value.
    public static CommonConstants.Alliance alliance = CommonConstants.Alliance.RED;
    // This class is a utility and should not be instantiated.
    private SharedState() {}
}
