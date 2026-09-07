package org.firstinspires.ftc.teamcode.teams.skyline.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.test.EncoderMoveCheck;
import org.firstinspires.ftc.teamcode.teams.skyline.SkylineBotConfig;

/** Encoder move checks and calibration measurements for the Skyline chassis. Buttons are in EncoderMoveCheck. */
@TeleOp(name = "SKYLINE: Encoder Move Check", group = "Skyline Test")
public class SkylineEncoderMoveCheck extends EncoderMoveCheck {
    @Override
    protected RobotConfig robotConfig() {
        return SkylineBotConfig.create();
    }
}
