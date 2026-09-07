package org.firstinspires.ftc.teamcode.teams.geargirls.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.test.EncoderMoveCheck;
import org.firstinspires.ftc.teamcode.teams.geargirls.GGBot2Config;

/** Encoder move checks and calibration measurements for the GearGirls chassis. Buttons are in EncoderMoveCheck. */
@TeleOp(name = "GG Encoder Move Check", group = "GearGirls Test")
public class GGEncoderMoveCheck extends EncoderMoveCheck {
    @Override
    protected RobotConfig robotConfig() {
        return GGBot2Config.create();
    }
}
