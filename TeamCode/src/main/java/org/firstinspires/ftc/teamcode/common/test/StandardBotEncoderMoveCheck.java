package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

/** EncoderMoveCheck for the StandardBot. Each team has its own copy of this three-line class. */
@TeleOp(name = "Encoder Move Check (StandardBot)", group = "Common Test")
public class StandardBotEncoderMoveCheck extends EncoderMoveCheck {
    @Override
    protected RobotConfig robotConfig() {
        return StandardBotConfig.create();
    }
}
