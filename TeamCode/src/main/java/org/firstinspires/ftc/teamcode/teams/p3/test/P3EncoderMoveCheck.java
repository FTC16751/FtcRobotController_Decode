package org.firstinspires.ftc.teamcode.teams.p3.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.test.EncoderMoveCheck;
import org.firstinspires.ftc.teamcode.teams.p3.P3Bot3Config;

/** Encoder move checks and calibration measurements for the P3 chassis. Buttons are in EncoderMoveCheck. */
@TeleOp(name = "P3 Encoder Move Check", group = "P3 Test")
public class P3EncoderMoveCheck extends EncoderMoveCheck {
    @Override
    protected RobotConfig robotConfig() {
        return P3Bot3Config.create();
    }
}
