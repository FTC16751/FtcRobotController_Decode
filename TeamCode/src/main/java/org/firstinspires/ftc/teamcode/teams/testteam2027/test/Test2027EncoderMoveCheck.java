package org.firstinspires.ftc.teamcode.teams.testteam2027.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.test.EncoderMoveCheck;
import org.firstinspires.ftc.teamcode.teams.testteam2027.Test2027BotConfig;

/**
 * Encoder move checks and calibration measurements for test2027bot. The buttons and what to
 * measure are documented in common/test/EncoderMoveCheck and doc/ROBOT_TEST_PLAN.md section F.
 */
@TeleOp(name = "Test2027: Encoder Move Check", group = "TestTeam2027 Test")
public class Test2027EncoderMoveCheck extends EncoderMoveCheck {
    @Override
    protected RobotConfig robotConfig() {
        return Test2027BotConfig.create();
    }
}
