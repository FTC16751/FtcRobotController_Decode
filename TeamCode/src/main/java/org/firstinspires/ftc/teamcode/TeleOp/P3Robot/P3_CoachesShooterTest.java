package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_LauncherUtil;
import org.firstinspires.ftc.teamcode.utilities.P3Robot.P3_Robot;

@TeleOp(name="P3 Teleop (Coaches shooter opmode)", group=" _P3opmodes")
public class P3_CoachesShooterTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private P3_LauncherUtil launcher;

    private double launcherVelocity = 0;


    @Override
    public void init() {
        launcher = new P3_LauncherUtil(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        handleLauncherControls();
        doTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        requestOpModeStop();
    }
    private void handleLauncherControls() {
        if (gamepad1.yWasPressed()) {
            launcherVelocity = 1200;
            launcher.setShooterMotorVelocity(launcherVelocity);
        } else if (gamepad1.aWasPressed()) {
            launcherVelocity = 0;
            launcher.setShooterMotorVelocity(launcherVelocity);
        } else if (gamepad1.xWasPressed()) {
            launcherVelocity = launcherVelocity-100;
            launcher.setShooterMotorVelocity(launcherVelocity);
        } else if (gamepad1.bWasPressed()) {
            launcherVelocity = launcherVelocity+100;
            launcher.setShooterMotorVelocity(launcherVelocity);
        }
    }
    private void doTelemetry() {
        telemetry.addData("requested velocity: ", launcherVelocity);
        telemetry.addData("launcher velocity: ", launcher.getShooterMotorVelocity());
    }
}
