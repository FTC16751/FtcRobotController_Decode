package org.firstinspires.ftc.teamcode.TeleOp.P3Robot;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.PriorSeason.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pedro Path DrivetrainTest", group="Concept")
public class PedroPatchDrivetrainTest extends OpMode{

    private Follower follower;
    double driveCoefficient = 1.0;
    private Drivetrain dt;
    private boolean isAutoOrienting;

    @Override
    public void init() {
        Pose startingPose = new Pose(9,9,0);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        dt = new Drivetrain(gamepad1, follower, startingPose);
        driveCoefficient = 0.5;
        isAutoOrienting = false;
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        dt.update();
        telemetry.update();

        dt.runTeleOpDrive(driveCoefficient, isAutoOrienting);
        
        /* AFTER normal teleOp drive is tested with above method, uncomment below to test auto-orienting.
        if (gamepad1.a) {
            isAutoOrienting = !isAutoOrienting;
        }
        */

        telemetry.addData("X", dt.position.getX());
        telemetry.addData("Y", dt.position.getY());
        telemetry.addData("Heading", dt.position.getHeading());
        telemetry.addData("", "");
        telemetry.addData("Velocity: ", follower.getVelocity());
    }
}