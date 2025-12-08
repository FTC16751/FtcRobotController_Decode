package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;

public class Drivetrain {

    private Follower follower;
    private Gamepad gamepad1;

    public Pose position;

    public Pose drivePower;

    public Drivetrain(Gamepad gamepad1, Follower follower, Pose startingPose) {
        this.gamepad1 = gamepad1;
        this.follower = follower;
        follower.setStartingPose(startingPose);
        follower.update();

        position = Constants.startingPos;
        drivePower = new Pose();
    }

    public void runTeleOpDrive(double driveCoefficient, boolean isAutoOrienting) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * driveCoefficient,
                    -gamepad1.left_stick_x * driveCoefficient,
                    gamepad1.right_stick_x * driveCoefficient*.75,
                    false
            );
    }

    public void runPedroTeleOpDrive(double drive, double strafe, double turn, double driveCoefficient, boolean robotCentric) {
        follower.setTeleOpDrive(
                drive * driveCoefficient,
                strafe * driveCoefficient,
                turn * driveCoefficient,
                robotCentric
        );
    }


    public void update() {
        follower.update();
        position = follower.getPose();

    }

    public void resetPose(Pose pose) {
        follower.setStartingPose(pose);
    }

}