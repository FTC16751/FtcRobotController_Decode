package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;

public class Drivetrain {
    private FilteredPIDFController xPid, yPid;
    private PIDFController headingPid;
    private Follower follower;
    private Gamepad gamepad1;

    public Pose position;

    public Pose drivePower;

    public Drivetrain(Gamepad gamepad1, Follower follower, Pose startingPose) {
        this.gamepad1 = gamepad1;
        this.follower = follower;
        follower.setStartingPose(startingPose);
        follower.update();

        headingPid = new PIDFController(followerConstants.coefficientsHeadingPIDF);
        xPid = new FilteredPIDFController(followerConstants.coefficientsDrivePIDF);
        yPid = new FilteredPIDFController(followerConstants.coefficientsDrivePIDF);

        position = Constants.startingPos;
        drivePower = new Pose();
    }

    private void orbit(double posMultiplier) {
        headingPid.updatePosition(position.getHeading());
        headingPid.setTargetPosition(calculateRobotCentricTargetHeading());
        follower.setTeleOpDrive(
                gamepad1.left_stick_x * posMultiplier,
                -gamepad1.left_stick_y * posMultiplier,
                headingPid.run(),
                false
        );

        /*  TEST Position Lock Drive
        drivePower = calculateDrive(
                        gamepad1.left_stick_x * posMultiplier,
                        gamepad1.left_stick_y * posMultiplier,
                        gamepad1.right_stick_x * headingMultiplier
                    );

        follower.setTeleOpDrive(drivePower.getX(), drivePower.getY(), drivePower.getHeading());
        */
    }

    public void runTeleOpDrive(double driveCoefficient, boolean isAutoOrienting) {
        if (!isAutoOrienting) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * driveCoefficient,
                    -gamepad1.left_stick_x * driveCoefficient,
                    -gamepad1.right_stick_x * driveCoefficient,
                    false
            );

        /*  TEST Position Lock Drive
        drivePower = calculateDrive(
                        gamepad1.left_stick_x * driveCoefficient,
                        -gamepad1.left_stick_y * driveCoefficient,
                        gamepad1.right_stick_x * driveCoefficient
                    );

        follower.setTeleOpDrive(
            calculateDrive(drivePower.getX(), drivePower.getY(), drivePower.getHeading());
        );
        */
        } else {
            orbit(driveCoefficient);
        }
    }

    public Pose calculateDrive(double xPower, double yPower, double headingPower, double posThreshold, double headingThreshold) {
        xPid.updatePosition(position.getX());
        yPid.updatePosition(position.getY());
        headingPid.updatePosition(position.getHeading());

        Pose outputPower = new Pose();

        if (xPower < posThreshold) {
            xPid.setTargetPosition(position.getX());
            outputPower = new Pose(xPid.run(), outputPower.getY(), outputPower.getHeading());
        } else {
            outputPower = new Pose(xPower, outputPower.getY(), outputPower.getHeading());
        }

        if (yPower < posThreshold) {
            yPid.setTargetPosition(position.getY());
            outputPower = new Pose(outputPower.getX(), yPid.run(), outputPower.getHeading());
        } else {
            outputPower = new Pose(outputPower.getX(), yPower, outputPower.getHeading());
        }

        if (headingPower < headingThreshold) {
            headingPid.setTargetPosition(position.getHeading());
            outputPower = new Pose(outputPower.getX(), outputPower.getY(), headingPid.run());
        } else {
            outputPower = new Pose(outputPower.getX(), outputPower.getY(), headingPower);
        }

        return outputPower;
    }

    public double calculateRobotCentricTargetHeading() {
        double adjacent = 144 - position.getX();
        double opposite = 144 - position.getY();
        return opposite / adjacent;
    }


    public void update() {
        follower.update();
        position = follower.getPose();
        xPid.updatePosition(position.getX());
        yPid.updatePosition(position.getY());
        headingPid.updatePosition(position.getHeading());
    }
}