package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.utilities.Common.RobotConfig;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Pose startingPos = new Pose(9,9,0);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(-34.46)
            .lateralZeroPowerAcceleration(-64.23)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.01905, 0, 0.0035, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.03, 0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("Front_Right")
            .rightRearMotorName("Rear_Right")
            .leftRearMotorName("Rear_Left")
            .leftFrontMotorName("Front_Left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(86.71)
            .yVelocity(60.75);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(38)
            .strafePodX(-168)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }



    /**
     * Creates a fully configured Follower instance using the provided RobotConfig profile.
     * This method acts as a bridge between RobotConfig system and Pedro Pathing.
     * Use this method for all new, multi-robot compatible code.
     *
     * @param hardwareMap The OpMode's hardwareMap.
     * @param config The complete RobotConfig profile for the specific robot being used.
     * @return A configured Pedro Pathing Follower.
     */
    public static Follower createFollower(HardwareMap hardwareMap, RobotConfig config) {

        // Guard clause: If the provided config doesn't have Pedro Pathing data, we can't build a follower.
        if (config.pedroPathing == null) {
            throw new IllegalArgumentException("Cannot create Pedro Pathing Follower: The provided RobotConfig does not contain a PedroPathingConfig.");
        }

        // 1. Configure FollowerConstants from RobotConfig.pedroPathing
        FollowerConstants followerConfig = new FollowerConstants()
                .mass(config.pedroPathing.followerMass)
                .forwardZeroPowerAcceleration(config.pedroPathing.forwardZeroPowerAccel)
                .lateralZeroPowerAcceleration(config.pedroPathing.lateralZeroPowerAccel)
                .translationalPIDFCoefficients(config.pedroPathing.translationalPIDF)
                .headingPIDFCoefficients(config.pedroPathing.headingPIDF);

        // 2. Configure MecanumConstants from RobotConfig.drivetrain and RobotConfig.pedroPathing
        MecanumConstants driveConfig = new MecanumConstants()
                .maxPower(1.0)
                .rightFrontMotorName("Front_Right")
                .rightRearMotorName("Rear_Right")
                .leftRearMotorName("Rear_Left")
                .leftFrontMotorName("Front_Left")
                .leftFrontMotorDirection(config.drivetrain.leftFrontDirection)
                .leftRearMotorDirection(config.drivetrain.leftRearDirection)
                .rightFrontMotorDirection(config.drivetrain.rightFrontDirection)
                .rightRearMotorDirection(config.drivetrain.rightRearDirection)
                .xVelocity(config.pedroPathing.driveMaxVelo)
                .yVelocity(config.pedroPathing.strafeMaxVelo);

        // 3. Configure PinpointConstants from RobotConfig.odometry
        PinpointConstants localizerConfig = new PinpointConstants()
                .forwardPodY(config.odometry.pinpointOffsetY_mm)
                .strafePodX(config.odometry.pinpointOffsetX_mm)
                .distanceUnit(DistanceUnit.MM)
                .hardwareMapName("odo")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(config.odometry.pinpointXPodDirection)
                .strafeEncoderDirection(config.odometry.pinpointYPodDirection);

        // 4. Build the Follower using all the constructed parts from the RobotConfig object
        return new FollowerBuilder(followerConfig, hardwareMap)
                .pathConstraints(config.pedroPathing.pathConstraints)
                .mecanumDrivetrain(driveConfig)
                .pinpointLocalizer(localizerConfig)
                .build();
    }
}
