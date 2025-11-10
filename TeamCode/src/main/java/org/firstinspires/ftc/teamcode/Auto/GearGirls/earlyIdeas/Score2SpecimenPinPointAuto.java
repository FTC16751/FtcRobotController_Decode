package org.firstinspires.ftc.teamcode.Auto.GearGirls.earlyIdeas;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name="Score 2 Specimen", group="Right", preselectTeleOp = "Driver Control - Into The Deep")
@Disabled
public class Score2SpecimenPinPointAuto extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    MecanumDrive pinPointDrive;
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /* initialize the robot hardware */
        initializeHardware();

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

        MecanumDrive pinPointDrive = new MecanumDrive(hardwareMap,startPose);
        Pose2d driveToSubmersible = new Pose2d(-15,0,Math.toRadians(0));
        Pose2d driveToObservationZone_pt1 = new Pose2d(-10.4,24,Math.toRadians(180));
        Pose2d driveToObservationZone_pt2 = new Pose2d(2,24,Math.toRadians(180));

        Pose2d driveToSubmersible2 = new Pose2d(-30,-12,Math.toRadians(0));

        Pose2d park = new Pose2d(-5,29,Math.toRadians(0));

        waitForStart();


        //driveToSubmersible
        Actions.runBlocking(
                pinPointDrive.actionBuilder(startPose)
                        .strafeToLinearHeading(driveToSubmersible.position, driveToSubmersible.heading,new TranslationalVelConstraint(20))
                        .build());

//
//        //driveToObservationZone()
//        Actions.runBlocking(
//                pinPointDrive.actionBuilder(pinPointDrive.localizer.getPose())
//                        .strafeToLinearHeading(driveToObservationZone_pt1.position, driveToObservationZone_pt1.heading)
//                        .strafeToLinearHeading(driveToObservationZone_pt2.position,driveToObservationZone_pt2.heading,new TranslationalVelConstraint(20))
//                        //.lineToX(-3)
//                        //.lineToX(-5,new TranslationalVelConstraint(30))
//                        .build()
//        );
//
//
//
//        //driveToSubmersible
//        Actions.runBlocking(
//                pinPointDrive.actionBuilder(driveToObservationZone_pt1)
//                        .strafeToLinearHeading(driveToSubmersible2.position, driveToSubmersible.heading)
//                        .build());
//
//
//
//        //park();
//        initializeStartPositions();
//        Actions.runBlocking(
//                pinPointDrive.actionBuilder(driveToSubmersible)
//                        .strafeToLinearHeading(park.position, park.heading)
//                        .build()
//        );

        safeWaitSeconds(5);
    }

    private void initializeHardware() throws InterruptedException {
        /***************************************************************
         this method will initialize all the hardware used on the robot
         this includes the drive train and any other subsystems for the robot.
         the subsystems will change from year to year depending on the challenge
         the drivetrain usually stays constant year over year.
         ***************************************************************/

        Thread.sleep(250); //give enough time to initialize and set light colors
        //drive.init(hardwareMap,telemetry); //initialize the drive subsystem

        initializeStartPositions();
    }
    private void initializeStartPositions() {

    }


    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }

    }
}