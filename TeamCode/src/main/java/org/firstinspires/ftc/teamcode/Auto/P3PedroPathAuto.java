package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "P3 Pedro Path Auto", group = "Examples")
@Disabled
public class P3PedroPathAuto extends OpMode{

        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;

        private AUTO_STATE pathState;

        private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
        private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose parkPose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
        //private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
        //private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private enum AUTO_STATE {
        START,
        MOVE_TO_SCORE_POSE,
        SHOOT,
        SHOOTING,
        MOVE_TO_PARK
    }

    private Path pathToScore;

    private Path pathToPark;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        pathToScore = new Path(new BezierLine(startPose, scorePose));
        pathToScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pathToPark = new Path(new BezierLine(scorePose, parkPose));
        pathToPark.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(pathToScore);
                setPathState(AUTO_STATE.MOVE_TO_SCORE_POSE);
                break;
            case MOVE_TO_SCORE_POSE:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pathToScore,true);
                    setPathState(AUTO_STATE.SHOOT);
                }
                break;
            case SHOOT:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    // NEED TIMER STATE MACHINE IN BETWEEN SHOOTING AND MOVING
                    // START MOTORS 1, 2, AND 3 BEFORE SHOOTING (ANOTHER STATE MACHINE OR FUNCTION)

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(AUTO_STATE.MOVE_TO_PARK);
                }
                break;
            case MOVE_TO_PARK:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pathToPark,true);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(AUTO_STATE pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(AUTO_STATE.START);
    }

    /** We do not use this because everything should auto   matically disable **/
    @Override
    public void stop() {}

}
