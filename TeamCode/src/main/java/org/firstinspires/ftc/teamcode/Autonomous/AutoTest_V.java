package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Example Auto 2")
public class AutoTest_V extends OpMode {
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(48, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(48, 32, Math.toRadians(135));

    private final Pose align1 = new Pose (48, 84, Math.toRadians(180));

    private final Pose pick1 = new Pose(18, 84, Math.toRadians(180));

    private Path scorePreload;
    private PathChain path1, path2, path3, path4;
    public void buildPaths() {
        //follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, align1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), align1.getHeading())
                .build();
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(align1, pick1))
                .setLinearHeadingInterpolation(align1.getHeading(), pick1.getHeading())
                .build();
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(pick1, scorePose))
                .setLinearHeadingInterpolation(pick1.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                pathTimer.resetTimer();
                break;

            case 1:
                while (pathTimer.getElapsedTimeSeconds() < 3.0){}
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(path2, true);

                    setPathState(2);
                }

                break;
            case 2:
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() { follower.update();
        autonomousPathUpdate();
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
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
