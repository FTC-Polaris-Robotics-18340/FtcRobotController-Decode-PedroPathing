package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous (name = "practice auto")
public class AutoTest2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;



    public enum PathState {
        //START POSIITON
        //Drive > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_ALIGNPOS,

        DRIVE_ALIGNPOS_PICK1

    }

    PathState pathState;

    private final Pose startPose = new Pose(56, 9, Math.toRadians(90));//48, 8, 90
    private final Pose shootPose = new Pose(56, 100, Math.toRadians(135)); //48, 96, 135

    private final Pose alignPose = new Pose(56, 78, Math.toRadians(183));//183

    private final Pose pick1Pose = new Pose(35, 78, Math.toRadians(183));//183







    private PathChain driveStartPosShootPos, driveShootPosAlignPos, driveAlignPosPick1;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosAlignPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, alignPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), alignPose.getHeading())
                .build();

        driveAlignPosPick1 = follower.pathBuilder()

                .addPath(new BezierLine(alignPose, pick1Pose))
                .setLinearHeadingInterpolation(alignPose.getHeading(), pick1Pose.getHeading())
                .setVelocityConstraint(0.2)
                .build();

    }

    public void statePathUpdate(){
        switch(pathState){

            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(driveShootPosAlignPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ALIGNPOS);
                }
                break;

            case DRIVE_SHOOTPOS_ALIGNPOS:
                if(!follower.isBusy()){
                    follower.followPath(driveAlignPosPick1, true);
                    setPathState(PathState.DRIVE_ALIGNPOS_PICK1);
                }
                break;

            case DRIVE_ALIGNPOS_PICK1:
                if(!follower.isBusy()){
                    telemetry.addLine("Reached pick1Pose");
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }


    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

    }

}
