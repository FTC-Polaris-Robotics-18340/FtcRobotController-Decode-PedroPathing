package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "practice auto extended")
public class AutoTest3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;



    public enum PathState {
        //START POSIITON
        //Drive > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOTPOS,


        DRIVE_SHOOTPOS_ALIGN1,

        DRIVE_ALIGN1_PICK1,

        DRIVE_PICK1_SHOOTPOS,

        DRIVE_SHOOTPOS_ALIGN2,

        DRIVE_ALIGN2_PICK2,

        DRIVE_PICK2_SHOOTPOS,

        DRIVE_SHOOTPOS_ALIGN3,

        DRIVE_ALIGN3_PICK3,

        DRIVE_PICK3_SHOOTPOS,


        DONE

    }

    PathState pathState;

    private final Pose startPose = new Pose(56, 9, Math.toRadians(90));

    private final Pose shootPose = new Pose(56, 135, Math.toRadians(90)); //y:100

    private final Pose alignPose = new Pose(56, 78, Math.toRadians(180));//183

    private final Pose pick1Pose = new Pose(35, 78, Math.toRadians(180));//183

    private final Pose align2pose = new Pose(48, 54, Math.toRadians(180));//y = 60

    private final Pose pick2Pose = new Pose(35, 54, Math.toRadians(180));//y = 60

    private final Pose align3pose = new Pose(48, 30, Math.toRadians(180));//y = 60

    private final Pose pick3Pose = new Pose(35, 30, Math.toRadians(180));//y = 60









    private PathChain driveStartPosShootPos,
            driveShootPosAlign1, driveAlign1Pick1, drivePick1ShootPos,
            driveShootPosAlign2, driveAlign2Pick2, drivePick2ScorePos,
            driveShootPosAlign3, driveAlign3Pick3, drivePick3ScorePos;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosAlign1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, alignPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), alignPose.getHeading())
                .build();

        driveAlign1Pick1 = follower.pathBuilder()

                .addPath(new BezierLine(alignPose, pick1Pose))
                .setLinearHeadingInterpolation(alignPose.getHeading(), pick1Pose.getHeading())
                .setVelocityConstraint(0.2)
                .build();
        drivePick1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(pick1Pose, shootPose))
                .setLinearHeadingInterpolation(pick1Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosAlign2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, align2pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), align2pose.getHeading())
                .build();

        driveAlign2Pick2 = follower.pathBuilder()
                .addPath(new BezierLine(align2pose, pick2Pose))
                .setLinearHeadingInterpolation(align2pose.getHeading(), pick2Pose.getHeading())
                .build();

        drivePick2ScorePos  = follower.pathBuilder()
                .addPath(new BezierLine(pick2Pose, shootPose))
                .setLinearHeadingInterpolation(pick2Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosAlign3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, align3pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), align3pose.getHeading())
                .build();
        driveAlign3Pick3 = follower.pathBuilder()
                .addPath(new BezierLine(align3pose, pick3Pose))
                .setLinearHeadingInterpolation(align3pose.getHeading(), pick3Pose.getHeading())
                .build();
        drivePick3ScorePos = follower.pathBuilder()
                .addPath(new BezierLine(pick3Pose, shootPose))
                .setLinearHeadingInterpolation(pick3Pose.getHeading(), shootPose.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState){

            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.DRIVE_SHOOTPOS_ALIGN1);
                break;

            case DRIVE_SHOOTPOS_ALIGN1: //SHOOT_PRELOAD
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(driveShootPosAlign1, true);
                    setPathState(PathState.DRIVE_ALIGN1_PICK1);
                }
                break;

            case DRIVE_ALIGN1_PICK1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(driveAlign1Pick1, true);
                    setPathState(PathState.DRIVE_PICK1_SHOOTPOS);
                }
                break;

            case DRIVE_PICK1_SHOOTPOS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(drivePick1ShootPos, true);
                    //setPathState(PathState.DRIVE_SHOOTPOS_ALIGN2);
                    setPathState(PathState.DRIVE_SHOOTPOS_ALIGN2);
                    break;
                }




            case DRIVE_SHOOTPOS_ALIGN2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(driveShootPosAlign2, true);
                    setPathState(PathState.DRIVE_ALIGN2_PICK2);
                    break;
                }


            case DRIVE_ALIGN2_PICK2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(driveAlign2Pick2, true);
                    setPathState(PathState.DRIVE_PICK2_SHOOTPOS);
                    break;
                }
                /*



            case DRIVE_PICK2_SHOOTPOS :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(drivePick2ScorePos, true);
                    setPathState(PathState.DONE);
                    break;
                }

                 */








            case DONE:
                telemetry.addLine("DONE");


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
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
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