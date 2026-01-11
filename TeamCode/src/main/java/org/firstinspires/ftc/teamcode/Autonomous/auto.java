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



@Autonomous (name = "practice auto extended")
public class auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private final Pose startPose = new Pose(63, 8, Math.toRadians(90));





    public enum PathState {
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










    private PathChain driveStartToAlign1,
            driveAlign1ToPick1, drivePick1ToShoot, driveShootToAlign2,driveAlign2ToPick2,
            drivePick2ToShoot, driveShootToAlign3, driveAlign3ToPick3,
            drivePick3ToShoot;

    public void buildPaths(){
        driveStartToAlign1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.000, 8.000),

                                new Pose(44.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        driveAlign1ToPick1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 84.000),

                                new Pose(23.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        drivePick1ToShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 84.000),

                                new Pose(63.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        driveShootToAlign2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.000, 95.000),

                                new Pose(44.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        driveAlign2ToPick2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 60.000),

                                new Pose(23.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        drivePick2ToShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 60.000),

                                new Pose(63.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        driveShootToAlign3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.000, 95.000),

                                new Pose(44.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        driveAlign3ToPick3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 36.000),

                                new Pose(23.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        drivePick3ToShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 35.000),

                                new Pose(63.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

    }

    public void statePathUpdate(){
        switch(pathState){

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

