package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous (name = "practice auto extended")
public class Auto_V1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private final Pose startPose = new Pose(63, 8, Math.toRadians(90));





    public enum PathState {
        DRIVESTARTALIGN1,
        DRIVEALIGN1PICK1, DRIVEPICK1SHOOT, DRIVESHOOTALIGN2,DRIVEALIGN2PICK2,
        DRIVEPICK2SHOOT, DRIVESHOOTALIGN3, DRIVEALIGN3PICK3,
        DRIVEPICK3SHOOT,

        DONE
        ;

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
        switch (pathState){
            case DRIVESTARTALIGN1:
                follower.followPath(driveStartToAlign1, true);
                setPathState(PathState.DRIVEALIGN1PICK1);
                break;
            case DRIVEALIGN1PICK1:
                if (!follower.isBusy() ){//&& pathTimer.getElapsedTimeSeconds() > 5
                    follower.followPath(driveAlign1ToPick1, true);
                    setPathState(PathState.DRIVEPICK1SHOOT);
                }
                break;
            case DRIVEPICK1SHOOT:
                if (!follower.isBusy() ){
                    follower.followPath(drivePick1ToShoot, true);
                    setPathState(PathState.DRIVESHOOTALIGN2);
                }
                break;
            case DRIVESHOOTALIGN2:
                if (!follower.isBusy() ){
                    follower.followPath(driveShootToAlign2, true);
                    setPathState(PathState.DRIVEALIGN2PICK2);
                }
                break;
            case DRIVEALIGN2PICK2:
                if (!follower.isBusy() ){
                    follower.followPath(driveAlign2ToPick2, true);
                    setPathState(PathState.DRIVEPICK2SHOOT);
                }
                break;
            case DRIVEPICK2SHOOT:
                if (!follower.isBusy() ){
                    follower.followPath(drivePick2ToShoot, true);
                    setPathState(PathState.DRIVESHOOTALIGN3);
                }
                break;
            case DRIVESHOOTALIGN3:
                if (!follower.isBusy() ){
                    follower.followPath(driveShootToAlign3, true);
                    setPathState(PathState.DRIVEALIGN3PICK3);
                }
                break;
            case DRIVEALIGN3PICK3:
                if (!follower.isBusy() ){
                    follower.followPath(driveAlign3ToPick3, true);
                    setPathState(PathState.DRIVEPICK3SHOOT);
                }
                break;
            case DRIVEPICK3SHOOT:
                if (!follower.isBusy() ){
                    follower.followPath(drivePick3ToShoot, true);
                    setPathState(PathState.DONE);
                }
                break;
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
        pathState = PathState.DRIVESTARTALIGN1;
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

