package org.firstinspires.ftc.teamcode.TeleOp.LimelightTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled

@TeleOp(name = "LLTest")
public class LLTest extends OpMode {

    private Limelight3A limelight;
    private double smoothedTa = 0.0;
    private final double alpha = 0.3; // Smoothing factor
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start(){
        super.start();
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            double rawTa = llResult.getTa();
            if (smoothedTa == 0.0) {
                smoothedTa = rawTa; // Initialize on first valid reading
            } else {
                smoothedTa = alpha * rawTa + (1 - alpha) * smoothedTa; // Exponential smoothing
            }

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("rawTa", llResult.getTa());
            telemetry.addData("smoothedTa", smoothedTa);
            telemetry.addData("Bot Pose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.addData("Distance from tag", getDistFromTag(smoothedTa));
            telemetry.update();
        }
    }

    public double getDistFromTag(double ta) {
        // eq: y = Ax^B
        double A = 66.64891;
        double B = -0.5320883;
        return (A * Math.pow(ta, B));
    }
}