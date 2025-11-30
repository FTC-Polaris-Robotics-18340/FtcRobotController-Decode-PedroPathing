package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Lock On April Tag")
public class TurretLockOnTag extends OpMode {

    private Limelight3A limelight;

    private IMU imu;

    private DcMotor YawMotor;
    private DcMotor PitchMotor;

    // PID state
    private double lastError = 0;
    private double integralSum = 0;
    private long lastTime = 0;

    // PID constants (tune these!)
    private final double kP = 0.02;
    private final double kI = 0.000;
    private final double kD = 0.000;

    private final double kPy = 0.02;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        YawMotor = hardwareMap.get(DcMotor.class, "Yaw");
        YawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        YawMotor.setPower(0);

        PitchMotor = hardwareMap.get(DcMotor.class, "Pitch");
        PitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PitchMotor.setPower(0);
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
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Bot Pose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());

            double tx = llResult.getTx(); // tx is also the new error this loop

            double ty = llResult.getTy(); // ty is also the new error this loop

            long currentTime = System.nanoTime();
            double dt = (currentTime - lastTime) / 1e9;
            lastTime = currentTime;

            integralSum += tx * dt;
            double derivative = (tx - lastError) / dt;
            lastError = tx;

            double output = (kP * tx) + (kI * integralSum) + (kD * derivative);
            output = Math.max(-1, Math.min(1, output));

            double outputY = (kPy * ty);
            outputY = Math.max(-1, Math.min(1, output));

            YawMotor.setPower(output);
            PitchMotor.setPower(outputY);

            telemetry.addData("PID Error in Yaw", tx);
            telemetry.addData("PID Output in Yaw", output);

            telemetry.addData("PID Error in Pitch", ty);
            telemetry.addData("PID Output in Pitch", outputY);

        }
    }
}
