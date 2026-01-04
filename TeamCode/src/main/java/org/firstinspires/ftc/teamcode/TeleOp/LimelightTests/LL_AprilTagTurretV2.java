package org.firstinspires.ftc.teamcode.TeleOp.LimelightTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LL_V2 ")
public class LL_AprilTagTurretV2 extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private Servo yawServo;
    private Servo hoodServo;

    /* =========================
       Servo configuration
       ========================= */

    // These MUST match your physical center
    private static final double YAW_CENTER  = 0.5;
    private static final double HOOD_CENTER = 0.5;

    // Limelight field of view (official values)
    private static final double LIMELIGHT_FOV_X_DEG = 29.8;
    private static final double LIMELIGHT_FOV_Y_DEG = 24.85;

    // Deadzone to stop jitter
    private static final double DEADZONE_DEG = 1.0;

    @Override
    public void init() {

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // Servos
        yawServo  = hardwareMap.get(Servo.class, "YawServo");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        yawServo.setPosition(YAW_CENTER);
        hoodServo.setPosition(HOOD_CENTER);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Update robot orientation (important for Limelight)
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = result.getTx(); // horizontal error
            double ty = result.getTy(); // vertical error

            // Deadzone
            if (Math.abs(tx) < DEADZONE_DEG) tx = 0;
            if (Math.abs(ty) < DEADZONE_DEG) ty = 0;

            /* =========================
               YAW (FIXED DIRECTION)
               ========================= */

            // IMPORTANT: positive tx = target right
            // Servo needs to move RIGHT → ADD offset
            double yawOffset = tx / LIMELIGHT_FOV_X_DEG;
            double yawPos = YAW_CENTER + yawOffset;

            yawPos = Math.max(0.0, Math.min(1.0, yawPos));
            yawServo.setPosition(yawPos);

            /* =========================
               HOOD
               ========================= */

            double hoodOffset = -ty / LIMELIGHT_FOV_Y_DEG;
            double hoodPos = HOOD_CENTER + hoodOffset;

            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
            hoodServo.setPosition(hoodPos);

            telemetry.addData("Tx (deg)", tx);
            telemetry.addData("Yaw Pos", yawPos);
            telemetry.addData("Ty (deg)", ty);
            telemetry.addData("Hood Pos", hoodPos);

        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();
    }
}