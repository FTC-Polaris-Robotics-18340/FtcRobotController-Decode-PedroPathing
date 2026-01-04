package org.firstinspires.ftc.teamcode.TeleOp.LimelightTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LLV3")
public class LLPID extends OpMode {

    /* =========================
       Hardware
       ========================= */
    private Limelight3A limelight;
    private IMU imu;
    private Servo yawServo;
    private Servo hoodServo;

    /* =========================
       Servo Configuration
       ========================= */
    private static final double YAW_CENTER = 0.5;
    private static final double HOOD_CENTER = 0.5;

    private static final double YAW_RANGE_DEG = 180.0;
    private static final double HOOD_RANGE_DEG = 180.0;

    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 1.0;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 1.0;

    /* =========================
       PID Constants (STABLE)
       ========================= */
    private static final double YAW_KP = 0.5;
    private static final double YAW_KD = 0.02;

    private static final double HOOD_KP = 0.3;

    /* =========================
       Stability Controls
       ========================= */
    private static final double ERROR_DEADBAND = 0.08;
    private static final double MAX_SERVO_STEP = 0.008;

    /* =========================
       PID State
       ========================= */
    private double lastYawError = 0.0;

    @Override
    public void init() {

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Servos
        yawServo = hardwareMap.get(Servo.class, "YawServo");
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

        /* =========================
           Update robot orientation
           ========================= */
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = -result.getTx();
            double ty = -result.getTy();

            /* =========================
               Normalize errors
               ========================= */
            double yawError = tx / (YAW_RANGE_DEG / 2.0);
            double hoodError = ty / (HOOD_RANGE_DEG / 2.0);

            /* =========================
               Deadband
               ========================= */
            if (Math.abs(yawError) < ERROR_DEADBAND) yawError = 0;
            if (Math.abs(hoodError) < ERROR_DEADBAND) hoodError = 0;

            /* =========================
               Yaw PID (P + D)
               ========================= */
            double yawDerivative = yawError - lastYawError;
            if (yawError == 0) yawDerivative = 0;
            lastYawError = yawError;

            double yawDelta =
                    (YAW_KP * yawError) +
                            (YAW_KD * yawDerivative);

            yawDelta = clamp(yawDelta, -MAX_SERVO_STEP, MAX_SERVO_STEP);

            double yawPos = yawServo.getPosition() + yawDelta;
            yawPos = clamp(yawPos, YAW_MIN, YAW_MAX);
            yawServo.setPosition(yawPos);

            /* =========================
               Hood Control (P only)
               ========================= */
            double hoodDelta = HOOD_KP * hoodError;
            hoodDelta = clamp(hoodDelta, -MAX_SERVO_STEP, MAX_SERVO_STEP);

            double hoodPos = hoodServo.getPosition() + hoodDelta;
            hoodPos = clamp(hoodPos, HOOD_MIN, HOOD_MAX);
            hoodServo.setPosition(hoodPos);

            /* =========================
               Telemetry
               ========================= */
            telemetry.addData("Tx", tx);
            telemetry.addData("Yaw Error", yawError);
            telemetry.addData("Yaw Pos", yawPos);
            telemetry.addData("Hood Pos", hoodPos);

        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();
    }

    /* =========================
       Utility
       ========================= */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}