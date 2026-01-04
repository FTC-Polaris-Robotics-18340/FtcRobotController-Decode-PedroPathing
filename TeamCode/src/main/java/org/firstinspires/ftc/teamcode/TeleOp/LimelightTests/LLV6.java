package org.firstinspires.ftc.teamcode.TeleOp.LimelightTests; // MAIN ONE LLV6

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LL_V6")
public class LLV6 extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    private Servo yawServo;
    private Servo hoodServo;

    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 1.0;

    /* =========================
       TURNT CONTROL TUNING
       ========================= */
    private static final double YAW_KP = 0.1;    //1     // slightly lower for stability
    private static final double YAW_KD = 0.03;          // derivative only for large errors
    private static final double MAX_YAW_VEL = 0.1;      // fast snap far away
    private static final double MIN_YAW_VEL = 0.002;    // smooth at small errors
    private static final double TX_FILTER_ALPHA = 0.20; // stronger smoothing
    private static final double LOCK_ERROR = 0.04;  //0.03    // deadzone near zero

    /* =========================
       HOOD SETTINGS
       ========================= */
    private static final double HOOD_KP = 0.6;
    private static final double MAX_HOOD_STEP = 0.04;
    private static final double HOOD_MIN = 0.25;
    private static final double HOOD_MAX = 0.75;

    private double filteredTx = 0.0;
    private double lastYawError = 0.0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));

        yawServo = hardwareMap.get(Servo.class, "YawServo");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        yawServo.setPosition(YAW_CENTER);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            // =======================
            // Turret Yaw - Smooth & Fast
            // =======================

            double rawTx = -result.getTx();

            // EMA filter
            filteredTx += TX_FILTER_ALPHA * (rawTx - filteredTx);

            double yawError = filteredTx / 90.0;

            // Lock deadzone
            if (Math.abs(yawError) < LOCK_ERROR) {
                yawError = 0;
                lastYawError = 0;
            }

            // Derivative only for large errors
            double derivative = 0;
            if (Math.abs(yawError) > 0.1) {
                derivative = yawError - lastYawError;
            }

            lastYawError = yawError;

            // Exponential proportional scaling
            double scaledKP = YAW_KP * Math.pow(Math.abs(yawError), 0.6);
            double yawCmd = scaledKP * yawError + YAW_KD * derivative;

            // Soft speed limit
            double speedLimit = Math.max(MIN_YAW_VEL,
                    Math.min(MAX_YAW_VEL,
                            Math.pow(Math.abs(yawError), 0.7) * MAX_YAW_VEL * 1.8));

            yawCmd = clamp(yawCmd, -speedLimit, speedLimit);

            yawServo.setPosition(clamp(yawServo.getPosition() + yawCmd, YAW_MIN, YAW_MAX));

            // =======================
            // Hood - stable
            // =======================
            double ty = -result.getTy();
            double hoodDelta = clamp(HOOD_KP * (ty / 90.0), -MAX_HOOD_STEP, MAX_HOOD_STEP);
            hoodServo.setPosition(clamp(hoodServo.getPosition() + hoodDelta, HOOD_MIN, HOOD_MAX));

            telemetry.addData("Tx", rawTx);
            telemetry.addData("Filtered Tx", filteredTx);
            telemetry.addData("Yaw Error", yawError);

        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}

