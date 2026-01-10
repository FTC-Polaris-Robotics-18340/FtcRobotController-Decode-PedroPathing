package org.firstinspires.ftc.teamcode.TeleOp.LimelightTests; //MAIN ONE LLV5

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LLV5")
public class LLV5 extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    private Servo yawServo;
    private Servo hoodServo;

    private static final double YAW_CENTER = 0.5;
    private static final double HOOD_CENTER = 0.5;

    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 1.0;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 1.0;

    /* =========================
       TUNING (IMPORTANT)
       ========================= */

    // Turret response
    private static final double YAW_KP = 0.9;      // speed
    private static final double YAW_KD = 0.015;    // damping (smoothness)

    // Motion limits
    private static final double MAX_YAW_VEL = 0.06;   // fast
    private static final double MIN_YAW_VEL = 0.0015; // smooth near target

    // Filtering
    private static final double TX_FILTER_ALPHA = 0.25; // 0.2–0.3 sweet spot

    // Stop zone (prevents jitter)
    private static final double LOCK_ERROR = 0.015;

    // Hood
    private static final double HOOD_KP = 0.35;
    private static final double MAX_HOOD_STEP = 0.04;

    /* ========================= */

    private double lastYawError = 0.0;
    private double filteredTx = 0.0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        yawServo = hardwareMap.get(Servo.class, "turret");
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

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            /* =========================
               FILTER TX (KEY FIX)
               ========================= */

            double rawTx = -result.getTx();
            filteredTx += TX_FILTER_ALPHA * (rawTx - filteredTx);

            double yawError = filteredTx / 90.0; // normalize

            /* =========================
               LOCK ZONE (NO JITTER)
               ========================= */

            if (Math.abs(yawError) < LOCK_ERROR) {
                yawError = 0;
                lastYawError = 0;
            }

            /* =========================
               PD CONTROL
               ========================= */

            double derivative = yawError - lastYawError;
            lastYawError = yawError;

            double yawVelocity = (YAW_KP * yawError) + (YAW_KD * derivative);

            /* =========================
               SOFT SPEED SCALING
               ========================= */

            double speedLimit =
                    Math.max(MIN_YAW_VEL,
                            Math.min(MAX_YAW_VEL,
                                    Math.abs(yawError) * MAX_YAW_VEL * 1.4));

            yawVelocity = clamp(yawVelocity, -speedLimit, speedLimit);

            double yawPos = yawServo.getPosition() + yawVelocity;
            yawServo.setPosition(clamp(yawPos, YAW_MIN, YAW_MAX));

            /* =========================
               HOOD (UNCHANGED, SIMPLE)
               ========================= */

            double ty = -result.getTy();
            double hoodError = ty / 90.0;

            double hoodDelta = clamp(HOOD_KP * hoodError,
                    -MAX_HOOD_STEP, MAX_HOOD_STEP);

            hoodServo.setPosition(clamp(
                    hoodServo.getPosition() + hoodDelta,
                    HOOD_MIN, HOOD_MAX));

            telemetry.addData("Raw Tx", rawTx);
            telemetry.addData("Filtered Tx", filteredTx);
            telemetry.addData("Yaw Error", yawError);
            telemetry.addData("Yaw Pos", yawPos);

        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}



