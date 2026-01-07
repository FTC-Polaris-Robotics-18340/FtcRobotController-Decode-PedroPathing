package org.firstinspires.ftc.teamcode.TeleOp.LimelightTestNewRobot;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LLNewV1")
public class LLNewV1 extends OpMode {

    /* =========================
       HARDWARE
       ========================= */
    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private IMU imu;

    /* =========================
       PD CONTROL (TUNE THESE)
       ========================= */
    private static final double KP = 0.9;
    private static final double KD = 0.015;

    /* =========================
       MOTION LIMITS
       ========================= */
    private static final double MAX_POWER = 0.6;
    private static final double MIN_POWER = 0.02;
    private static final double LOCK_ERROR = 0.015;

    /* =========================
       FILTERING
       ========================= */
    private static final double TX_FILTER_ALPHA = 0.25;

    /* =========================
       DISTANCE CALC (CHANGE!)
       ========================= */
    private static final double LIMELIGHT_HEIGHT = 10.0; // inches
    private static final double TAG_HEIGHT = 24.0;       // inches
    private static final double LIMELIGHT_ANGLE = 20.0;  // degrees

    /* ========================= */

    private double filteredTx = 0.0;
    private double lastYawError = 0.0;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        telemetry.addLine("Motor Turret Tracking Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Update Limelight with robot yaw
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            /* =========================
               FILTER TX
               ========================= */
            double rawTx = -result.getTx(); // invert if turret moves wrong way
            filteredTx += TX_FILTER_ALPHA * (rawTx - filteredTx);

            /* =========================
               ERROR (TARGET = 0)
               ========================= */
            double yawError = filteredTx / 90.0;

            /* =========================
               DEADZONE (NO JITTER)
               ========================= */
            if (Math.abs(yawError) < LOCK_ERROR) {
                turretMotor.setPower(0);
                lastYawError = 0;
            } else {

                /* =========================
                   PD CONTROL
                   ========================= */
                double derivative = yawError - lastYawError;
                lastYawError = yawError;

                double power = (KP * yawError) + (KD * derivative);

                /* =========================
                   SOFT SPEED LIMIT
                   ========================= */
                double speedLimit = Math.max(
                        MIN_POWER,
                        Math.min(MAX_POWER, Math.abs(yawError) * MAX_POWER * 1.4)
                );

                power = clamp(power, -speedLimit, speedLimit);
                turretMotor.setPower(power);
            }

            /* =========================
               DISTANCE CALC
               ========================= */
            double ty = result.getTy();
            double distance =
                    (TAG_HEIGHT - LIMELIGHT_HEIGHT) /
                            Math.tan(Math.toRadians(LIMELIGHT_ANGLE + ty));

            /* =========================
               TELEMETRY
               ========================= */
            telemetry.addData("Raw Tx", rawTx);
            telemetry.addData("Filtered Tx", filteredTx);
            telemetry.addData("Yaw Error", yawError);
            telemetry.addData("Turret Power", turretMotor.getPower());
            telemetry.addData("Ty", ty);
            telemetry.addData("Distance (in)", distance);

        } else {
            turretMotor.setPower(0);
            telemetry.addLine("No AprilTag Detected");
        }

        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}