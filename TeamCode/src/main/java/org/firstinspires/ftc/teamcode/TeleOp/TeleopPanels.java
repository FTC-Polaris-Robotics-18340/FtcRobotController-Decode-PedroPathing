package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@TeleOp(name = "sampleTeleV8_Panels")
public class TeleopPanels extends OpMode {

    // ---------------- Hardware ----------------
    private DcMotorEx flywheelMotor;
    private Servo hood;
    private Servo yawServo;
    private Limelight3A limelight;
    private IMU imu;

    // ---------------- Flywheel PIDF ----------------
    private final double F = 18.2;
    private final double P = 285;

    // ---------------- Hood control ----------------
    private static final double HOOD_KP = 0.6;
    private static final double MAX_HOOD_STEP = 0.04;
    private static final double HOOD_MIN = 0;
    private static final double HOOD_MAX = 1;
    private double hoodPosition = 0.5;

    // ---------------- Turret yaw (LLV5 VALUES) ----------------
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 1.0;

    private static final double YAW_KP = 0.9;
    private static final double YAW_KD = 0.015;

    private static final double MAX_YAW_VEL = 0.06;
    private static final double MIN_YAW_VEL = 0.0015;

    private static final double TX_FILTER_ALPHA = 0.25;
    private static final double LOCK_ERROR = 0.015;

    private double filteredTx = 0.0;
    private double lastYawError = 0.0;

    // ---------------- Camera + target ----------------
    private static final double CAMERA_HEIGHT = 0.3048;
    private static final double CAMERA_ANGLE = Math.toRadians(28.2);
    private static final double TARGET_HEIGHT = 0.7493;

    @Override
    public void init() {

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Sp");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(hoodPosition);

        yawServo = hardwareMap.get(Servo.class, "YawServo");
        yawServo.setPosition(YAW_CENTER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addLine("Init complete");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // ---------------- Panels live values ----------------
        double targetRPM = org.firstinspires.ftc.teamcode.TeleOp.ConstantsForPanels.PANEL_RPM;
        double targetHood = org.firstinspires.ftc.teamcode.TeleOp.ConstantsForPanels.PANEL_HOOD_POS;

        // ---------------- Flywheel ----------------
        flywheelMotor.setVelocity(targetRPM);

        // ---------------- Hood ----------------
        double hoodError = targetHood - hoodPosition;
        double hoodDelta = clamp(HOOD_KP * hoodError,
                -MAX_HOOD_STEP, MAX_HOOD_STEP);
        hoodPosition = clamp(hoodPosition + hoodDelta,
                HOOD_MIN, HOOD_MAX);
        hood.setPosition(hoodPosition);

        // ---------------- Turret + Limelight ----------------
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double rawTx = -result.getTx();
            filteredTx += TX_FILTER_ALPHA * (rawTx - filteredTx);

            double yawError = filteredTx / 90.0;
            if (Math.abs(yawError) < LOCK_ERROR) {
                yawError = 0;
                lastYawError = 0;
            }

            double derivative = yawError - lastYawError;
            lastYawError = yawError;

            double yawVelocity =
                    (YAW_KP * yawError) + (YAW_KD * derivative);

            double speedLimit = Math.max(
                    MIN_YAW_VEL,
                    Math.min(MAX_YAW_VEL,
                            Math.abs(yawError) * MAX_YAW_VEL * 1.4)
            );

            yawVelocity = clamp(yawVelocity,
                    -speedLimit, speedLimit);

            double yawPos =
                    yawServo.getPosition() + yawVelocity;

            yawServo.setPosition(
                    clamp(yawPos, YAW_MIN, YAW_MAX)
            );

            telemetry.addData("RPM", targetRPM);
            telemetry.addData("Hood Pos", hoodPosition);
            telemetry.addData("Yaw Pos", yawPos);
            telemetry.addData("Tx Raw", rawTx);
            telemetry.addData("Tx Filtered", filteredTx);


        } else {
            telemetry.addLine("AprilTag: NO");
            flywheelMotor.setVelocity(0);
        }

        telemetry.update();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}