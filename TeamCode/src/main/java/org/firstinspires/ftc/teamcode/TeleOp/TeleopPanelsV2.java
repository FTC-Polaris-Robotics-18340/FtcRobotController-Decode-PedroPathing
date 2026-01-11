package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "sampleTeleV8_Panels_MotorTurret")
public class TeleopPanelsV2 extends OpMode {

    /* ========================= HARDWARE ========================= */
    private DcMotorEx flywheelMotor;
    private Servo hood;

    private DcMotorEx intake;

    // Turret hardware (FROM LLNewV1)
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private IMU imu;

    /* ========================= FLYWHEEL PIDF ========================= */
    private final double F = 17.9430; //19.4
    private final double P = 286.1 //285
            ;

    /* ========================= HOOD CONTROL ========================= */
    private static final double HOOD_KP = 0.6;
    private static final double MAX_HOOD_STEP = 0.04;
    private static final double HOOD_MIN = 0;
    private static final double HOOD_MAX = 1;
    private double hoodPosition = 0.5;

    /* ========================= TURRET PD (FROM LLNewV1) ========================= */
    private static final double KP = 0.9;
    private static final double KD = 0.015;

    private static final double MAX_POWER = 0.6;
    private static final double MIN_POWER = 0.02;
    private static final double LOCK_ERROR = 0.015;

    private static final double TX_FILTER_ALPHA = 0.25;

    /* ========================= DISTANCE CALC ========================= */
    private static final double LIMELIGHT_HEIGHT = 11.25; // inches
    private static final double TAG_HEIGHT = 29.5;        // inches
    private static final double LIMELIGHT_ANGLE = 7.4;    // degrees

    /* ========================= STATE ========================= */
    private double filteredTx = 0.0;
    private double lastYawError = 0.0;

    @Override
    public void init() {

        /* -------- Flywheel -------- */
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );


        /* -------- Hood -------- */
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(hoodPosition);

        /* -------- Turret Motor (NEW) -------- */
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        /* -------- Limelight -------- */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline

        /* -------- IMU -------- */
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        telemetry.addLine("Panels TeleOp (Motor Turret) Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        intake.setPower(1.0);


        /* ========================= TARGET VALUES ========================= */
        double targetRPM =
                org.firstinspires.ftc.teamcode.TeleOp.ConstantsForPanels.PANEL_RPM;
        double targetHood =
                org.firstinspires.ftc.teamcode.TeleOp.ConstantsForPanels.PANEL_HOOD_POS;

        /* ========================= FLYWHEEL ========================= */
        flywheelMotor.setVelocity(targetRPM);

        /* ========================= HOOD ========================= */
        double hoodError = targetHood - hoodPosition;
        double hoodDelta = clamp(HOOD_KP * hoodError, -MAX_HOOD_STEP, MAX_HOOD_STEP);
        hoodPosition = clamp(hoodPosition + hoodDelta, HOOD_MIN, HOOD_MAX);
        hood.setPosition(hoodPosition);

        /* ========================= LIMELIGHT + TURRET ========================= */
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            /* -------- TX FILTER -------- */
            double rawTx = -result.getTx();
            filteredTx += TX_FILTER_ALPHA * (rawTx - filteredTx);

            /* -------- ERROR -------- */
            double yawError = filteredTx / 90.0;

            if (Math.abs(yawError) < LOCK_ERROR) {
                turretMotor.setPower(0);
                lastYawError = 0;
            } else {
                double derivative = yawError - lastYawError;
                lastYawError = yawError;

                double power = (KP * yawError) + (KD * derivative);

                double speedLimit = Math.max(
                        MIN_POWER,
                        Math.min(MAX_POWER, Math.abs(yawError) * MAX_POWER * 1.4)
                );

                power = clamp(power, -speedLimit, speedLimit);
                turretMotor.setPower(power);
            }

            /* -------- DISTANCE -------- */
            double ty = result.getTy();
            double distance =
                    (TAG_HEIGHT - LIMELIGHT_HEIGHT) /
                            Math.tan(Math.toRadians(LIMELIGHT_ANGLE + ty));

            telemetry.addData("RPM", targetRPM);
            telemetry.addData("Hood Pos", hoodPosition);
            telemetry.addData("Turret Power", turretMotor.getPower());
            telemetry.addData("Tx Raw", rawTx);
            telemetry.addData("Tx Filtered", filteredTx);
            telemetry.addData("Distance (in)", distance);
            telemetry.addData("Flywheel Velocity", flywheelMotor.getVelocity());

        } else {
            turretMotor.setPower(0);
            telemetry.addLine("AprilTag: NO");
        }

        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
