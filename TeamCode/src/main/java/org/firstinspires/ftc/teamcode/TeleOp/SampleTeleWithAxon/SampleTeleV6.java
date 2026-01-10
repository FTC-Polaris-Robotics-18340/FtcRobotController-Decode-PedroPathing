package org.firstinspires.ftc.teamcode.TeleOp.SampleTeleWithAxon;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "sampleTeleV6")
public class SampleTeleV6 extends OpMode {

    // ---------------- Hardware ----------------
    private DcMotorEx flywheelMotor;
    private Servo hood;
    private Servo yawServo;
    private Limelight3A limelight;

    // ---------------- Flywheel PIDF ----------------
    private final double F = 17.9430;
    private final double P = 286.1;
    private double curTargetVelocity = 0;

    // ---------------- Hood control ----------------
    private static final double HOOD_KP = 0.6;
    private static final double MAX_HOOD_STEP = 0.04;
    private static final double HOOD_MIN = 0;
    private static final double HOOD_MAX = 1;
    private double hoodPosition = 0.5;

    // ---------------- Turret yaw ----------------
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 1.0;

    private static final double TX_FILTER_ALPHA = 0.15;
    private static final double LOCK_ERROR_TX = 0.3;

    private double filteredTx = 0.0;

    // ---------------- Camera + target ----------------
    private static final double CAMERA_HEIGHT = 0.3048; // meters
    private static final double CAMERA_ANGLE = Math.toRadians(28.2); // tilt in radians
    private static final double TARGET_HEIGHT = 0.7493; // meters

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

        telemetry.addLine("Init complete");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            telemetry.addLine("AprilTag: YES");

            // ---------------- Compute M ----------------
            double tyRad = Math.toRadians(result.getTy());
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(CAMERA_ANGLE + tyRad); // your "distance value"

            // ---------------- Shooter setpoint using regression ----------------
            ShooterSetpoint sp = getShooterSetpoint(distance);

            // Flywheel
            curTargetVelocity = sp.rpm;
            flywheelMotor.setVelocity(curTargetVelocity);

            // Hood (rate-limited)
            double hoodError = sp.hoodPos - hoodPosition;
            double hoodDelta = clamp(HOOD_KP * hoodError, -MAX_HOOD_STEP, MAX_HOOD_STEP);
            hoodPosition = clamp(hoodPosition + hoodDelta, HOOD_MIN, HOOD_MAX);
            hood.setPosition(hoodPosition);

            // ---------------- Turret Yaw - ABSOLUTE ----------------
            double rawTx = -result.getTx(); // degrees left/right
            filteredTx += TX_FILTER_ALPHA * (rawTx - filteredTx);

            // Deadzone to prevent tiny oscillations
            if (Math.abs(filteredTx) < LOCK_ERROR_TX) filteredTx = 0;

            // Map to absolute servo position
            double targetYawPos = YAW_CENTER + clamp(filteredTx / 90.0, -0.5, 0.5);

            // Smooth movement
            double smoothYaw = 0.1 * targetYawPos + 0.9 * yawServo.getPosition();
            yawServo.setPosition(clamp(smoothYaw, YAW_MIN, YAW_MAX) - 0.01);

            // ---------------- Telemetry ----------------
            telemetry.addData("M Value", "%.2f", distance);
            telemetry.addData("Target RPM", "%.0f", curTargetVelocity);
            telemetry.addData("Hood Target", "%.3f", sp.hoodPos);
            telemetry.addData("Hood Pos", "%.3f", hoodPosition);
            telemetry.addData("Raw Tx", rawTx);
            telemetry.addData("Filtered Tx", filteredTx);
            telemetry.addData("Yaw Pos", smoothYaw);

        } else {
            telemetry.addLine("AprilTag: NO");
            flywheelMotor.setVelocity(0);
        }

        telemetry.update();
    }

    // ---------------- Shooter setpoint using regression ----------------
    private ShooterSetpoint getShooterSetpoint(double M) {
        double rpm = 270.75576 * M * M + 1008.18433 * M + 620.52429; // quadratic regression RPM
        double hood = -0.470467*M*M + 1.46428*M -0.525118; // example linear regression for hood, replace with your regression
        hood = clamp(hood, HOOD_MIN, HOOD_MAX);
        rpm = Math.max(0, rpm);
        return new ShooterSetpoint(hood, rpm);
    }

    // ---------------- Utility ----------------
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ---------------- Shooter helper class ----------------
    private static class ShooterSetpoint {
        double hoodPos;
        double rpm;
        ShooterSetpoint(double hoodPos, double rpm) {
            this.hoodPos = hoodPos;
            this.rpm = rpm;
        }
    }
}
