package org.firstinspires.ftc.teamcode.TeleOp.SampleTeleWithAxon;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "sampleTeleV5")
public class SampleTeleV5 extends OpMode {

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

    private static final double TX_FILTER_ALPHA = 0.15; // stronger smoothing
    private static final double LOCK_ERROR_TX = 0.3;    // ignore tiny tx changes

    private double filteredTx = 0.0;

    // ---------------- Camera + target ----------------
    private static final double CAMERA_HEIGHT = 0.3048; // meters
    private static final double CAMERA_ANGLE = Math.toRadians(28.2); // 38.7
    private static final double TARGET_HEIGHT = 0.7493; // meter

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

            // ---------------- Compute distance ----------------
            double tyRad = Math.toRadians(result.getTy());
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(CAMERA_ANGLE + tyRad);

            // ---------------- Shooter setpoint ----------------
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
            double smoothYaw = 0.1 * targetYawPos + 0.9 * yawServo.getPosition(); //10, 90
            yawServo.setPosition(clamp(smoothYaw, YAW_MIN, YAW_MAX) -0.01); //added 0.01 because it was too much to the right

            // ---------------- Telemetry ----------------
            telemetry.addData("Distance (m)", "%.2f", distance);
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

    // ---------------- Shooter setpoint table ----------------
    private ShooterSetpoint getShooterSetpoint(double distance) {
        if (distance < 2.0) return new ShooterSetpoint(0.25, 1500);//1400RPM (not enough power)11
        //0.42 too high
        else if (distance < 2.5) return new ShooterSetpoint(0.3, 1500);
        else if (distance < 3.0) return new ShooterSetpoint(0.3, 1500);
        else return new ShooterSetpoint(0.54, 1600);
    }
    //0.3
    //1500

    //0 --> Up --> goes low
    //1 --> Down --> goes high

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