package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "sampleTeleAutoHood")
public class sampleTele extends OpMode {

    // Flywheel motor
    private DcMotorEx flywheelMotor;

    // Hood servo
    private Servo hood;

    // Limelight
    private Limelight3A limelight;

    // Flywheel settings
    private final double highVelocity = 1500;
    private final double lowVelocity = 900;
    private double curTargetVelocity = highVelocity;

    // Fixed PIDF values
    private final double F = 17.9430;
    private final double P = 286.1;

    // Hood control settings (LLV6-style)
    private static final double HOOD_KP = 0.6;
    private static final double MAX_HOOD_STEP = 0.04; // max change per loop
    private static final double HOOD_MIN = 0.25;
    private static final double HOOD_MAX = 0.75;

    private double hoodPosition = 0.5; // start in middle

    @Override
    public void init() {
        // Flywheel motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Sp");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        // Hood servo
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(hoodPosition);

        // Limelight
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
        // --- Flywheel Controls ---
        if (gamepad1.yWasPressed()) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }
        flywheelMotor.setVelocity(curTargetVelocity);

        // --- Automatic Hood Controls (LLV6-style) ---
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = -result.getTy(); // vertical angle to target
            double hoodError = ty / 90.0; // scale error
            double hoodDelta = clamp(HOOD_KP * hoodError, -MAX_HOOD_STEP, MAX_HOOD_STEP);

            hoodPosition = clamp(hoodPosition + hoodDelta, HOOD_MIN, HOOD_MAX);
            hood.setPosition(hoodPosition);

            // --- Telemetry ---
            telemetry.addLine("AprilTag Detected: YES");
            telemetry.addData("Hood Error", "%.4f", hoodError);
            telemetry.addData("Hood Delta", "%.4f", hoodDelta);
            telemetry.addData("Hood Position", "%.4f", hoodPosition);
            telemetry.addData("Ty", ty);
        } else {
            telemetry.addLine("AprilTag Detected: NO");
        }

        telemetry.addData("Flywheel Target", curTargetVelocity);
        telemetry.update();
    }

    // --- Utility ---
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

