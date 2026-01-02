package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "sampleTeleWithYaw")
public class sampleTele_withYaw extends OpMode {

    // Flywheel motor
    private DcMotorEx flywheelMotor;

    // Hood and turret servos
    private Servo hood;
    private Servo yawServo;

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

    // Turret yaw control settings (LLV6-style)
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 1.0;

    private static final double YAW_KP = 0.95;
    private static final double YAW_KD = 0.03;
    private static final double MAX_YAW_VEL = 0.1;
    private static final double MIN_YAW_VEL = 0.002;
    private static final double TX_FILTER_ALPHA = 0.18;
    private static final double LOCK_ERROR = 0.04;

    private double filteredTx = 0.0;
    private double lastYawError = 0.0;

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

        // Turret yaw servo
        yawServo = hardwareMap.get(Servo.class, "YawServo");
        yawServo.setPosition(YAW_CENTER);

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

        // --- Automatic Hood & Turret Controls ---
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            telemetry.addLine("AprilTag Detected: YES");

            // --- Turret Yaw (LLV6-style) ---
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

            // --- Hood ---
            double ty = -result.getTy(); // vertical angle to target
            double hoodError = ty / 90.0;
            double hoodDelta = clamp(HOOD_KP * hoodError, -MAX_HOOD_STEP, MAX_HOOD_STEP);
            hoodPosition = clamp(hoodPosition + hoodDelta, HOOD_MIN, HOOD_MAX);
            hood.setPosition(hoodPosition);

            // --- Telemetry ---
            telemetry.addData("Yaw Error", "%.4f", yawError);
            telemetry.addData("Yaw Cmd", "%.4f", yawCmd);
            telemetry.addData("Hood Error", "%.4f", hoodError);
            telemetry.addData("Hood Delta", "%.4f", hoodDelta);
            telemetry.addData("Hood Position", "%.4f", hoodPosition);
            telemetry.addData("Ty", ty);
            telemetry.addData("Tx", rawTx);

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
