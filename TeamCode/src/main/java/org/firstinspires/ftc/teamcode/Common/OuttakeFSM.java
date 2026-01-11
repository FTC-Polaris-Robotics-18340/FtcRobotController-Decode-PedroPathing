package org.firstinspires.ftc.teamcode.Common;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeFSM {

    /* =========================
       Hardware
       ========================= */
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo stopper;
    private Servo kicker;
    private DcMotorEx turret;
    private Limelight3A limelight;      // AprilTag lock

    /* =========================
       FSM
       ========================= */
    private enum FlywheelState {
        IDLE,
        TAG_LOCK,       // NEW
        SPIN_UP,
        OPEN_STOPPER_KICK,
        FINISH_SHOOTING
    }

    private FlywheelState flywheelState = FlywheelState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();

    private int shotsRemaining = 0;

    /* =========================
       Constants
       ========================= */



    private static final double GATE_OPEN = 0.6;
    private static final double GATE_CLOSED = 0.0;

    private static final double KICK_OUT = 0.7;
    private static final double KICK_IN = 0.2;

    private static final double TARGET_VELOCITY = 1500;
    private static double targetVelocity = TARGET_VELOCITY;

    /* Turret PID (from LLNewV1 style) */
    private static final double KP = 0.9;


    private double filteredTx = 0.0;
    private double ty = 0;
    private double distance = 0;
    private double lastYawError = 0.0;
    private double yawError = 0;
    private static final double TX_FILTER_ALPHA = 0.25;
    private static final double MAX_POWER = 0.6;
    private static final double MIN_POWER = 0.02;
    private static final double LOCK_ERROR = 0.015;

    private static final double LIMELIGHT_HEIGHT = 10.0; // inches
    private static final double TAG_HEIGHT = 24.0;       // inches
    private static final double LIMELIGHT_ANGLE = 20.0;  // degrees

    /* =========================
       Init
       ========================= */
    public void init(HardwareMap hwMap) {

        shooter = hwMap.get(DcMotorEx.class, "shooter");
        stopper = hwMap.get(Servo.class, "stopper");
        kicker = hwMap.get(Servo.class, "kicker");
        turret = hwMap.get(DcMotorEx.class, "turret");
        intake  = hwMap.get(DcMotorEx.class, "intake");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        stopper.setPosition(GATE_CLOSED);
        kicker.setPosition(KICK_IN);
        turret.setPower(0);
    }

    /* =========================
       Update FSM (call every loop)
       ========================= */
    public void update() {

        switch (flywheelState) {
            case IDLE:
                shooter.setPower(0);
                turret.setPower(0);
                intake.setPower(0);

                if (shotsRemaining > 0) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.TAG_LOCK;
                }
                break;

            case TAG_LOCK:
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    double rawTx = -result.getTx(); // invert if turret moves wrong way
                    filteredTx += TX_FILTER_ALPHA * (rawTx - filteredTx);
                    yawError = filteredTx / 90.0;
                    if (Math.abs(yawError) < LOCK_ERROR) {
                        turret.setPower(0);
                        lastYawError = 0;
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        double derivative = yawError - lastYawError;
                        lastYawError = yawError;

                        final double KD = 0.1;
                        double power = (KP * yawError) + (KD * derivative);

                        double speedLimit = Math.max(
                                MIN_POWER,
                                Math.min(MAX_POWER, Math.abs(yawError) * MAX_POWER * 1.4)
                        );

                        power = clamp(power, -speedLimit, speedLimit);
                        turret.setPower(power);
                    }
                    ty = result.getTy();
                    distance =
                            (TAG_HEIGHT - LIMELIGHT_HEIGHT) /
                                    Math.tan(Math.toRadians(LIMELIGHT_ANGLE + ty));

                } else {
                    turret.setPower(0);
                }

                break;

            case SPIN_UP:
                shooter.setVelocity(targetVelocity);
                shooter.setPower(1);

                if (shooter.getVelocity() >= targetVelocity - 50) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.OPEN_STOPPER_KICK;
                }
                break;

            case OPEN_STOPPER_KICK:
                intake.setPower(1);
                stopper.setPosition(GATE_OPEN);
                if (stateTimer.seconds() >= 0.2) {
                    kicker.setPosition(KICK_OUT);
                    if (stateTimer.seconds() >= 0.4) {
                        stateTimer.reset();
                        flywheelState = FlywheelState.FINISH_SHOOTING;
                    }
                }
                break;

            case FINISH_SHOOTING:
                kicker.setPosition(KICK_IN);
                stopper.setPosition(GATE_CLOSED);
                intake.setPower(0);

                shotsRemaining--;

                if (shotsRemaining > 0) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.TAG_LOCK;
                } else {
                    flywheelState = FlywheelState.IDLE;
                }
                break;
        }

        telemetry.addData("Filtered Tx", filteredTx);
        telemetry.addData("Yaw Error", yawError);
        telemetry.addData("Turret Power", turret.getPower());
        telemetry.addData("Ty", ty);
        telemetry.addData("Distance (in)", distance);
    }

    /* =========================
       Public Control
       ========================= */
    public void fireShots(int count) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = count;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }

    /* =========================
       Utility
       ========================= */
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
