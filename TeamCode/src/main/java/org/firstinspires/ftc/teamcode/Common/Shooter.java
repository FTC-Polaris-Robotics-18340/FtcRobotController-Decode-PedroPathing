package org.firstinspires.ftc.teamcode.Common;

import static org.firstinspires.ftc.teamcode.Common.Constants.ShooterCloseVelocity;
import static org.firstinspires.ftc.teamcode.Common.Constants.ShooterFarVelocity;
import static org.firstinspires.ftc.teamcode.Common.Constants.Shooter_P;
import static org.firstinspires.ftc.teamcode.Common.Constants.Shooter_F;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter { // 6000 RPM
    private final DcMotorEx shooter;
    private double speed;
    private double lastRequestedVelocity = 0.0;
    private PIDFCoefficients lastPidf = null;

    public Shooter(DcMotorEx s){
        shooter = s;
        initShooter();
    }

    public Shooter(HardwareMap hardwareMap){
        // bind to the shooter motor using the configured name "Sp"
        shooter = hardwareMap.get(DcMotorEx.class, "Sp");
        initShooter();
    }

    private void initShooter(){
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Shooter_P, 0, 0, Shooter_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public double getSpeed(){
        return speed;
    }

    public void spoolToClose(){
        shooter.setVelocity(ShooterCloseVelocity);
    }

    public void spoolToFar(){
        shooter.setVelocity(ShooterFarVelocity);
    }

    /**
     * Map a normalized power [0..1] to a velocity using ShooterFarVelocity as max
     * and apply it while in RUN_USING_ENCODER mode.
     */
    public void setVelocityFromPower(double power) {
        if (power <= 0) {
            stop();
            return;
        }
        try {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double vel = Math.max(0.0, Math.min(1.0, power)) * ShooterFarVelocity;
            lastRequestedVelocity = vel;
            shooter.setVelocity(vel);
        } catch (Exception e) {
            // fallback: set raw power if velocity control isn't supported
            try {
                shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooter.setPower(power);
            } catch (Exception ignored) {}
        }
    }

    public double getLastRequestedVelocity() {
        return lastRequestedVelocity;
    }

    public String getPIDFCoefficientsString() {
        try {
            PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            lastPidf = c;
            return String.format("P=%.3f I=%.3f D=%.3f F=%.3f", c.p, c.i, c.d, c.f);
        } catch (Exception e) {
            return "N/A";
        }
    }

    /**
     * Apply new PIDF coefficients at runtime for RUN_USING_ENCODER.
     */
    public void setPIDF(double p, double i, double d, double f) {
        try {
            PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
            lastPidf = coeffs;
        } catch (Exception ignored) {}
    }

    public PIDFCoefficients getLastPidf() {
        return lastPidf;
    }

    // allow direct power control and stop
    public void setPower(double power){
        // ensure raw power control uses RUN_WITHOUT_ENCODER so motor isn't constrained
        try {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception ignored) {}
        shooter.setPower(power);
    }

    public void stop(){
        shooter.setPower(0);
    }

    // expose current power and velocity for telemetry/debugging
    public double getPower() {
        return shooter.getPower();
    }

    public double getVelocity() {
        try {
            return shooter.getVelocity();
        } catch (Exception e) {
            return 0;
        }
    }

    public int getPosition() {
        try {
            return shooter.getCurrentPosition();
        } catch (Exception e) {
            return Integer.MIN_VALUE;
        }
    }

    public String getRunMode() {
        try {
            return shooter.getMode().toString();
        } catch (Exception e) {
            return "N/A";
        }
    }

    /**
     * Force the motor to RUN_WITHOUT_ENCODER and set raw power.
     * Returns the run mode after attempting the change.
     */
    public String forceRunWithoutEncoderAndPower(double power) {
        try {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(power);
            return shooter.getMode().toString();
        } catch (Exception e) {
            return "ERR";
        }
    }
}
