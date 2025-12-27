package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Common.Shooter;

@TeleOp
public class FlywheelTuner extends OpMode {
    Shooter shooter;
    double highVelocity = 1500;
    double lowVelocity = 900;
    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    // previous button states for edge detection
    boolean prevY = false;
    boolean prevB = false;
    boolean prevDpadL = false;
    boolean prevDpadR = false;
    boolean prevDpadU = false;
    boolean prevDpadD = false;
    boolean prevA = false;
    boolean prevX = false;

    @Override
    public void init(){
        shooter = new Shooter(hardwareMap);
        telemetry.addLine ("FlywheelTuner init complete — using Shooter subsystem");
    }

    @Override
    public void loop(){
        // edge-detect buttons
        boolean curY = gamepad1.y;
        boolean curB = gamepad1.b;
        boolean curDpadL = gamepad1.dpad_left;
        boolean curDpadR = gamepad1.dpad_right;
        boolean curDpadU = gamepad1.dpad_up;
        boolean curDpadD = gamepad1.dpad_down;
        boolean curA = gamepad1.a;
        boolean curX = gamepad1.x;

        if (curY && !prevY){ // toggle high/low
            if (curTargetVelocity == highVelocity) curTargetVelocity = lowVelocity;
            else curTargetVelocity = highVelocity;
        }

        if (curB && !prevB){ // cycle step size
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (curDpadL && !prevDpadL){
            F -= stepSizes[stepIndex];
        }
        if (curDpadR && !prevDpadR){
            F += stepSizes[stepIndex];
        }
        if (curDpadU && !prevDpadU){
            P += stepSizes[stepIndex];
        }
        if (curDpadD && !prevDpadD){
            P -= stepSizes[stepIndex];
        }

        // A applies PIDF to the shooter; X stops it
        if (curA && !prevA){
            shooter.setPIDF(P, 0.0, 0.0, F);
        }
        if (curX && !prevX){
            shooter.stop();
        }

        // Always try to set target velocity (if trigger pressed, scale)
        double trigger = gamepad1.left_trigger; // use left trigger to enable/scale
        if (trigger > 0.01) {
            double scaledTarget = curTargetVelocity * trigger; // scale by trigger
            shooter.setVelocityFromPower(trigger); // easier: map normalized power to velocity
        } else {
            // if no trigger, do not force movement — user can press A to apply PIDF and then pull trigger
        }

        // telemetry
        double curVelocity = shooter.getVelocity();
        double lastReq = shooter.getLastRequestedVelocity();
        double error = lastReq - curVelocity;

        telemetry.addData("TargetVel (high/low)", "%.1f / %.1f", highVelocity, lowVelocity);
        telemetry.addData("CurTarget", "%.1f", curTargetVelocity);
        telemetry.addData("LastRequested", "%.2f", lastReq);
        telemetry.addData("CurrentVel", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("---- PIDF ----");
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Step", "%.4f (B cycles)", stepSizes[stepIndex]);
        telemetry.addData("PIDFcoeffs", shooter.getPIDFCoefficientsString());
        telemetry.addData("RunMode", shooter.getRunMode());
        telemetry.addData("Position", shooter.getPosition());

        // update previous states
        prevY = curY;
        prevB = curB;
        prevDpadL = curDpadL;
        prevDpadR = curDpadR;
        prevDpadU = curDpadU;
        prevDpadD = curDpadD;
        prevA = curA;
        prevX = curX;
    }
}
