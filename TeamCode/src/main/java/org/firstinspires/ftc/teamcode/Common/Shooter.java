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

    public Shooter(DcMotorEx s){
        shooter = s;
        initShooter();
    }

    public Shooter(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
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
}
