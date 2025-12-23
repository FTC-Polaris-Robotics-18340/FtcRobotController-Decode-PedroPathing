package org.firstinspires.ftc.teamcode.Common;

import static org.firstinspires.ftc.teamcode.Common.Constants.Turret_kP;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurretMotor { // Will change this to motor later
    private final DcMotorEx Turret;

    public TurretMotor(DcMotorEx T){
        Turret = T;
    }

    public TurretMotor(HardwareMap hardwareMap){
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
    }

    public double getPosition(){
        return Turret.getCurrentPosition();
    }

    public void lockOnRed(double tx) {
        double output = (Turret_kP * tx);
        output = Range.clip(output, -1, 1);

        Turret.setPower(output);
    }
}
