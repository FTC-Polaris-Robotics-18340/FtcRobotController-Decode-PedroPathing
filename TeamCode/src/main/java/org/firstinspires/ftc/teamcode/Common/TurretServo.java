package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretServo { // Will change this to motor later
    public final Servo Turret;
//    private final AnalogInput TurretFeedback;

    public TurretServo(Servo T, AnalogInput TF){
        Turret = T;
//        TurretFeedback = TF;
    }

    public TurretServo(HardwareMap hardwareMap){
        Turret = hardwareMap.get(Servo.class, "YawServo");
//        TurretFeedback = hardwareMap.get(AnalogInput.class, "YawServoFeedback");
    }

//    public double getPosition() {
//        // Position is given as a voltage on encoder wire, return position by dividing by voltage
//        return TurretFeedback.getVoltage() / 6.0;
//    }

    public void lockOn() {
        // TODO: Add this based on the code from TurretLockOnTag.java in other branches
    }
}
