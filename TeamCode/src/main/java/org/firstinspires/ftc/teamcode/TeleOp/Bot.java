package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Bot {
    public MotorEx FrontLeft;
    public MotorEx FrontRight;
    public MotorEx BackLeft;
    public MotorEx BackRight;

    /*public SimpleServo Shoulder;
    public SimpleServo Elbow;
    public SimpleServo Wrist;
    public SimpleServo Claw;*/





    private static HardwareMap hwMapRobot;

    /*public void Collide(){
        OuttakeLeft.setPosition(1);
        OuttakeRight.setPosition(0);
    }*/
    public void init(HardwareMap hwMap) {
        hwMapRobot = hwMap;

        /*Shoulder = new SimpleServo(hwMap, "shoulder", 0.0, 1.0);
        Elbow = new SimpleServo(hwMap, "elbow", 0.0, 1.0);

        Wrist = new SimpleServo(hwMap,"wrist",0.0,1.0);
        Claw = new SimpleServo(hwMap,"claw",0.0,1.0 );*/

        FrontLeft = new MotorEx(hwMap, "FL", Motor.GoBILDA.RPM_312);

        FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        FrontRight = new MotorEx(hwMap, "FR", Motor.GoBILDA.RPM_312);
        FrontRight.setInverted(true);
        FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        BackLeft = new MotorEx(hwMap, "BL", Motor.GoBILDA.RPM_312);
        BackLeft.setInverted(true);
        BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        BackRight = new MotorEx(hwMap, "BR", Motor.GoBILDA.RPM_312);
        BackRight.setInverted(true);
        BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


    }
}
