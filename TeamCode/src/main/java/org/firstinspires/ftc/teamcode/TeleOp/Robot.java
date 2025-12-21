package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public MotorEx FrontLeft;
    public MotorEx FrontRight;
    public MotorEx BackLeft;
    public MotorEx BackRight;

    private static HardwareMap hwMapRobot;


    public void init(HardwareMap hwMap) {
        hwMapRobot = hwMap;

        FrontLeft = new MotorEx(hwMap, "FL");
        FrontRight = new MotorEx(hwMap, "FR");
        BackLeft = new MotorEx(hwMap, "BL");
        BackRight = new MotorEx(hwMap, "BR");

        //Invert Motors if needed

        FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}
