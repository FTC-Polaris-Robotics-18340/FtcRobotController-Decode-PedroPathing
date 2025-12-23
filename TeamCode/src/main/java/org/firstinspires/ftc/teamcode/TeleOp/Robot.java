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

    public Robot(HardwareMap hardwareMap) {
        FrontLeft = new MotorEx(hardwareMap, "FL");
        FrontRight = new MotorEx(hardwareMap, "FR");
        BackLeft = new MotorEx(hardwareMap, "BL");
        BackRight = new MotorEx(hardwareMap, "BR");

        //Invert Motors if needed
        FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}
