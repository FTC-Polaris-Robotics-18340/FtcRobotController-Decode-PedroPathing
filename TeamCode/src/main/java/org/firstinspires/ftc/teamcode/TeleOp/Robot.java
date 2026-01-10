package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.Intake;
import org.firstinspires.ftc.teamcode.Common.Shooter;

public class Robot {
    public MotorEx FrontLeft;
    public MotorEx FrontRight;
    public MotorEx BackLeft;
    public MotorEx BackRight;
    public DcMotorEx Intake;
    public DcMotorEx Shooter;
    public Servo Kicker;
    public Servo Stopper;
    public Servo Hood;



    public Robot(HardwareMap hardwareMap) {

        FrontLeft = new MotorEx(hardwareMap, "FL");
        FrontRight = new MotorEx(hardwareMap, "FR");
        BackLeft = new MotorEx(hardwareMap, "BL");
        BackRight = new MotorEx(hardwareMap, "BR");

        FrontLeft.setInverted(true);
        BackLeft.setInverted(true);
        FrontRight.setInverted(true);
        BackRight.setInverted(true);

        FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        Intake  = hardwareMap.get(DcMotorEx.class, "intake");
        Kicker  = hardwareMap.get(Servo.class, "kicker");
        Stopper = hardwareMap.get(Servo.class, "stopper");
        Hood    = hardwareMap.get(Servo.class, "hood");

        Shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}