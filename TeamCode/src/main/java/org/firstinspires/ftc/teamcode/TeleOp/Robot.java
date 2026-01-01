package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Common.Intake;
import org.firstinspires.ftc.teamcode.Common.Shooter;

public class Robot {
    public MotorEx FrontLeft;
    public MotorEx FrontRight;
    public MotorEx BackLeft;
    public MotorEx BackRight;
    public MotorEx inTake;
    public MotorEx outTake;
    public Intake intake;
    public Shooter shooter;


    public Robot(HardwareMap hardwareMap) {
        FrontLeft = new MotorEx(hardwareMap, "FL");
        FrontRight = new MotorEx(hardwareMap, "FR");
        BackLeft = new MotorEx(hardwareMap, "BL");
        BackRight = new MotorEx(hardwareMap, "BR");

        // bind intake/outtake using the exact device names from your expansion hub
        // (your configuration uses "In" and "Sp")
        try {
            inTake = new MotorEx(hardwareMap, "In");
        } catch (Exception ignored) {
            inTake = null;
        }

//        try {
//            outTake = new MotorEx(hardwareMap, "Sp");
//        } catch (Exception ignored) {
//            outTake = null;
//        }

        // Prefer passing the exact DcMotorEx instances into subsystem wrappers
        try {
            DcMotorEx inDc = hardwareMap.get(DcMotorEx.class, "In");
            intake = new Intake(inDc);
        } catch (Exception ignored) {
            // fallback to hardware-map based constructor inside Intake
            intake = new Intake(hardwareMap);
        }

        try {
            DcMotorEx outDc = hardwareMap.get(DcMotorEx.class, "Sp");
            shooter = new Shooter(outDc);
        } catch (Exception ignored) {
            shooter = new Shooter(hardwareMap);
        }

        //Invert Motors if needed
        FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}