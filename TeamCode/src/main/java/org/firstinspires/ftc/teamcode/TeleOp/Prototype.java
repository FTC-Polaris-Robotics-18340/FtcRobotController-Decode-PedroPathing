package org.firstinspires.ftc.teamcode.TeleOp;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ProtoBot")
public class Prototype extends LinearOpMode{
    static Bot robot;
    static MecanumDrive drive;

    private DcMotor Intake;

    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;

    private DcMotor Outtake;
    private Servo Kicker;
    private Servo Helper;
    private void HardwareStart(){
        robot = new Bot();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);


    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        HardwareStart();
        Intake = hardwareMap.get(DcMotor.class, "IN");
        Outtake = hardwareMap.get(DcMotor.class, "OUT");
        Kicker = hardwareMap.get(Servo.class,"kicker" );
        Helper = hardwareMap.get(Servo.class, "helper");
        // start servos in a safe (retracted) position
        Kicker.setPosition(0.0);
        Helper.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );
            if(gamepad1.right_bumper){
                Intake.setPower(1.0);
                Outtake.setPower(1.0);
            }
            else{
                Intake.setPower(0.0);
                Outtake.setPower(0.0);
            }

            if (gamepad2.dpad_right) {
                Kicker.setPosition(1.0);
            }
            if(gamepad2.dpad_left){
                Helper.setPosition(1.0);
            }
            if(gamepad2.x){
                Kicker.setPosition(0.0);
                Helper.setPosition(0.0);
            }


            telemetry.addData("Motor Power", Intake.getPower());
            telemetry.addData("Outtake Power", Outtake.getPower());
            telemetry.addData("Kicker pos", Kicker.getPosition());
            telemetry.addData("Helper pos", Helper.getPosition());
            telemetry.update();
        }
    }
    /*protected void runSample(){
        float gain = 2;
        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

    }*/

}

