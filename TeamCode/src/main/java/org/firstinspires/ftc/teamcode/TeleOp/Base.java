package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="baseOp", group = "Examples")
public class Base extends LinearOpMode {
    private DcMotor Intake;
    private DcMotor Spinner;
    private Servo Kicker;
    private DcMotor Shooter;

    private Servo Hood;

    @Override
    public void runOpMode(){
        Intake = hardwareMap.get(DcMotor.class, "FL");
        Spinner = hardwareMap.get(DcMotor.class, "FR");
        Kicker = hardwareMap.get(Servo.class,"kicker" );
        Shooter = hardwareMap.get(DcMotor.class, "BL");
        Hood = hardwareMap.get(Servo.class, "hood");
        // start servos in a safe (retracted) position
        Kicker.setPosition(0.0);
        Hood.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.right_bumper){
                Intake.setPower(1.0);
            } else {
                Intake.setPower(0.0);
            }
            if(gamepad1.left_bumper){
                Shooter.setPower(1.0);
            } else {
                Shooter.setPower(0.0);
            }
            if (gamepad1.dpad_right) {
                Kicker.setPosition(1.0);
            }
            if(gamepad1.dpad_left){
                Kicker.setPosition(0.0);
                //Helper.setPosition(0.0);
            }

            if (gamepad2.dpad_left) {
                Spinner.setPower(0.2);
            }else if(gamepad2.dpad_right){
                Spinner.setPower(-0.2);
            }else{
                Spinner.setPower(0.0);
            }

            if(gamepad2.dpad_down){
                Hood.setPosition(0.9);
            }
            if(gamepad2.dpad_up){
                Hood.setPosition(0.0);
            }


            telemetry.addData("Motor Power", Intake.getPower());
            telemetry.addData("Outtake Power", Spinner.getPower());
            telemetry.addData("Kicker pos", Kicker.getPosition());
            telemetry.addData("Helper pos", Shooter.getPower());
            telemetry.addData("Hood pos", Hood.getPosition());
            telemetry.update();
        }
    }
}
