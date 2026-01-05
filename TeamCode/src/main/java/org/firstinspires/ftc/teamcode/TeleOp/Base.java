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
    private DcMotor Kicker;
    private DcMotor Shooter;

    private Servo Hood;
    private Servo Stopper;


    @Override
    public void runOpMode(){
        Intake = hardwareMap.get(DcMotor.class, "FL");
        Spinner = hardwareMap.get(DcMotor.class, "FR");
        Kicker = hardwareMap.get(DcMotor.class,"BL" );
        Shooter = hardwareMap.get(DcMotor.class, "BR");
        Hood = hardwareMap.get(Servo.class, "hood");
        Stopper = hardwareMap.get(Servo.class,"stopper" );
        // start servos in a safe (retracted) position
        Hood.setPosition(0.0);
        Stopper.setPosition(0.4);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.right_bumper){
                Intake.setPower(-1.0);
                Kicker.setPower(1.0);
                Stopper.setPosition(0.4);
            } else {
                Intake.setPower(0.0);
                Kicker.setPower(0.0);
            }
            if(gamepad1.left_bumper){
                Kicker.setPower(1.0);
                Shooter.setPower(-1.0);
                Stopper.setPosition(0.15);
                Intake.setPower(-1.0);
            } else {
                Shooter.setPower(0.0);
                Intake.setPower(0.0);
                Kicker.setPower(0.0);
            }

            if (gamepad2.dpad_left) {
                Spinner.setPower(0.3);
            }else if(gamepad2.dpad_right){
                Spinner.setPower(-0.3);
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
            telemetry.addData("Spinner Power", Spinner.getPower());
            telemetry.addData("Kicker Power", Kicker.getPower());
            telemetry.addData("Shooter Power", Shooter.getPower());
            telemetry.addData("Hood pos", Hood.getPosition());
            telemetry.addData("Stopper pos", Stopper.getPosition());
            telemetry.update();
        }
    }
}
