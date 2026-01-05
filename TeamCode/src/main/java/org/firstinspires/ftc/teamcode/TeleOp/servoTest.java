package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="servoTest", group = "Examples")
public class servoTest extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class,"kicker" );
        // start servos in a safe (retracted) position
        servo.setPosition(0.15);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_right){
                servo.setPosition(0.4);
            }
            if(gamepad1.dpad_left){
                servo.setPosition(0.15);
            }

            telemetry.addData("Kicker pos", servo.getPosition());
            telemetry.update();
        }
    }
}
