package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="baseOp", group = "Examples")
public class Base extends LinearOpMode {
    private DcMotor Intake; //front 3 rows of wheels
    private DcMotor Spinner; //turret
    private DcMotor Kicker; // back wheel thing
    private DcMotor Shooter; //shooter

    private Servo Hood;
    private Servo Stopper; // servo that blocks opening of shooter


    @Override
    public void runOpMode(){
        Intake = hardwareMap.get(DcMotor.class, "FL");
        Spinner = hardwareMap.get(DcMotor.class, "FR");
        Kicker = hardwareMap.get(DcMotor.class,"BL" );
        Shooter = hardwareMap.get(DcMotor.class, "BR");
        Hood = hardwareMap.get(Servo.class, "hood");
        Stopper = hardwareMap.get(Servo.class,"stopper" );
        // start servos in a safe (retracted) position
        Hood.setPosition(0.9);
        Stopper.setPosition(0.4);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            /*if(gamepad1.right_bumper){ //hold ball
                //Intake.setPower(-1.0);
                //Kicker.setPower(-1.0);
                //Stopper.setPosition(0.15);
            } else {
                //Intake.setPower(0.0);
                //Kicker.setPower(0.0);
            }
            if(gamepad1.left_bumper) { // kick up and shoot
                //Kicker.setPower(-0.5);
                //Shooter.setPower(-1.0);
                //Stopper.setPosition(0.15);
                //Intake.setPower(-1.0);
            } else {
                //Shooter.setPower(0.0);
                //Intake.setPower(0.0);
                //Kicker.setPower(0.0);
            }*/

            // ---------------- MAIN WORKING CODE -----------------
            //bumper functionality
            //right
            Intake.setPower((gamepad1.right_bumper || gamepad1.left_bumper) ? 1.0 : 0.0);
            Kicker.setPower((gamepad1.right_bumper) ? 1.0 : 0.0);
            double stopPos = 0.15;
            if (gamepad1.right_bumper) {
                stopPos = 0.4;
            }
            Stopper.setPosition(stopPos);

            //left
            if (gamepad1.left_bumper) {
                stopPos = 0.15;
            }
            Kicker.setPower(gamepad1.left_bumper ? -0.5 : 0.0);
            Stopper.setPosition(stopPos);
            Shooter.setPower(gamepad1.left_bumper ? -1.0 : 0.0);

            //----------------------------------------------------


            //Intake.setPower(gamepad1.left_bumper  ? 1.0 : 0.0);

            //single-line intake
            //Intake.setPower(gamepad1.a  ? 1.0 : 0.0);
            //single-line stopper
            //Stopper.setPosition(gamepad1.x ? 0.4 : 0.15);
            //SL shoot
            //Shooter.setPower(gamepad1.y ? 1.0 : 0.0);
            //SL kickr
            //Kicker.setPower(gamepad1.b ? 1.0 : 0.0);
            // ----------- TROUBLESHOOTING CONTROLS -----------
            /*if (gamepad1.a) { //intake
                Intake.setPower(-1.0);
            } else {
                Intake.setPower(0.0);
            }*/
            /*if (gamepad1.b) { //kicker
                Kicker.setPower(-1.0);
            } else {
                Kicker.setPower(0.0);
            }*/
            /*if (gamepad1.x) { //stopper
                Stopper.setPosition(0.4);
            } else {
                Stopper.setPosition(0.15);
            }*/
            /*if (gamepad1.y) { //shooter
                Shooter.setPower(-1.0);
            }*/
            //----------------------------------------------

            if (gamepad2.dpad_left) { // turn turret left
                Spinner.setPower(0.3);
            }else if(gamepad2.dpad_right){ // turn turret right
                Spinner.setPower(-0.3);
            }else{
                Spinner.setPower(0.0);
            }

            if(gamepad2.dpad_up){ // hood up?
                Hood.setPosition(0.9);
            }
            if(gamepad2.dpad_down){ // hood down?
                Hood.setPosition(0.0);
            }

            //TODO: turret lock-on & shooting mechanism

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
