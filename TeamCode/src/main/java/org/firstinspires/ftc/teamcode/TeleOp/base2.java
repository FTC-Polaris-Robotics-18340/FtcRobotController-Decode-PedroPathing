package org.firstinspires.ftc.teamcode.TeleOp;




import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name="baseOpV2", group = "Examples")
public class base2 extends LinearOpMode {
    private DcMotor Intake; //front 3 rows of wheels
    private DcMotor Spinner; //turret
    //private DcMotor Kicker; // back wheel thing - no longer motor
    private DcMotor Shooter; //shooter

    private Servo Hood;
    private Servo Kicker;
    private Servo Stopper; // servo that blocks opening of shooter


    @Override
    public void runOpMode(){
        Intake = hardwareMap.get(DcMotor.class, "intake");
        //Spinner = hardwareMap.get(DcMotor.class, "FR");

        Shooter = hardwareMap.get(DcMotor.class, "shooter");
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Hood = hardwareMap.get(Servo.class, "hood");

        Stopper = hardwareMap.get(Servo.class,"stopper" );

        Kicker = hardwareMap.get(Servo.class, "kicker");
        //Hood.setPosition(0.9);
        //Stopper.setPosition(0.4);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        double stopPos = 1.0;
        double kickPos = 0.0; //TODO: ADJUST TO ACTUAL POS
        double shooterPower = 0;
        double intakePower = 0;

        while(opModeIsActive()){

            if (gamepad1.left_trigger > 0.1){
                intakePower = 1.0;
            }else{
                intakePower = 0.0;
            }

            if (gamepad1.right_trigger > 0.1){
                shooterPower = 1.0;
                stopPos = 0.0;
            }else{
                shooterPower = 0.0;
                stopPos = 1.0;
            }
            if (gamepad1.left_bumper) {
                kickPos = 1.0;
            }else{
                kickPos = 0.0;
            }

            Stopper.setPosition(stopPos);
            Kicker.setPosition(kickPos);
            Shooter.setPower(shooterPower);
            Intake.setPower(intakePower);


            telemetry.addData("Motor Power", Intake.getPower());
            telemetry.addData("Kicker Power", Kicker.getPosition());
            telemetry.addData("Shooter Power", Shooter.getPower());
            telemetry.addData("Hood pos", Hood.getPosition());
            telemetry.addData("Stopper pos", Stopper.getPosition());
            telemetry.update();
        }
    }
}

