package org.firstinspires.ftc.teamcode.TeleOp;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MainTeleop")
public class TeleOpWithPID extends LinearOpMode {

    private DcMotor Intake;
    private DcMotorEx Shooter;

    private Servo Kicker;
    private Servo Stopper;
    private Servo Hood;
    private MecanumDrive drive;
    private DcMotor FL, FR, BL, BR;

    //Shooter tuning
    private static final double TARGET_VELOCITY = 500; //1500
    private static final double VELOCITY_TOLERANCE = 50;  // allowed error

    // PIDF
    private static final double P = 286.1;
    private static final double I = 0;
    private static final double D = 0;
    private static final double F = 17.9430;

    // Servo positions
    private static final double STOPPER_BLOCK = 1.0;
    private static final double STOPPER_OPEN  = 0.0;

    private static final double KICK_REST = 0.0;
    private static final double KICK_FIRE = 1.0;
    private static final double AUTO_SPEED = 0.6;
    private static final long AUTO_INTERVAL_MS = 4000; // time per direction

    private boolean autoForward = true;
    private long lastSwitchTime = 0;


    private static final double STRAIGHT_SPEED = 1;

    @Override
    public void runOpMode() {

        // Drivetrain hardware
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");



        // Initialize MecanumDrive
        //drive = new MecanumDrive(FL, FR, BL, BR);

        Intake  = hardwareMap.get(DcMotor.class, "intake");

        Shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F)
        );

        Kicker  = hardwareMap.get(Servo.class, "kicker");
        Stopper = hardwareMap.get(Servo.class, "stopper");
        Hood    = hardwareMap.get(Servo.class, "hood");

        Stopper.setPosition(STOPPER_BLOCK);
        Kicker.setPosition(KICK_REST);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;   // forward/backward
            double strafe  = gamepad1.left_stick_x;    // left/right
            double rotate  = gamepad1.right_stick_x;   // rotation
            //drive.drive(forward, strafe, rotate);

            //Intake
            if (gamepad1.left_trigger > 0.1) {
                Intake.setPower(1.0);
            } else {
                Intake.setPower(0.0);
            }

            if (gamepad1.right_trigger > 0.1) {
                Shooter.setVelocity(TARGET_VELOCITY);
                Stopper.setPosition(STOPPER_OPEN);
            } else {
                Shooter.setVelocity(0);
                Stopper.setPosition(STOPPER_BLOCK);
            }

            //Velocity check
            double currentVelocity = Shooter.getVelocity();
            double velocityError = Math.abs(TARGET_VELOCITY - currentVelocity);

            boolean shooterReady;
            if (velocityError < VELOCITY_TOLERANCE) {
                shooterReady = true;
            } else {
                shooterReady = false;
            }

            //Kicker (only fires if shooter is ready)
            if (shooterReady) { //gamepad1.left_bumper &&
                Kicker.setPosition(KICK_FIRE);
            } else {
                Kicker.setPosition(KICK_REST);
            }


            if (gamepad1.right_bumper) {

               // drive.drive(STRAIGHT_SPEED, 0.0, 0.0);

            } else if (gamepad1.left_bumper){
                //drive.drive(-STRAIGHT_SPEED, 0.0, 0.0);
            }

            else {
                //drive.drive(forward, strafe, rotate);
            }

            //Telemetry
            telemetry.addData("Target Vel", TARGET_VELOCITY);
            telemetry.addData("Current Vel", "%.1f", currentVelocity);
            telemetry.addData("Error", "%.1f", velocityError);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.update();
        }
    }
}
