package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


/*
----------------- TASKS FOR THIS FILE --------------- || done?
    - basic driving                                   || y
    - intaking + shooting process                     || y
    - shooter calc (equations w/ power & hood pos)    || TODO
    - turret mechanism + calc (equation, ll, etc.)    || TODO
*/
@TeleOp(name="MainTeleOp")
public class TeleOpWithPID extends LinearOpMode {

    private DcMotor Intake;
    private DcMotorEx Shooter;

    private Servo Kicker;
    private Servo Stopper;
    private Servo Hood;

    // ===== Shooter tuning =====
    private static double TARGET_VELOCITY = 1500;   // TODO: FIND NUM FROM EQUATION
    private static final double VELOCITY_TOLERANCE = 50;  // allowed error

    // PIDF (from tuner)
    private static final double P = 286.1;
    private static final double I = 0;
    private static final double D = 0;
    private static final double F = 17.9430;

    // Servo positions
    private static final double STOPPER_BLOCK = 0.4;
    private static final double STOPPER_OPEN  = 0.15;

    private static final double KICK_REST = 0.0;
    private static final double KICK_FIRE = 1.0;

    // basic drive
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    /*// not working drive stuff
    public static Robot robot;
    public static MecanumDrive drive;
    public static GamepadEx gamepad1Ex;
    public static GamepadEx gamepad2Ex;
    public static TurretServo turretServo;
    //Servo hoodServo; - already defined*/

    /*private void HardwareStart() {
        robot = new Robot(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);
        turretServo = new TurretServo(hardwareMap);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
    }*/

    @Override
    public void runOpMode() {

        // ============== INTAKE & SHOOTER ===============
        Intake = hardwareMap.get(DcMotor.class, "intake");

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

        telemetry.addLine("PID Shooter Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ================ BASIC DRIVE =================
            double lX = gamepad1.left_stick_x;
            double lY = gamepad1.left_stick_y;
            double rX = gamepad1.right_stick_x;

            drive(lY, lX, rX);

            //===============================================

            // ===== Intake =====
            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) { // <- MAY CAUSE PROBLEMS
                Intake.setPower(1.0);
                Stopper.setPosition(STOPPER_OPEN);
            } else {
                Intake.setPower(0.0);
                Stopper.setPosition(STOPPER_BLOCK);
            }

            // ===== Shooter spin-up =====
            if (gamepad1.right_trigger > 0.1) {
                Shooter.setVelocity(TARGET_VELOCITY);
            } else {
                Shooter.setVelocity(0);
            }

            // ===== Velocity check =====
            double currentVelocity = Shooter.getVelocity();
            double velocityError = Math.abs(TARGET_VELOCITY - currentVelocity);

            boolean shooterReady;
            if (velocityError < VELOCITY_TOLERANCE) {
                shooterReady = true;
            } else {
                shooterReady = false;
            }

            // ===== Kicker (only fires if shooter is ready) =====
            if (gamepad1.left_bumper && shooterReady) {
                Kicker.setPosition(KICK_FIRE);
            } else {
                Kicker.setPosition(KICK_REST);
            }

            // ===== Telemetry =====
            telemetry.addData("Target Vel", TARGET_VELOCITY);
            telemetry.addData("Current Vel", "%.1f", currentVelocity);
            telemetry.addData("Error", "%.1f", velocityError);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.update();

            // =======================================================
            /*// ========= MECANUM DRIVE not working, use BasicDrive.java =========
            // previous-button states for simple edge detection
            boolean prevDpadUp = false;
            boolean prevDpadDown = false;
            boolean prevY = false;
            boolean prevA = false;

            while(opModeIsActive()){
                drive.driveRobotCentric(
                        gamepad1Ex.getLeftX(),
                        gamepad1Ex.getLeftY(),
                        gamepad1Ex.getRightX(),
                        true
                );

                //double outPower = gamepad1.left_trigger;  // left trigger controls shooter power

                //robot.shooter.setVelocityFromPower(outPower);

             */
        }
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        FL.setPower(maxSpeed * (frontLeftPower / maxPower));
        FR.setPower(maxSpeed * (frontRightPower / maxPower));
        BL.setPower(maxSpeed * (backLeftPower / maxPower));
        BR.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
