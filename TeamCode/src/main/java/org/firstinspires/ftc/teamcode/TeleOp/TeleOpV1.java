package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
// using raw gamepad trigger values instead of GamepadKeys.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.TurretServo;

@Configurable
@TeleOp(name = "Tele_26")
public class TeleOpV1 extends LinearOpMode{
    static Robot robot;
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;

    private static double targetVelocity = 500; //CONSTANT VALUE FOR NOW, will be changed after regression
    private static double velocityTolerance = 50;
    Servo hoodServo;
    private void HardwareStart() {
        robot = new Robot(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        robot.Kicker.setPosition(0.0);
        robot.Stopper.setPosition(1.0);
        robot.Hood.setPosition(0.0);
    }
    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

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

            if (gamepad1.left_trigger > 0.1) {
                robot.Intake.setPower(1.0);
            } else {
                robot.Intake.setPower(0.0);
            }

            //Shooter spin-up
            if (gamepad1.right_trigger > 0.1) {
                robot.Shooter.setVelocity(targetVelocity);
                //robot.Stopper.setPosition(0.0);
            } else {
                robot.Shooter.setVelocity(0);
                //robot.Stopper.setPosition(1.0);
            }

            if (gamepad1.a){
                drive.driveWithMotorPowers(1.0, 0.0, 0.0, 0.0);
            }
            if (gamepad1.b){
                drive.driveWithMotorPowers(0.0, 1.0, 0.0, 0.0);
            }
            if (gamepad1.x){
                drive.driveWithMotorPowers(0.0, 0.0, 1.0, 0.0);
            }
            if (gamepad1.y){
                drive.driveWithMotorPowers(0.0, 0.0, 0.0, 1.0);
            }

            double kickPos = 0.0;
            if (gamepad1.right_bumper) {
                kickPos = 1.0;
            }
            robot.Kicker.setPosition(kickPos);

            double stopPos = 0.0;
            if (gamepad1.left_bumper){
                stopPos = 1.0;
            }
            robot.Stopper.setPosition(stopPos);

            double hoodPos = 0.0;
            if (gamepad1.dpad_down) {
                hoodPos = 1.0;
            }
            robot.Hood.setPosition(hoodPos);

            double currentVelocity = robot.Shooter.getVelocity();
            double velocityError = Math.abs(targetVelocity - currentVelocity);

            boolean shooterReady;
            if (velocityError < velocityTolerance) {
                shooterReady = true;
            } else {
                shooterReady = false;
            }

            //Kicker (only fires if shooter is ready)
            if (shooterReady) {
                robot.Kicker.setPosition(1.0); //kicks ball up
            } else {
                robot.Kicker.setPosition(0.0); //does not kick
            }





            try {
                telemetry.addData("Shooter velocity", robot.Shooter.getVelocity());
                //telemetry.addData("shooter_velocity", robot.shooter.getVelocity());
                //telemetry.addData("shooter_position", robot.shooter.getPosition());
                //telemetry.addData("shooter_runMode", robot.shooter.getRunMode());
                //telemetry.addData("shooter_reqVelocity", robot.shooter.getLastRequestedVelocity());
                //telemetry.addData("shooter_pidf", robot.shooter.getPIDFCoefficientsString());
            } catch (Exception ignored) {}




            telemetry.update();
        }
    }
}
//control hub servos:
//0: stopper
//1: hood

//expansion hub servos:
//0: kicker

