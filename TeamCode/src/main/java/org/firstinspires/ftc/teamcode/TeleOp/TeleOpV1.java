package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
// using raw gamepad trigger values instead of GamepadKeys.Trigger
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.TurretServo;

@Configurable
@TeleOp(name = "testTele")
public class TeleOpV1 extends LinearOpMode{
    static Robot robot;
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static TurretServo turretServo;
    Servo hoodServo;



    private void HardwareStart() {
        robot = new Robot(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);
        turretServo = new TurretServo(hardwareMap);

        hoodServo = hardwareMap.get(Servo.class, "hood");
       
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
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

            double outPower = gamepad1.left_trigger;  // left trigger controls shooter power

            robot.shooter.setVelocityFromPower(outPower);

            try {
                telemetry.addData("shooter_power", robot.shooter.getPower());
                telemetry.addData("shooter_velocity", robot.shooter.getVelocity());
                telemetry.addData("shooter_position", robot.shooter.getPosition());
                telemetry.addData("shooter_runMode", robot.shooter.getRunMode());
                telemetry.addData("shooter_reqVelocity", robot.shooter.getLastRequestedVelocity());
                telemetry.addData("shooter_pidf", robot.shooter.getPIDFCoefficientsString());
            } catch (Exception ignored) {}


            turretServo.Turret.setPosition(0.7);
            hoodServo.setPosition(0.7);


            telemetry.update();

            
        }
    }
}
