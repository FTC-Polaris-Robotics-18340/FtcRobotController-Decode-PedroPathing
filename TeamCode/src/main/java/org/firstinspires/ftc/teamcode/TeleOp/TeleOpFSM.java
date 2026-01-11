package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.OuttakeFSM;

@Configurable
@TeleOp(name = "TeleFSM")
public class TeleOpFSM extends LinearOpMode{
    static Robot robot;
    static MecanumDrive drive;
    private OuttakeFSM outtakeFSM = new OuttakeFSM();
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;

    private static double targetVelocity = 1500; //CONSTANT VALUE FOR NOW, will be changed after regression
    private static double velocityTolerance = 50;
    Servo hoodServo;
    private void HardwareStart() {
        robot = new Robot(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        robot.Kicker.setPosition(0.0);
        robot.Stopper.setPosition(1.0);
        robot.Hood.setPosition(0.4);

        outtakeFSM.init(hardwareMap, telemetry);
    }
    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

        while(opModeIsActive()){
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );

            outtakeFSM.update();

            if (gamepad2.left_trigger > 0.1) {
                robot.Intake.setPower(1.0);
            } else {
                robot.Intake.setPower(0.0);
            }

            if (gamepad2.right_trigger > 0.1) {
                outtakeFSM.fireShots(1);
            }

            try {
                telemetry.addData("Shooter velocity", robot.Shooter.getVelocity());
                telemetry.addData("Outtake FSM Busy", outtakeFSM.isBusy());
                telemetry.addData("State", outtakeFSM.getState());
            } catch (Exception ignored) {}

            telemetry.update();
        }
    }
}