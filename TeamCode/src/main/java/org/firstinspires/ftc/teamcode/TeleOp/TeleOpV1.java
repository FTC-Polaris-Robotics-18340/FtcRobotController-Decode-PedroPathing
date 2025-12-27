package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
// using raw gamepad trigger values instead of GamepadKeys.Trigger
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
@TeleOp(name = "testTele")
public class TeleOpV1 extends LinearOpMode{
    static Robot robot;
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;



    private void HardwareStart() {
        robot = new Robot(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);
       
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

            // Use gamepad1 triggers (left and right) to control intake/outtake power
            double inPower = gamepad1.right_trigger; // right trigger runs intake
            double outPower = gamepad1.left_trigger;  // left trigger controls shooter power

                    // if trigger requests power but encoder/velocity shows no movement,
                    // try forcing RUN_WITHOUT_ENCODER and set raw power as a diagnostic/test.
                    boolean forcedNoEncoder = false;
                    try {
                        if (outPower > 0.05 && robot.shooter.getVelocity() == 0 && Math.abs(robot.shooter.getPosition()) < 10) {
                            String newMode = robot.shooter.forceRunWithoutEncoderAndPower(outPower);
                            telemetry.addData("forced_no_encoder_mode", newMode);
                            forcedNoEncoder = true;
                        }
                    } catch (Exception ignored) {}
                    telemetry.addData("forced_no_encoder", forcedNoEncoder);
            robot.intake.setPower(inPower);

            // Use encoder-based velocity control mapped from trigger
            robot.shooter.setVelocityFromPower(outPower);

            // still attempt FTCLib MotorEx direct set as a fallback (won't hurt)
            boolean motorExUsed = false;
            try {
                if (robot.outTake != null && outPower > 0) {
                    robot.outTake.set(outPower);
                    motorExUsed = true;
                }
            } catch (Exception ignored) {}

            // telemetry for debugging trigger -> shooter
            telemetry.addData("left_trigger", outPower);
            try {
                telemetry.addData("shooter_power", robot.shooter.getPower());
                telemetry.addData("shooter_velocity", robot.shooter.getVelocity());
                telemetry.addData("shooter_position", robot.shooter.getPosition());
                telemetry.addData("shooter_runMode", robot.shooter.getRunMode());
                telemetry.addData("shooter_reqVelocity", robot.shooter.getLastRequestedVelocity());
                telemetry.addData("shooter_pidf", robot.shooter.getPIDFCoefficientsString());
            } catch (Exception ignored) {}
            telemetry.addData("used_MotorEx_outTake", motorExUsed);
            // show last requested pidf values separately
            try {
                PIDFCoefficients last = robot.shooter.getLastPidf();
                if (last != null) telemetry.addData("pidf_vals", String.format("P=%.3f F=%.3f", last.p, last.f));
            } catch (Exception ignored) {}

            // live PIDF tuning: dpad up/down change F, Y/A change P (edge-detect)
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;

            if (dpadUp && !prevDpadUp) {
                PIDFCoefficients c = robot.shooter.getLastPidf();
                double newF = (c==null?6.0:c.f) + 1.0;
                double p = (c==null?0.2:c.p);
                robot.shooter.setPIDF(p, 0, 0, newF);
            }
            if (dpadDown && !prevDpadDown) {
                PIDFCoefficients c = robot.shooter.getLastPidf();
                double newF = Math.max(0.0, (c==null?6.0:c.f) - 1.0);
                double p = (c==null?0.2:c.p);
                robot.shooter.setPIDF(p, 0, 0, newF);
            }
            if (y && !prevY) {
                PIDFCoefficients c = robot.shooter.getLastPidf();
                double newP = (c==null?0.2:c.p) + 0.05;
                double f = (c==null?6.0:c.f);
                robot.shooter.setPIDF(newP, 0, 0, f);
            }
            if (a && !prevA) {
                PIDFCoefficients c = robot.shooter.getLastPidf();
                double newP = Math.max(0.0, (c==null?0.2:c.p) - 0.05);
                double f = (c==null?6.0:c.f);
                robot.shooter.setPIDF(newP, 0, 0, f);
            }

            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;
            prevY = y;
            prevA = a;

            telemetry.update();

            
        }
    }
}
