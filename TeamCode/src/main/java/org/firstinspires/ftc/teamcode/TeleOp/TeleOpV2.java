package org.firstinspires.ftc.teamcode.TeleOp;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
@TeleOp(name = "Tele26_withTurret")
public class TeleOpV2 extends LinearOpMode {

    static Robot robot;
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;

    /* =========================
       SHOOTER
       ========================= */
    private static double targetVelocity = 1300;
    private static double velocityTolerance = 50;

    /* =========================
       LIMELIGHT + TURRET
       ========================= */
    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private IMU imu;

    private static final double KP = 0.9;
    private static final double KD = 0.015;

    private static final double MAX_POWER = 0.6;
    private static final double MIN_POWER = 0.02;
    private static final double LOCK_ERROR = 0.015;

    private static final double TX_FILTER_ALPHA = 0.25;

    private static final double LIMELIGHT_HEIGHT = 11.25;
    private static final double TAG_HEIGHT = 29.5;
    private static final double LIMELIGHT_ANGLE = 7.4;

    private double filteredTx = 0.0;
    private double lastYawError = 0.0;

    private void HardwareStart() {
        robot = new Robot(hardwareMap);
        drive = new MecanumDrive(
                robot.FrontLeft,
                robot.FrontRight,
                robot.BackLeft,
                robot.BackRight
        );

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        robot.Kicker.setPosition(0.0);
        robot.Stopper.setPosition(1.0);
        robot.Hood.setPosition(0.4);

        /* ===== LIMELIGHT INIT ===== */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu.initialize(new IMU.Parameters(orientation));


    }

    @Override
    public void runOpMode() {
        HardwareStart();
        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            /* =========================
               DRIVE
               ========================= */
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );

            /* =========================
               INTAKE
               ========================= */
            if (gamepad2.left_trigger > 0.1) {
                robot.Intake.setPower(1.0);
            } else {
                robot.Intake.setPower(0.0);
            }

            /* =========================
               SHOOTER
               ========================= */
            if (gamepad2.right_trigger > 0.1) {
                robot.Shooter.setVelocity(targetVelocity);
                robot.Intake.setPower(1.0);
                robot.Stopper.setPosition(0.0);
            } else {
                robot.Shooter.setVelocity(0);
                robot.Stopper.setPosition(1.0);
            }

            /* =========================
               HOOD
               ========================= */
            if (gamepad1.dpad_down) {
                robot.Hood.setPosition(1.0);
            } else {
                robot.Hood.setPosition(0.0);
            }

            /* =========================
               SHOOTER READY
               ========================= */
            double currentVelocity = robot.Shooter.getVelocity();
            double velocityError =
                    Math.abs(targetVelocity - currentVelocity);

            boolean shooterReady =
                    velocityError < velocityTolerance;

            if (shooterReady) {
                robot.Kicker.setPosition(1.0);
            } else {
                robot.Kicker.setPosition(0.0);
            }

            /* =========================
               TURRET APRILTAG LOCK
               ========================= */
            YawPitchRollAngles angles =
                    imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(angles.getYaw());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double rawTx = -result.getTx();
                filteredTx +=
                        TX_FILTER_ALPHA * (rawTx - filteredTx);

                double yawError = filteredTx / 90.0;

                if (Math.abs(yawError) < LOCK_ERROR) {
                    turretMotor.setPower(0);
                    lastYawError = 0;
                } else {
                    double derivative =
                            yawError - lastYawError;
                    lastYawError = yawError;

                    double power =
                            (KP * yawError) +
                                    (KD * derivative);

                    double speedLimit = Math.max(
                            MIN_POWER,
                            Math.min(
                                    MAX_POWER,
                                    Math.abs(yawError)
                                            * MAX_POWER * 1.4
                            )
                    );

                    turretMotor.setPower(
                            clamp(power,
                                    -speedLimit,
                                    speedLimit)
                    );
                }

                double ty = result.getTy();
                double distance =
                        (TAG_HEIGHT - LIMELIGHT_HEIGHT) /
                                Math.tan(
                                        Math.toRadians(
                                                LIMELIGHT_ANGLE + ty
                                        )
                                );

                telemetry.addData("Turret Tx", filteredTx);
                telemetry.addData("Distance (in)", distance);

            } else {
                turretMotor.setPower(0);
                telemetry.addLine("No AprilTag");
            }

            telemetry.addData(
                    "Shooter Velocity",
                    robot.Shooter.getVelocity()
            );
            telemetry.update();
        }
    }

    private double clamp(double value,
                         double min,
                         double max) {
        return Math.max(min,
                Math.min(max, value));
    }
}
