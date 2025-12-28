package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range; // This might already be in use for clamping


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Lock On April Tag")
public class TurretLockOnTag extends OpMode {

    private Limelight3A limelight;

    private IMU imu;

    private DcMotor YawMotor;
    private DcMotor PitchMotor;

    //trying shoot servo
    private Servo shootServo;
    private AnalogInput shootServoFeedback;

    //trying yaw stuff but w servo
    private Servo yawServo;
    private AnalogInput yawServoFeedback;

    // PID state variables
    private double lastErrorYaw = 0;
    private double integralSumYaw = 0;
    private double lastErrorPitch = 0;
    private double integralSumPitch = 0;
    private long lastTime = 0;

    // PID constants - Tune these!
    private final double kP = 0.02;  // Yaw Motor kP
    private final double kI = 0.000;
    private final double kD = 0.000;
    private final double kPy = 0.02; // Pitch Motor kP

    private final double DEADZONE = 0.5;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        YawMotor = hardwareMap.get(DcMotor.class, "Yaw");
        YawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        YawMotor.setPower(0);

        PitchMotor = hardwareMap.get(DcMotor.class, "Pitch");
        PitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PitchMotor.setPower(0);

        // Initialize the Axon Yaw Servo and Feedback
        yawServo = hardwareMap.get(Servo.class, "YawServo"); // Name in config
        yawServoFeedback = hardwareMap.get(AnalogInput.class, "YawServoFeedback"); // Name in config

        //shoot servo stuff
        shootServo = hardwareMap.get(Servo.class, "ShootServo"); // Name in config
        shootServoFeedback = hardwareMap.get(AnalogInput.class, "ShootServoFeedback"); // Name in config
    }

    @Override
    public void start(){
        super.start();
        limelight.start();
        lastTime = System.nanoTime();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Bot Pose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());

            double tx = llResult.getTx(); // tx is also the new error this loop

            double ty = llResult.getTy(); // ty is also the new error this loop

            long currentTime = System.nanoTime();
            //works for all PIDs
            double dt = (currentTime - lastTime) / 1e9;
            lastTime = currentTime;

            // Yaw PID Calculation
            integralSumYaw += tx * dt;
            double derivativeYaw = (tx - lastErrorYaw) / dt;
            lastErrorYaw = tx;

            double output = (kP * tx) + (kI * integralSumYaw) + (kD * derivativeYaw);
            output = Range.clip(output, -1, 1);

            double outputY = (kPy * ty);
            outputY = Range.clip(outputY, -1, 1); // Use Range.clip for clean code

            // --- SHOOTER SERVO LOGIC ---
            // If we have a valid result, run the shooter logic (which should be handled separately from yaw/pitch movement)
            // The previous code runs this regardless of whether 'tx' or 'ty' is in range.

            // --- SHOOTER SERVO LOGIC (Direct Position) ---
            // 1. Calculate your desired angle (Degrees) based on ty/distance math
            double desiredAngle = angleFromDist();

            // 2. Convert degrees to 0.0 - 1.0 using your helper method
            double finalPositionCommand = convertDegreesToPosition(desiredAngle);

            // 3. Send command directly to the servo
            shootServo.setPosition(Range.clip(finalPositionCommand, 0.0, 1.0));
            // end of shooter stuff---------------------------------------


            // --- YAW SERVO LOGIC ---
            // Only apply power/movement if the error is greater than the deadzone
            if (Math.abs(tx) > DEADZONE) {
                // Map tx directly to a servo position offset
                // 0.5 is center, 0.01 is the 'sensitivity' (Tune this!)
                double targetYawPos = 0.5 + (tx * 0.02);
                yawServo.setPosition(Range.clip(targetYawPos, 0.0, 1.0));

                YawMotor.setPower(output); // Keep motor for torque
            } else {
                YawMotor.setPower(0);
                integralSumYaw = 0; // RESET Integral when in deadzone to prevent "shivering"
            }

            if (Math.abs(ty) > DEADZONE) {
                integralSumPitch += ty * dt; // Track vertical error
                PitchMotor.setPower(outputY);
            } else {
                PitchMotor.setPower(0);
                integralSumPitch = 0; // Fix: Use the correct variable name
            }


            telemetry.addData("PID Error in Yaw", tx);
            telemetry.addData("PID Output in Yaw", output);

            telemetry.addData("PID Error in Pitch", ty);
            telemetry.addData("PID Output in Pitch", outputY);

        } else {
            // TARGET LOST: Stop everything for safety
            YawMotor.setPower(0);
            PitchMotor.setPower(0);
            // Optional: shootServo.setPosition(0.5); // Reset shooter to idle
            integralSumYaw = 0;
            integralSumPitch = 0;
        }
    }

    public double convertDegreesToPosition(double deg) {
        // This calculates a position ratio (0.0 to 1.0) based on degrees
        double pos = deg / 180.0;
        return pos;
    }

    public double angFromDist(double dist, boolean hi) {
        //find quadratic eq. constants from quad regression given inputs & outputs
        //coeffs for high velocity
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        if (hi) {
            a = 67.0;
            b = 67.0;
            c = 67.0;
        } else {
            a = 41.0;
            b = 41.0;
            c = 41.0;
        }

        double ang = a * dist * dist + b * dist + c;
        return ang;
    }
}