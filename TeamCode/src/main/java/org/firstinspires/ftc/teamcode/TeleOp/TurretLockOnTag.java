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

    // PID state
    private double lastError = 0;
    private double lastAngleError = 0;
    private double integralSum = 0;
    private double integralSum2 = 0;
    private long lastTime = 0;

    // PID constants (tune these!)
    private final double kP = 0.02;
    private final double kI = 0.000;
    private final double kD = 0.000;

    private final double kP2 = 0.02;
    private final double kI2 = 0.000;
    private final double kD2 = 0.000;

    private final double kPy = 0.02;

    // PID state variables for Yaw axis (using your provided structure)
    private double lastErrorYaw = 0; // Tracks the previous 'tx' error
    private double integralSumYaw = 0;


    // PID w servo
    private final double kP_Yaw = 0.02; // Tune these specifically for servo movement
    private final double kI_Yaw = 0.000;
    private final double kD_Yaw = 0.001;

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

            integralSum += tx * dt;
            double derivative = (tx - lastError) / dt;
            lastError = tx;

            double output = (kP * tx) + (kI * integralSum) + (kD * derivative);
            output = Math.max(-1, Math.min(1, output));

            double outputY = (kPy * ty);
            outputY = Range.clip(outputY, -1, 1); // Use Range.clip for clean code

            // --- SHOOTER SERVO LOGIC ---
            // If we have a valid result, run the shooter logic (which should be handled separately from yaw/pitch movement)
            // The previous code runs this regardless of whether 'tx' or 'ty' is in range.

            double feedbackVoltage = shootServoFeedback.getVoltage();
            // Normalize the voltage from 0.0-3.3V to 0.0-1.0 position value
            double shootCurrentPos = feedbackVoltage / 3.3;
            // whatever linear relationship, we can find angle here
            double currAngle = shootCurrentPos; // Changed variable type

            //PID for servo of shooter--------------------
            //based on math, we find out the desired angle based on distance,
            // then just simply put it as a ratio 0 < r < 1.
            double desiredAngle = 67; //placeholder (degrees)
            double angleError = desiredAngle - currAngle;
            integralSum2 += angleError * dt;
            double derivative2 = (angleError - lastAngleError) / dt;
            lastAngleError = angleError;
            double output2 = (kP2 * angleError) + (kI2 * integralSum2) + (kD2 * derivative2);
            output2 = Range.clip(output2, -1, 1); // Use Range.clip

            // Use current position + incremental PID nudge
            double finalPositionCommand = Range.clip(shootCurrentPos + (output2 * 0.05), 0.0, 1.0);
            shootServo.setPosition(finalPositionCommand);
            // end of shooter stuff---------------------------------------


            // --- YAW SERVO LOGIC ---
            // Only apply power/movement if the error is greater than the deadzone
            if (Math.abs(tx) > DEADZONE) {
                //replace original motor turret tracking w ts servo-------------------
                // 1. Read the current position from the feedback wire first, normalizing the voltage (0-3.3V) to a 0.0-1.0 position value.
                currentServoPositionValue = yawServoFeedback.getVoltage() / 3.3; // Reusing existing variable name - ASSUMING ITS 0-3.3

                // 2. The existing 'output' variable (your yaw PID result, -1 to 1) is used as an *adjustment* to the current position.
                // Use a small constant like 0.05 to scale the adjustment so the movement is smooth.
                double finalPositionCommand2 = Range.clip(currentServoPositionValue + (output * 0.05), 0.0, 1.0);

                // 3. Send the command to the Axon Servo
                yawServo.setPosition(finalPositionCommand2);

                // If you still want the motor to run at the same time:
                YawMotor.setPower(output);

            } else {
                // Stop movement if we are in the deadzone
                YawMotor.setPower(0);
                // We do NOT stop the servo position command; it holds its last commanded position
            }


            // --- PITCH MOTOR LOGIC ---
            if (Math.abs(ty) > DEADZONE) {
                PitchMotor.setPower(outputY);
            } else {
                PitchMotor.setPower(0);
            }

            telemetry.addData("PID Error in Yaw", tx);
            telemetry.addData("PID Output in Yaw", output);

            telemetry.addData("PID Error in Pitch", ty);
            telemetry.addData("PID Output in Pitch", outputY);

        }
    }

    public double convertDegreesToPosition(double deg) {
        // This calculates a position ratio (0.0 to 1.0) based on degrees
        double pos = deg / 180.0;
        return pos;
    }
}