package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "April Tag + Turret")
public class LL_AprilTagTurret extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private Servo YawServo;
    private Servo hood;

    /* =========================
       Servo Configuration
       ========================= */

    // Servo centers (straight forward / level)
    private static final double YAW_CENTER = 0.5;
    private static final double HOOD_CENTER = 0.5;

    // Servo angular ranges in degrees (total travel)
    private static final double YAW_RANGE_DEG = 180.0;
    private static final double HOOD_RANGE_DEG = 180.0;

    // Small deadzone to avoid jitter
    private static final double DEADZONE_DEG = 0.5;

    private static final double scale = 2.0;
    //smoothing tx stuff
    private final double txConst = 0.3;
    private double smoothedTx = 0.0;

    //yawFeeedback
    private AnalogInput yawFeedback;
    //ADDED JUST NOW
    //private static final double LIMELIGHT_FOV_X_DEG = 29.8; // horizontal FOV

    @Override
    public void init() {

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Servos
        YawServo = hardwareMap.get(Servo.class, "YawServo");
        hood = hardwareMap.get(Servo.class, "hood");

        // Set initial positions
        YawServo.setPosition(YAW_CENTER);
        hood.setPosition(HOOD_CENTER);

        //yaw feedback
        yawFeedback = hardwareMap.get(AnalogInput.class, "YawServoFeedback");
    }

    @Override
    public void start() {
        super.start();
        limelight.start();
    }

    @Override
    public void loop() {

        // Update robot orientation for Limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            double tx = llResult.getTx(); // horizontal angle to tag
            double ty = llResult.getTy(); // vertical angle to tag

            // Apply deadzone to avoid jitter
            if (Math.abs(tx) < DEADZONE_DEG) tx = 0;
            if (Math.abs(ty) < DEADZONE_DEG) ty = 0;


            //smoothing tx

            if (smoothedTx == 0.0) {
                smoothedTx = tx;
            } else {
                smoothedTx = txConst * tx + (1-txConst) * smoothedTx;
            }

            /* =========================
               Move Yaw Servo
               ========================= */

            //current true position
            double currVoltage = yawFeedback.getVoltage();
            telemetry.addData("volt", currVoltage);
            double trueCurPos = currVoltage/3.3; //assuming 1.0 angle corresponds to 3.3 voltage
            telemetry.addData("True Cur Pos", trueCurPos);

            double yawOffset = -smoothedTx / YAW_RANGE_DEG /*scale*/;
            //yawOffset = (double) Math.round(yawOffset * 100);
            //yawOffset /= 100.0;
            telemetry.addData("Raw Yaw offset", yawOffset);

            double yawPos = trueCurPos + yawOffset;
            telemetry.addData("Raw Yaw Servo Pos", yawPos);

            yawPos = Math.max(0.0, Math.min(1.0, yawPos));
            telemetry.addData("Yaw Cur Pos", YawServo.getPosition());
            YawServo.setPosition(yawPos);





            /* =========================
               Move Hood Servo
               ========================= */
            /*
            double hoodOffset = ty / HOOD_RANGE_DEG;
            double hoodPos = HOOD_CENTER + hoodOffset;
            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
            hood.setPosition(hoodPos);*/

            /* =========================
               Telemetry
               ========================= */
            telemetry.addData("Tx (deg) - raw", tx);
            telemetry.addData("Tx (deg) - smoothed", smoothedTx);

            telemetry.addData("Ty (deg)", ty);
            telemetry.addData("Yaw Servo Pos", yawPos);

            //telemetry.addData("Hood Servo Pos", hoodPos);


        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();
    }
}
