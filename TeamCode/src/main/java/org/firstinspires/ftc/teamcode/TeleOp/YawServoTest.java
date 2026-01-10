package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Yaw Servo Test")
public class YawServoTest extends OpMode {

    private Servo yawServo;

    @Override
    public void init() {
        yawServo = hardwareMap.get(Servo.class, "YawServo");
        telemetry.addLine("Init complete. Ready to test.");
        telemetry.update();
        yawServo.setPosition(0.0);
    }

    @Override
    public void start() {
        telemetry.addLine("Starting test...");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Move to full left
        yawServo.setPosition(0.0);
        telemetry.addData("Yaw Servo Pos", 0.0);
        telemetry.update();
        sleep(5000); // pause for 1 second

        // Move to full right
        yawServo.setPosition(1.0);
        telemetry.addData("Yaw Servo Pos", 1.0);
        telemetry.update();
        sleep(5000); // pause for 1 second

        // Move to center
        yawServo.setPosition(0.5);
        telemetry.addData("Yaw Servo Pos", 0.5);
        telemetry.update();
        sleep(5000); // pause for 1 second

        telemetry.addLine("Test complete. Check if servo moved left, right, center.");
        telemetry.update();

        // Stop the OpMode
        requestOpModeStop();
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
