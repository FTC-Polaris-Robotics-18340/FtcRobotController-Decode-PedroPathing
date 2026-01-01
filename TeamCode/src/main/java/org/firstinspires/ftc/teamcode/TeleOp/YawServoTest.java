package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Yaw Servo Test")
public class YawServoTest extends OpMode {

    private Servo hood;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        telemetry.addLine("Init complete. Ready to test.");
        telemetry.update();
        hood.setPosition(0.0);
    }

    @Override
    public void start() {
        telemetry.addLine("Starting test...");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Move to full left
        hood.setPosition(0.0);
        telemetry.addData("Yaw Servo Pos", 0.0);
        telemetry.update();
        sleep(5000); // pause for 1 second

        // Move to full right
        hood.setPosition(1.0);
        telemetry.addData("Yaw Servo Pos", 1.0);
        telemetry.update();
        sleep(5000); // pause for 1 second

        // Move to center
        hood.setPosition(0.5);
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
