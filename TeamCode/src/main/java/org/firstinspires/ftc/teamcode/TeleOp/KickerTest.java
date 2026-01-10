package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Axon Kicker Test")
public class KickerTest extends OpMode {

    private Servo kicker;

    // Tune these values
    private static final double ARM_UP = 0.3;
    private static final double ARM_DOWN = 0.1;

    @Override
    public void init() {
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setDirection(Servo.Direction.REVERSE);
        kicker.setPosition(ARM_DOWN);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            kicker.setPosition(ARM_UP);
        }

        if (gamepad1.b) {
            kicker.setPosition(ARM_DOWN);
        }

        telemetry.addData("Servo Position", kicker.getPosition());
        telemetry.update();
    }
}
