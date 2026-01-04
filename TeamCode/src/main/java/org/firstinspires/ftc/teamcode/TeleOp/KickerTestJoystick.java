package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Axon Kicker Joystick")
public class KickerTestJoystick extends OpMode {

    private Servo armServo;

    // Safe Axon range — tune these
    private static final double ARM_MIN = 0;
    private static final double ARM_MAX = 1;

    // How fast the arm moves per loop
    private static final double SPEED = 0.01;

    private double armPosition = 0.5;

    @Override
    public void init() {
        armServo = hardwareMap.get(Servo.class, "kicker");

        // Reverse if your arm starts on the other side
        armServo.setDirection(Servo.Direction.REVERSE);

        armServo.setPosition(armPosition);
    }

    @Override
    public void loop() {
        // FTC joystick: up is negative
        double input = -gamepad1.left_stick_y;

        // Deadzone to prevent jitter
        if (Math.abs(input) > 0.05) {
            armPosition += input * SPEED;
        }

        // Clamp to safe range
        armPosition = Range.clip(armPosition, ARM_MIN, ARM_MAX);

        armServo.setPosition(armPosition);

        telemetry.addData("Joystick", input);
        telemetry.addData("Arm Position", armPosition);
        telemetry.update();
    }
}
