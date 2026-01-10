package org.firstinspires.ftc.teamcode.TeleOp.SampleTeleWithAxon;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name = "Outreach Tele")
public class OutreachTele extends OpMode {

    private Servo armServo;

    // Safe Axon range — tune these
    private static final double TURRET_MIN = 0;
    private static final double TURRET_MAX = 1;

    private static final double HOOD_MIN = 0;
    private static final double HOOD_MAX = 1;

    // How fast the arm moves per loop
    private static final double SPEED = 0.01;

    private double Turret_POS = 0.5;

    private double Hood_Pos = 0.5;
    private DcMotorEx flywheelMotor;
    private Servo hood;
    private Servo turret;
    public double hood_POS = 0.5;


    public double highVelocity = 1500;
    public double lowVelocity = 900;

    public double curTargetVelocity = 0;
    double F = 17.9430; // Feedforward gain to counteract constant forces like friction
    double P = 286.1;   // Proportional gain to correct error based on how far off the velocity is

    @Override
    public void init() {
        turret = hardwareMap.get(Servo.class, "YawServo");

        // Reverse if your arm starts on the other side
        hood = hardwareMap.get(Servo.class, "hood");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Sp");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setPosition(Turret_POS);
        hood.setPosition(Hood_Pos);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
    }
    @Override
    public void loop() {
        // FTC joystick: up is negative
        double inputTurret = -gamepad1.left_stick_x;

        // Deadzone to prevent jitter
        if (Math.abs(inputTurret) > 0.05) {
            Turret_POS += inputTurret * SPEED;
        }
        telemetry.addData("Turret Input", inputTurret);
        telemetry.addData("Turret Pos", Turret_POS);



        double inputHood = -gamepad1.right_stick_y;
        if (Math.abs(inputHood) > 0.05) {
            hood_POS += inputHood * SPEED;
        }
        telemetry.addData("Hood Input", inputHood);
        telemetry.addData("hood Pos", hood_POS);



        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.xWasPressed()){
            curTargetVelocity = 0;
        }


        // Clamp to safe range
        Turret_POS = Range.clip(Turret_POS, TURRET_MIN, TURRET_MAX);
        Hood_Pos = Range.clip(hood_POS, TURRET_MIN, TURRET_MAX);

        turret.setPosition(Turret_POS);
        hood.setPosition(Hood_Pos);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Command the motor to run at the current target velocity
        flywheelMotor.setVelocity(curTargetVelocity);

        telemetry.addData("hood Pos", Hood_Pos);

        telemetry.addData("Turret Position", Turret_POS);

        telemetry.update();
    }
}