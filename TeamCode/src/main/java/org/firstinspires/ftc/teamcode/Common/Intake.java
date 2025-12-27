package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake { // 1620 RPM

    private final DcMotorEx intake;

    public Intake(DcMotorEx I) {
        intake = I;
    }

    public Intake(HardwareMap hardwareMap) {
        // bind to the intake motor using the configured name "In"
        intake = hardwareMap.get(DcMotorEx.class, "In");
    }

    public void startIntake() {
        intake.setPower(1);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    // allow setting intake power directly
    public void setPower(double power) {
        intake.setPower(power);
    }
}
