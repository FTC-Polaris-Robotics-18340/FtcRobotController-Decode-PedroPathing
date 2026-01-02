package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimelightTest")
public class LimelightTest extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight"); // make sure this matches your hardware map
            limelight.pipelineSwitch(0); // choose pipeline 0 for AprilTags
            telemetry.addLine("Limelight object initialized");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find Limelight in hardware map!");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (limelight != null) limelight.start();
    }

    @Override
    public void loop() {
        if (limelight == null) {
            telemetry.addLine("Limelight not initialized");
            telemetry.update();
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("Result is NULL - Limelight may not be connected");
        } else if (!result.isValid()) {
            telemetry.addLine("No valid AprilTag detected");
            telemetry.addData("Raw Tx", result.getTx());
            telemetry.addData("Raw Ty", result.getTy());
            telemetry.addData("Raw Ta", result.getTa());
        } else {
            telemetry.addLine("AprilTag DETECTED!");
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta (area)", result.getTa());
        }

        telemetry.update();
    }
}
