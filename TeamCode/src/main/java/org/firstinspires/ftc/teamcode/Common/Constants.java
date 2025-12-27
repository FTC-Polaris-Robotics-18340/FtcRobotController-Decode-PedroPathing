package org.firstinspires.ftc.teamcode.Common;

public class Constants {

    // Shooter Constants
    // Tuned PIDF for shooter velocity control (start conservative, iterate as needed)
    public final static double Shooter_P = 1.0;
    // F often needs to be non-zero and proportional to max velocity; adjust on robot
    public final static double Shooter_F = 15.0;
    public final static double ShooterCloseVelocity = 900;
    public final static double ShooterFarVelocity = 1500;

    // Turret Constants
    public final static double Turret_kP = 0.02;

}
