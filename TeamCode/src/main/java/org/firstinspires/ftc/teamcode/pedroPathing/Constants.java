package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
     public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.6119)
             .forwardZeroPowerAcceleration(-92.58932808172717)
             .lateralZeroPowerAcceleration(-75.23298211514792)
             .translationalPIDFCoefficients(new PIDFCoefficients(0.11, 0, 0.01, 0.028))
             .headingPIDFCoefficients(new PIDFCoefficients(0.66, 0, 0.002, 0.025))
             .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.024, 0, 0.0001, 0.6, 0.025))
             .centripetalScaling(0.0012)
             ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")//
            .rightRearMotorName("BR")//
            .leftRearMotorName("BL")//
            .leftFrontMotorName("FL")//
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)//
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)//
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)//
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)//
            .xVelocity(65.88364200141487)
            .yVelocity(52.46878700556719)

            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.25)
            .strafePodX(5.5)
            .distanceUnit(DistanceUnit.INCH)//maybe mm?
            .hardwareMapName("pinpoint")//check if in config
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)//
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)//
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);//

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            0.69,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)//
                .mecanumDrivetrain(driveConstants)//
                .build();
    }
}
