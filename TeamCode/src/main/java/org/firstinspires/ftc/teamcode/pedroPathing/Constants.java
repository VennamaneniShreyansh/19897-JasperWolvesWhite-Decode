package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//public class Constants {
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(12.5)
//            .forwardZeroPowerAcceleration(-40.195562029)
//            .lateralZeroPowerAcceleration(-55.88776139)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.00067, 0.04))
//            .headingPIDFCoefficients(new PIDFCoefficients(.67, 0, .042, .001))
////            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.035, 0, 0.00001, 0.6, 0.01))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00195, 0.6, 0.01))
//            .centripetalScaling(0.0003)
//            ;
//
//    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 3);
//
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            .rightFrontMotorName("fr")
//            .rightRearMotorName("br")
//            .leftRearMotorName("bl")
//            .leftFrontMotorName("fl")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(75.2125)
//            .yVelocity(60.0761)
//
//            ;
//
//    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
//            .forwardEncoder_HardwareMapName("fl")
//            .strafeEncoder_HardwareMapName("fr")
//            .forwardEncoderDirection(Encoder.FORWARD)
//            .strafeEncoderDirection(Encoder.FORWARD)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                            RevHubOrientationOnRobot.UsbFacingDirection.UP
//                    )
//            )
//            .strafePodX(-4.18)
//            .forwardPodY(-6.49)
//            .forwardTicksToInches(.0020382)
//            .strafeTicksToInches(.0020336)
//            ;
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .twoWheelLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .build();
//    }
//}


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.15)
            .forwardZeroPowerAcceleration(-36.7169274)
            .lateralZeroPowerAcceleration(-65.93005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.048, 0, 0.00067, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(0.677, 0, 0.0001, .001))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.035, 0, 0.00195, 0.6, 0.01))
            .centripetalScaling(0.0003)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.6, 3);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // --, ++, +-, -+
            .strafePodX(-5.6525)
            .forwardPodY(4.18)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(81.77397)
            .yVelocity(65.598865);
}