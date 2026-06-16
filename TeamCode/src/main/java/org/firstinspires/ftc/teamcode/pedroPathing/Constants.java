package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
     //       .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .forwardZeroPowerAcceleration(-52.3)
            .lateralZeroPowerAcceleration(-56)
            .useSecondaryDrivePIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.035,0,0.005,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1,0,0.02,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.005,0, 0.03))
            .centripetalScaling(25);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .xVelocity(64)
            .yVelocity(49)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(7.869652065828841E-4) // 7.869652065828841
            .strafeTicksToInches(7.87057549380334E-4) // 7.87057549380334
            .turnTicksToInches(-7.711195988258549E-4) // 7.711195988258549
            .leftPodY(7.5)
            .rightPodY(-7.5)
            .strafePodX(-1.5)
            .leftEncoder_HardwareMapName("intake")
            .rightEncoder_HardwareMapName("leftBack")
            .strafeEncoder_HardwareMapName("leftFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE);
            //.IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
