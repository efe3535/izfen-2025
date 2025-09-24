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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.95)
            .forwardZeroPowerAcceleration(-31.928)
            .lateralZeroPowerAcceleration(-35.7193)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.043,0,0.001,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006,0.0001,0.000005,0.6,0.015))
            .centripetalScaling(0.00055);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-167.82)
            .strafePodX(-60.635)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .rightRearMotorName("rr")
            .rightFrontMotorName("rf")
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(87.57501)
            .yVelocity(78.5477);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
