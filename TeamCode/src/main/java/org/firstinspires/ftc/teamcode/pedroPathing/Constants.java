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
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .forwardZeroPowerAcceleration(-29.0d)
            .lateralZeroPowerAcceleration(-50.0d)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.5d, 0.0d, 0.0d, 0.0d))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3d, 0.0d, 0.03d, 0.03d))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5d, 0.0d, 0.03d, 0.0d))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.0d, 0.0d, 0.05d, 0.03d))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025d, 0.0d, 1.0E-5d, 0.6d, 0.01d))
            .headingPIDFSwitch(0.3d)
            .centripetalScaling(0.0004d);
    public static PathConstraints PATH_CONSTRAINTS = new PathConstraints(0.99d, 100.0d, 1.0d, 1.0d);
    public static PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants().distanceUnit(DistanceUnit.MM).forwardPodY(130.0d).strafePodX(213.0d).hardwareMapName("odo1").encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD).forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD).strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants DRIVE_CONSTANTS = new MecanumConstants()
            .maxPower(1.0d)
            .xVelocity(64.3d)
            .yVelocity(52.3d)
            .rightFrontMotorName("rightFront")
            .leftFrontMotorName("leftFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FOLLOWER_CONSTANTS, hardwareMap)
                .pathConstraints(PATH_CONSTRAINTS)
                .pinpointLocalizer(LOCALIZER_CONSTANTS)
                .mecanumDrivetrain(DRIVE_CONSTANTS).build();
    }
}