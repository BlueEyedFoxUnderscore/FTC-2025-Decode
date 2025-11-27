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
            .mass(9.65)
            .forwardZeroPowerAcceleration(-29.0d)
            .lateralZeroPowerAcceleration(-50.0d)
            //TRANSLATION
            .useSecondaryTranslationalPIDF(true)
            /*far */.translationalPIDFCoefficients(new PIDFCoefficients(1d, 0.0d, 0.0d, 0.0d))
            /*near*/.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05d, 0.0d, 0.006d, 0.07d))
            .translationalPIDFSwitch(10.0d)
            //HEADING
            .useSecondaryHeadingPIDF(true)
            /* far*/.headingPIDFCoefficients(new PIDFCoefficients(2.0d, 0.0d, 0.0d, 0.0d))
            /*near*/.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.7d, 0.0d, 0.05d, 0.07d)) // d=.12 was the decel, but causes jumps from small changes
            .headingPIDFSwitch(40.0d*(Math.PI/180.0d))
            //DRIVE
            .useSecondaryDrivePIDF(true)
            /* far*/.drivePIDFCoefficients(new FilteredPIDFCoefficients(2d, 0.0d, 0d, 0.6d, 0.0d))
            ///*near*/.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.006d, 0.0d, .000005d, 0.6d, 0.03d))
            /*near*/.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.009d, 0.0d, .000005d, 0.6d, 0.03d))
            .drivePIDFSwitch(30)
            //CENTRIPETAL
            .centripetalScaling(0.0006d);
    public static PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            0.99d,
            100.0d,
            1.0d,
            1.0d);
    public static PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants()
            .distanceUnit(DistanceUnit.MM)
            .forwardPodY(130.0d)
            .strafePodX(213.0d)
            .hardwareMapName("odo1")
            //.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .customEncoderResolution(19.89436789f*72.0f/74.0f) //ticks-per-mm
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
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