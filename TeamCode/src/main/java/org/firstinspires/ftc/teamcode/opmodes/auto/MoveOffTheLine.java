package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class MoveOffTheLine extends LinearOpMode {
    private DcMotorEx transfer, leftFront, rightFront, leftBack, rightBack, intake, shooter1, shooter2;
    private IMU imu;

    private ElapsedTime imuTimeout = new ElapsedTime();

    DcMotor.Direction LF_DIR = DcMotor.Direction.FORWARD,
                      RF_DIR = DcMotor.Direction.REVERSE,
                      LR_DIR = DcMotor.Direction.FORWARD,
                      RR_DIR = DcMotor.Direction.REVERSE;

    @Override
    public void runOpMode() {
        // Initialize odometry
        //odometry1 = hardwareMap.get(GoBildaPinpointDriver.class,"odo1");

        // Initialize wheels
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set directions
        leftFront.setDirection(LF_DIR);
        rightFront.setDirection(RF_DIR);
        leftBack.setDirection(LR_DIR);
        rightBack.setDirection(RR_DIR);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "slave1");
        imu = hardwareMap.get(IMU.class, "imu");
        // Initialize IMU directly
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        imuTimeout.reset();

        telemetry.addLine("Initialized - waiting for IMU calibration...");
        telemetry.update();
        transfer.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(20, 0, 0, 0));
        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(20, 0,0, 0));
        transfer.setTargetPosition(0);
        intake.setTargetPosition(0);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //odometry1.setOffsets(-84.0, -168.0, DistanceUnit.MM); // T U N E   T H E S E
        //odometry1.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        waitForStart();
        imuTimeout.reset();

        while (opModeIsActive()) {
            leftFront.setPower(-1);
            rightFront.setPower(-1);
            leftBack.setPower(-1);
            rightBack.setPower(-1);
            if (imuTimeout.seconds() > 0.2) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                return;
            }
        }
    }
}
