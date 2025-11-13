package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.util.function.DoubleFunction;

@TeleOp//(name = "***FTC OpMode 2", group = "TeleOp")

public class MainControl extends OpMode {
    DcMotorEx leftFront, rightFront, leftBack, rightBack, intake, transfer;
    DcMotorEx shooter1, shooter2;
    IMU imu;
    double headingOffset = 0;
    boolean imuReady = false;
    ElapsedTime imuTimeout = new ElapsedTime();

    // Motor direction constants (adjust if wiring differs)
    DcMotor.Direction LF_DIR = DcMotor.Direction.FORWARD;
    DcMotor.Direction RF_DIR = DcMotor.Direction.REVERSE;
    DcMotor.Direction LR_DIR = DcMotor.Direction.FORWARD;
    DcMotor.Direction RR_DIR = DcMotor.Direction.REVERSE;

    //private MotorGroup shooterGroup;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

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

        imuTimeout.reset();

        telemetry.addLine("Initialized - waiting for IMU calibration...");
        telemetry.update();
        shooter1.setVelocityPIDFCoefficients(20, 0, 0, 1.0/28.0 * 6000.0/60.0*2.0*2800/1180);
        shooter2.setVelocityPIDFCoefficients(20, 0, 0, 1.0/28.0 * 6000.0/60.0*2.0*2800/1180);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double toTPS(double RPM) {
        return RPM * 28 / 60.0;
    }

    public double toRPM(double TPS) {
        return TPS * 60 / 28.0;
    }


    private double offset = 0;

    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.y) {
            offset = getRawHeading();
        }

        double botHeading = getRawHeading() - offset;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        int targetPower = gamepad1.right_bumper? gamepad1.left_bumper? 0: 100: gamepad1.left_bumper? -100: 0;

        telemetry.clear();
        intake.setPower(targetPower);
        shooter1.getController();

        if (gamepad1.a) {
            transfer.setPower(-1);
        } else {
            transfer.setPower( 0);
        }

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        double maxPower = 2760;
        double requestedPower = toTPS(gamepad1.left_trigger > 0.1? maxPower: 0); // max: 3800

        shooter1.setVelocity(requestedPower);
        shooter2.setVelocity(requestedPower);

        telemetry.addLine("Velocity Requested: " + toRPM(requestedPower));
        telemetry.addLine("Velocity: " + toRPM(shooter1.getVelocity()));


        if (Math.abs(requestedPower - shooter1.getVelocity()) < 60 && gamepad1.left_trigger > 0.1 && count >= 4) transfer.setPower(-1);
        else transfer.setPower(0);
        count = Math.abs(requestedPower - shooter1.getVelocity()) < 60? count + 1: 0;
    }

    int count = 0;

    private double getRawHeading() {
        Orientation angles = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );

        return angles.thirdAngle;
    }
}