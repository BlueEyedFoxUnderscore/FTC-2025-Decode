package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.StrictMath.PI;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

@TeleOp//(name = "***FTC OpMode 2", group = "TeleOp")

public class MainControl extends OpMode {
    ElapsedTime imuTimeout = new ElapsedTime();
    private Follower follower;

    //GoBildaPinpointDriver odometry1;
    //GoBildaPinpointDriver odometry2;

    //private MotorGroup shooterGroup;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.startTeleopDrive();
        follower.update();
        imuTimeout.reset();
        RobotContainer.init(hardwareMap, telemetry);

        telemetry.addLine("Initialized - waiting for IMU calibration...");
        telemetry.update();
        //odometry1.setOffsets(-84.0, -168.0, DistanceUnit.MM); // T U N E   T H E S E
        //odometry1.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //odometry2.setOffsets(-84.0, -168.0, DistanceUnit.MM); // T U N E   T H E S E
        //odometry2.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }
    double offset = 0;
    public void loop() {
        updateDrive();
        if (gamepad1.y) {
            reorient();
        }
        updateLoader();

        /**/
//        if (gamepad1.left_trigger > 0.1) {
//            flywheel.setRequested(2760, 2560);
//            if(flywheel.isStable()) {
//                transfer.setPower(-0.6);
//            } else {
//                transfer.setPower(0);
//            }
//        } else {
//            flywheel.setRequested(0, 0);
//            transfer.setPower(0);
//        }

        RobotContainer.LOADER.update();
        RobotContainer.FLYWHEEL.update();

        telemetry.addLine(String.valueOf(gamepad1.right_trigger));
        telemetry.update();
        telemetry.addLine(String.valueOf(RobotContainer.FLYWHEEL.getSpeed()));
        telemetry.clear();
//        gate.setPosition(gamepad1.right_trigger);
    }

    private void updateDrive() {
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false, offset);
        follower.update();
    }

    private void reorient() {
        offset = follower.getHeading() + PI;
    }

    private void intake() {
        RobotContainer.LOADER.intake();
    }

    private void cancelIntake() {
        RobotContainer.LOADER.cancelIntake();
    }

    private void launch() {
        RobotContainer.LOADER.launch();
    }

    private void spinUp() {
        RobotContainer.FLYWHEEL.setRequested(2800, 2400);
    }

    private void spinDown() {
        RobotContainer.FLYWHEEL.setRequested(0, 0);
    }

    private void cancelLaunch() {
        RobotContainer.LOADER.cancelLaunch();
    }

    private boolean flywheelStable() {
        return RobotContainer.FLYWHEEL.isStable();
    }

    private void updateLoader() {
        if (gamepad1.right_bumper) intake();
        else cancelIntake();
        if (gamepad1.left_bumper) {
            spinUp();
            if (flywheelStable()) launch();
        } else {
            spinDown();
            cancelLaunch();
        }
    }
}