package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

@Autonomous
public class PathAuto extends LinearOpMode {
    boolean done = false;
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        RobotContainer.init(hardwareMap, telemetry);

        waitForStart();
        follower.followPath(new PathBuilder(follower).addPath(new BezierLine(new Pose(), new Pose().withX(24)))
                .addParametricCallback(0, RobotContainer.LOADER::intake)
                .addParametricCallback(1, RobotContainer.LOADER::cancelIntake)
                .build());
        while (!done && opModeIsActive()) {
            follower.update();
            RobotContainer.LOADER.update();
        }
    }
}
