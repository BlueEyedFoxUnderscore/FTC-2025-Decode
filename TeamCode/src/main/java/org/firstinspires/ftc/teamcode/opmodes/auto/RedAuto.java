package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

public class RedAuto {
    public static PathChain START;
    public static PathChain REORIENT;
    public static PathChain COLLECT_GROUP_1;
    public static PathChain GROUP_SHOOT_1;
    public static PathChain COLLECT_GROUP_2;
    public static PathChain GROUPPICKUP2;
    public static PathChain GROUPSHOOT2A;
    public static PathChain GROUPSHOOT2;
    public static PathChain COLLECT_GROUP_3;
    public static PathChain GROUPPICKUP3;
    public static PathChain GROUPSHOOT3;
    public static PathChain GO_TO_SQUARE;
    public static PathChain LOOK_AT_THINGY;

    public static PathChain APRIL_TEST;
    public static PathChain APRIL_TEST_2;
    public static PathChain APRIL_TEST_3;
    public static PathChain APRIL_TEST_4;

    public static void init(Follower follower, PathAuto auto) {
        APRIL_TEST = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(121.716, 124.080), new Pose(95.381, 104.835))
                )
                .setConstantHeadingInterpolation(Math.toRadians(36))
                .addParametricCallback(0, () -> auto.setNextPath(APRIL_TEST_2))
                .addParametricCallback(0, () -> auto.setRunAtEnd(auto::reorient))
                .build();

        APRIL_TEST_2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine (new Pose(95.381, 104.835), new Pose(144.0-24.0*2-17.25/2.0,  144.0-24.0*1+16.0/2.0))))
                .setLinearHeadingInterpolation(Math.toRadians(36), 0)
                .build();

        APRIL_TEST_3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine (new Pose(95.381, 104.835), new Pose(144.0-24.0*2-17.25/2.0,  144.0-24.0*4+16.0/2.0))))
                .setLinearHeadingInterpolation(Math.toRadians(36), 0)
                .build();

        APRIL_TEST_4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine (new Pose(95.381, 104.835), new Pose(144.0-24.0*1-17.25/2.0,  144.0-24.0*0.5+1.5*0))))
                .setLinearHeadingInterpolation(Math.toRadians(36), 0)
                .build();

        START = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(121.716, 124.080), new Pose(95.381, 104.835))
                )
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(.00, auto::spinUp)
                    .addParametricCallback(.00, () -> auto.setNextPath(REORIENT))
                    .addParametricCallback(.00, () -> auto.setRunAtEnd(auto::launchBalls2))
                .build();

        REORIENT = follower
                .pathBuilder()
                .addPath(auto.stayAt(START.endPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(.00, () -> {
                        auto.setNextPath(COLLECT_GROUP_1);
                        auto.reorient();
                    })
                .build();

        COLLECT_GROUP_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.381, 104.835),
                                new Pose(86.771, 82.720),
                                new Pose(97.069, 83.564)
                        )
                )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .addParametricCallback(.02, RobotContainer.LOADER::intake)
                //    .addParametricCallback(.90, () -> auto.setFollowerMaxPower(0.5))
                .addPath(
                        new BezierLine(new Pose(97.069, 83.564), new Pose(121.210, 83.564))
                )
                    .setConstantHeadingInterpolation(0)
                    .addParametricCallback(.99, () -> auto.setFollowerMaxPower(1.0))
                .addPath(
                        new BezierLine(new Pose(121.210, 83.564), new Pose(95.381, 104.835))
                )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .addParametricCallback(.01, () -> auto.setNextPath(COLLECT_GROUP_2))
                    .addParametricCallback(.95, auto::spinUp)
                    .addParametricCallback(.99, auto::launchBalls3)
                .build();

        COLLECT_GROUP_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.381, 104.835),
                                new Pose(84.914, 57.904),
                                new Pose(96.900, 59.592)
                        )
                )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .addParametricCallback(.02, RobotContainer.LOADER::intake)
                //    .addParametricCallback(.90, () -> auto.setFollowerMaxPower(0.5))
                .addPath(
                        new BezierLine(new Pose(96.900, 59.592), new Pose(121.547, 59.254))
                )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addParametricCallback(.99, () -> auto.setFollowerMaxPower(1.0))
                .addPath(
                        new BezierLine(new Pose(121.547, 59.254), new Pose(110.586, 59.254))
                )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(110.586, 59.254),
                                new Pose(93.693, 57.397),
                                new Pose(95.381, 104.835)
                        )
                )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .addParametricCallback(.01, () -> auto.setNextPath(COLLECT_GROUP_3))
                    .addParametricCallback(.95, auto::spinUp)
                    .addParametricCallback(.99, auto::launchBalls3)
                .build();

        COLLECT_GROUP_3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.381, 104.835),
                                new Pose(84.070, 35.283),
                                new Pose(97.069, 35.283)
                        )
                )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .addParametricCallback(.02, RobotContainer.LOADER::intake)
                //    .addParametricCallback(.90, () -> auto.setFollowerMaxPower(0.5))
                .addPath(
                        new BezierLine(new Pose(97.069, 35.283), new Pose(121.210, 35.620))
                )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addParametricCallback(.99, () -> auto.setFollowerMaxPower(1.0))
                .addPath(
                        new BezierLine(new Pose(121.210, 35.620), new Pose(95.381, 104.835))
                )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .addParametricCallback(.01, () -> auto.setNextPath(GO_TO_SQUARE))
                    .addParametricCallback(.95, auto::spinUp)
                    .addParametricCallback(.99, auto::launchBalls3)
                .build();

        GROUPPICKUP3 = follower
                .pathBuilder()
                .build();

        GROUPSHOOT3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(121.210, 35.620), new Pose(95.381, 104.835))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();

        GO_TO_SQUARE = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(95.381, 104.835), new Pose(105.341, 33.426))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(90))
                .build();

        LOOK_AT_THINGY = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120.872, 126.443), new Pose(105.172, 114.288))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(135))
                .addParametricCallback(0.95, () -> auto.setRunAtEnd(() -> auto.setState(PathAuto.AutoState.GET_TAG_ID)))
                .build();
    }
}
