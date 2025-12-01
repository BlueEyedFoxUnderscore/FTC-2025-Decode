package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class RedAuto {
    public static PathChain CHECK_ORDER;
    public static PathChain SORT_TO_SHOOT;
    public static PathChain SORT_SCAN_BACKWARDS;
    public static PathChain SORT_SCAN_FORWARDS;
    public static PathChain READ_TO_REORIENT;
    public static PathChain SHOOT_TO_SORT;
    public static PathChain NAIVE_AUTO_START;
    public static PathChain REORIENT;
    public static PathChain COLLECT_GROUP_1;
    public static PathChain COLLECT_GROUP_2;
    public static PathChain COLLECT_GROUP_3;
    public static PathChain GO_TO_SQUARE;
    public static PathChain LOOK_AT_THINGY;

    public static PathChain APRIL_TEST;
    public static PathChain APRIL_TEST_2;
    public static PathChain APRIL_TEST_3;
    public static PathChain APRIL_TEST_4;

    public static void init(Follower follower, PathAuto auto) {

        AtomicReference<PathChain> afterCollectGroup1 = new AtomicReference<>();
        AtomicReference<PathChain> afterCollectGroup2 = new AtomicReference<>();
        AtomicReference<PathChain> afterCollectGroup3 = new AtomicReference<>();

        APRIL_TEST = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(121.716, 124.080), new Pose(95.381, 104.835))
                )
                .setConstantHeadingInterpolation(Math.toRadians(36))
                .addParametricCallback(0, () -> auto.setNextPath(APRIL_TEST_2, "APRIL_TEST_2 from APRIL_TEST"))
                .addParametricCallback(0, () -> auto.setRunAtEnd(auto::reorient, "auto::reorient from APRIL_TEST"))
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

        NAIVE_AUTO_START = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(121.716, 124.080), new Pose(95.381, 104.835))
                )
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(.00, () -> auto.spinUp("NATIVE_AUTO_START"))
                    .addParametricCallback(.00, () -> auto.setNextPath(REORIENT, "REORIENT from NAIVE_AUTO_START"))
                    .addParametricCallback(.00, () -> auto.setRunAtEnd(auto::launchBalls2, "auto::launchBalls2 from NAIVE_AUTO_START"))
                    .addParametricCallback(.00, () -> {
                        afterCollectGroup1.set(COLLECT_GROUP_2);
                        afterCollectGroup2.set(COLLECT_GROUP_3);
                        afterCollectGroup3.set(GO_TO_SQUARE);
                    })
                .build();


        REORIENT = follower
                .pathBuilder()
                .addPath(auto.stayAt(NAIVE_AUTO_START.endPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(.00, () -> {
                        auto.setNextPath(COLLECT_GROUP_1, "COLLECT_GROUP_1 from REORIENT");
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
                    .addParametricCallback(.01, () -> auto.setNextPath(afterCollectGroup1.get(), "afterCollectGroup1.get() from COLLECT_GROUP_1"))
                    .addParametricCallback(.95, ()->auto.spinUp("COLLECT_GROUP_1"))
                //    .addParametricCallback(.99, auto::launchBalls3)
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
                    .addParametricCallback(.01, () -> auto.setNextPath(afterCollectGroup2.get(), "afterCollectGroup2.get() from COLLECT_GROUP_1"))
                    .addParametricCallback(.95, ()->auto.spinUp("COLLECT_GROUP_2"))
                //    .addParametricCallback(.99, auto::launchBalls3)
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
                .addPath(
                        new BezierLine(new Pose(97.069, 35.283), new Pose(121.210, 35.620))
                )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addParametricCallback(.99, () -> auto.setFollowerMaxPower(1.0))
                .addPath(
                        new BezierLine(new Pose(121.210, 35.620), new Pose(95.381, 104.835))
                )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .addParametricCallback(.01, () -> auto.setNextPath(afterCollectGroup3.get(), "afterCollectGroup3.get() from COLLECT_GROUP_1"))
                    .addParametricCallback(.95, ()->auto.spinUp("COLLECT_GROUP_3"))
                //    .addParametricCallback(.99, auto::launchBalls3)
                .build();

        //GROUPSHOOT3 = follower
        //        .pathBuilder()
        //        .addPath(
        //                new BezierLine(new Pose(121.210, 35.620), new Pose(95.381, 104.835))
        //        )
        //        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
        //        .build();

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
                    .addParametricCallback(0.95, () -> auto.setRunAtEnd(() -> auto.setState(PathAuto.AutoState.GET_TAG_ID), "Auto -> GET_TAG_ID from LOOK_AT_THINGY"))
                .build();

        READ_TO_REORIENT = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120.872, 126.443), new Pose(86.603, 130.157)) // to read position
                )
                    .setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(120))
                    .addParametricCallback(0.95, () -> auto.setState(PathAuto.AutoState.GET_TAG_ID)) // start reading

                .addPath(
                        new BezierLine(new Pose(86.603, 130.157), new Pose(100.783, 109.393)) // to sort position
                )
                    .addParametricCallback(.00, () -> {
                        afterCollectGroup1.set(CHECK_ORDER);
                        afterCollectGroup2.set(CHECK_ORDER);
                        afterCollectGroup3.set(CHECK_ORDER);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(45))
                    .addParametricCallback(0.1, () -> auto.setRunAtEnd(auto::reorient, "auto::reorient from READ_TO_REORIENT"))
                    .addParametricCallback(0.15, () -> auto.spinUp("READ_TO_REORIENT"))
                    .addParametricCallback(0.2, () -> auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from READ_TO_REORIENT"))
                .build();

        AtomicInteger lastGroup = new AtomicInteger();

        CHECK_ORDER = follower
                .pathBuilder()
                //work
                .addPath(auto.stayAt(READ_TO_REORIENT.endPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    //.addParametricCallback(0.2, ()->auto.spinUp("CHECK_ORDER"))
                    .addParametricCallback(0.5, () -> auto.setRunAtEnd(() ->
                            {
                                switch (auto.heldstate) {
                                    case PGP:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.heldstate = PathAuto.BallState.GP;
                                                auto.launchBalls1();
                                                auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from CHECK_ORDER hPGP, gPPG");
                                                break;
                                            case PGP:
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                auto.launchBalls3();
                                                break;
                                            case GPP:
                                                auto.heldstate = PathAuto.BallState.PGP;
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hPGP, gPGP");
                                                auto.startNextPath();
                                                auto.setRunAtEnd(auto::cycle1);
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hPGP, gPPG");
                                                break;
                                        }
                                        break;
                                    case GPP:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.heldstate = PathAuto.BallState.GPP;
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hGPP, gPPG");
                                                auto.startNextPath();
                                                auto.setRunAtEnd(auto::cycle1);
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hGPP, gPPG");
                                                break;
                                            case PGP:
                                                auto.telemetry.addData("Fuck", "Shit");
                                                auto.telemetry.update();
                                                int fucksUpTheProgram = 0 / 0;
                                                // I messed up somehow
                                                break;
                                            case GPP:
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                auto.launchBalls3();
                                                break;
                                        }
                                        break;
                                    case PPG:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                auto.launchBalls3();
                                                break;
                                            case PGP:
                                                auto.heldstate = PathAuto.BallState.GPP;
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hPPG, gGPP");
                                                auto.startNextPath();
                                                auto.setRunAtEnd(auto::cycle1);
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hPPG, gGPP");
                                                break;
                                            case GPP:
                                                auto.telemetry.addData("Fuck", "Shit");
                                                auto.telemetry.update();
                                                int fucksUpTheProgram = 0 / 0;
                                                // I messed up somehow
                                        }
                                        break;
                                    case GP:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.heldstate = PathAuto.BallState.PG;
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER");
                                                auto.startNextPath();
                                                auto.setRunAtEnd(auto::cycle1);
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER");
                                                break;
                                            case PGP:
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                auto.launchBalls2();
                                                break;
                                            case GPP:
                                                auto.telemetry.addData("Fuck", "Shit");
                                                auto.telemetry.update();
                                                int fucksUpTheProgram = 0 / 0;
                                                // I messed up somehow
                                                break;
                                        }
                                        break;
                                    case PG:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                auto.setRunAtEnd(auto::launchBalls2);
                                                break;
                                            case PGP:
                                                auto.heldstate = PathAuto.BallState.GP;
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hPG, gPGP");
                                                auto.startNextPath();
                                                auto.setRunAtEnd(auto::cycle1);
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hPG, gPGP");
                                                break;
                                            case GPP:
                                                auto.telemetry.addData("Fuck", "Shit");
                                                auto.telemetry.update();
                                                int fucksUpTheProgram = 0 / 0;
                                                // I messed up somehow
                                                break;
                                        }
                                        break;
                                    case EMPTY:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                switch (lastGroup.get()) {
                                                    case 0:
                                                        auto.heldstate = PathAuto.BallState.PPG;
                                                        lastGroup.set(3);
                                                        auto.setNextPath(COLLECT_GROUP_3, "COLLECT_GROUP_3 from CHECK_ORDER hEMPTY, gPPG, 0");
                                                        break;
                                                    case 3:
                                                        auto.heldstate = PathAuto.BallState.GPP;
                                                        lastGroup.set(1);
                                                        auto.setNextPath(COLLECT_GROUP_1, "COLLECT_GROUP_1 from CHECK_ORDER hEMPTY, gPPG, 3");
                                                        break;
                                                    case 1:
                                                        // Lie about ball order so that we don't have to sort
                                                        auto.heldstate = auto.gamestate;
                                                        lastGroup.set(2);
                                                        auto.setNextPath(COLLECT_GROUP_2, "COLLECT_GROUP_2 from CHECK_ORDER hEMPTY, gPPG, 1");
                                                        break;
                                                    case 2:
                                                        auto.setNextPath(GO_TO_SQUARE, "GO_TO_SQUARE from CHECK_ORDER hEMPTY, gPPG, 2");
                                                        break;
                                                }
                                                break;
                                            case PGP:
                                                switch (lastGroup.get()) {
                                                    case 0:
                                                        auto.heldstate = PathAuto.BallState.PGP;
                                                        lastGroup.set(2);
                                                        auto.setNextPath(COLLECT_GROUP_2, "COLLECT_GROUP_2 from CHECK_ORDER hEMPTY, gPGP, 0");
                                                        break;
                                                    case 2:
                                                        auto.heldstate = PathAuto.BallState.PPG;
                                                        lastGroup.set(3);
                                                        auto.setNextPath(COLLECT_GROUP_3, "COLLECT_GROUP_3 from CHECK_ORDER hEMPTY, gPGP, 2");
                                                        break;
                                                    case 3:
                                                        // Lie about ball order so that we don't have to sort
                                                        auto.heldstate = auto.gamestate;
                                                        lastGroup.set(1);
                                                        auto.setNextPath(COLLECT_GROUP_1, "COLLECT_GROUP_1 from CHECK_ORDER hEMPTY, gPGP, 3");
                                                        break;
                                                    case 1:
                                                        auto.setNextPath(GO_TO_SQUARE, "GO_TO_SQUARE from CHECK_ORDER hEMPTY, gPGP, 1");
                                                        break;
                                                }
                                                break;
                                            case GPP:
                                                switch (lastGroup.get()) {
                                                    case 0:
                                                        auto.heldstate = PathAuto.BallState.GPP;
                                                        lastGroup.set(1);
                                                        auto.setNextPath(COLLECT_GROUP_1, "COLLECT_GROUP_1 from CHECK_ORDER hEMPTY, gGPP, 0");
                                                        break;
                                                    case 1:
                                                        auto.heldstate = PathAuto.BallState.PGP;
                                                        lastGroup.set(2);
                                                        auto.setNextPath(COLLECT_GROUP_2, "COLLECT_GROUP_2 from CHECK_ORDER hEMPTY, gGPP, 1");
                                                        break;
                                                    case 2:
                                                        // Lie about ball order so that we don't have to sort
                                                        auto.heldstate = auto.gamestate;
                                                        lastGroup.set(3);
                                                        auto.setNextPath(COLLECT_GROUP_3, "COLLECT_GROUP_3 from CHECK_ORDER hEMPTY, gGPP, 2");
                                                        break;
                                                    case 3:
                                                        auto.setNextPath(GO_TO_SQUARE, "GO_TO_SQUARE from CHECK_ORDER hEMPTY, gGPP, 3");
                                                    break;
                                                }
                                            break;
                                        }
                                    break;
                                }
                            }
                            , "complex behavior from CHECK_ORDER"))
                .build();

        SHOOT_TO_SORT = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.783, 109.393), new Pose(123.236, 118.145))
                )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(20))
                    .addParametricCallback(0.0, () -> auto.spinHalf("SHOOT_TO_SORT slow spin for sorting"))
                    .addParametricCallback(0.2, () -> auto.setNextPath(SORT_TO_SHOOT, "SHOOT_TO_SORT from SHOOT_TO_SORT")) // set this to what hapens after cycling
                    .addParametricCallback(0.5, () -> auto.setRunAtEnd(auto::sort, "auto::sort from SHOOT_TO_SORT")) // rename this cyclen, have cyclen remember next path and change it to sort_scan_forwards there instead if you want to cycle, or restore and call next path if done.
                .build();

        SORT_SCAN_FORWARDS = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.236, 118.145), new Pose(126.274, 119.327)) // 116
                )
                    .setConstantHeadingInterpolation(Math.toRadians(20))
                    .addParametricCallback(0.5, () -> auto.setNextPath(SORT_SCAN_BACKWARDS, "SORT_SCAN_BACKWARDS from SORT_SCAN_FORWARDS"))
                    .addParametricCallback(0.5, () -> auto.setRunAtEnd(auto::startNextPath, "auto::startNextPath from SORT_SCAN_FORWARDS"))
                .build();

        SORT_SCAN_BACKWARDS = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.274, 119.327), new Pose(123.236, 118.145))
                )
                    .setConstantHeadingInterpolation(Math.toRadians(20))
                .build();

        SORT_TO_SHOOT = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.236, 118.145), new Pose(100.783, 109.393))
                )
                    .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(45))
                    .addParametricCallback(0.1, () -> auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from SORT_TO_SHOOT"))
                    .addParametricCallback(0.2, () -> auto.setRunAtEnd(auto::startNextPath, "auto::startNextPath from SORT_TO_SHOOT"))
                .build();
    }

    static Runnable andThen(Runnable ...runnables) {
        return () -> {for (Runnable runnable : runnables) runnable.run();};
    }
}
