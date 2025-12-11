package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class RedAuto {
    public static PathChain START_EJECT_SORT_AUTO;
    public static PathChain PPG_EJECT_PATH_1;
    public static PathChain PPG_EJECT_PATH_1b;
    public static PathChain PPG_EJECT_PATH_2;
    public static PathChain PPG_EJECT_PATH_2b;
    public static PathChain PPG_EJECT_PATH_3;
    public static PathChain PPG_EJECT_PATH_4;
    public static PathChain PPG_EJECT_PATH_4b;
    public static PathChain PPG_EJECT_PATH_4c;
    public static PathChain PPG_EJECT_PATH_5;
    public static PathChain PPG_EJECT_PATH_5b;
    public static PathChain PGP_EJECT_PATH_1;
    public static PathChain GPP_EJECT_PATH_1;
    public static PathChain GPP_EJECT_PATH_2;
    public static PathChain GPP_EJECT_PATH_2b;
    public static PathChain GPP_EJECT_PATH_3;
    public static PathChain GPP_EJECT_PATH_4;
    public static PathChain GPP_EJECT_PATH_4b;
    public static PathChain GPP_EJECT_PATH_4c;
    public static PathChain GPP_EJECT_PATH_5;
    public static PathChain GPP_EJECT_PATH_5b;
    public static PathChain CHECK_ORDER;
    public static PathChain SORT_TO_SHOOT;
    public static PathChain SORT_SCAN_BACKWARDS;
    public static PathChain SORT_SCAN_FORWARDS;
    public static PathChain READ_TO_REORIENT;
    public static PathChain SHOOT_TO_SORT;
    public static PathChain NAIVE_AUTO_START;
    public static PathChain REORIENT;
    public static PathChain COLLECT_PPG_ROW;
    public static PathChain COLLECT_PGP_ROW;
    public static PathChain COLLECT_GPP_ROW;
    public static PathChain GO_TO_SQUARE;
    public static PathChain LOOK_AT_THINGY;

    private static final double PICKUP_SPEED = 0.5;

    public static BezierLine stayAt(Pose location) {
        return new BezierLine(location, location.withY(location.getY() + 1./1000.));
    }


    public static void init(Follower follower, PathAuto auto) {

        AtomicReference<PathChain> afterCollectPPG = new AtomicReference<>();
        AtomicReference<PathChain> afterCollectPGP = new AtomicReference<>();
        AtomicReference<PathChain> afterCollectGPP = new AtomicReference<>();

        final Pose START_FACING_GOAL_POSE = new Pose(121.716, 124.080).withHeading(Math.toRadians(36));

        final Pose START_PARALLEL_TO_GOAL_POSE = new Pose(120.872, 126.443).withHeading(Math.toRadians(126));
        final Pose READ_OBELISK_POSE = new Pose(/*86.603, 130.157,/**/ 101, 115).withHeading(Math.toRadians(135));

        final Pose SHOOTING_POSE = new Pose(98, 98).withHeading(Math.toRadians(45));

        final Pose PPG_APPROACH_CONTROL_POINT_POSE = new Pose(90-2, 84);
        final Pose PPG_ROW_HEAD_POSE = new Pose(94-2, 84).withHeading(Math.toRadians(0));
        final Pose PPG_ROW_TAIL_POSE = new Pose(121, 84).withHeading(Math.toRadians(0));

        final Pose PGP_APPROACH_CONTROL_POINT_POSE = new Pose(85.0212765957447, 60);
        final Pose PGP_ROW_HEAD_POSE = new Pose(94, 60).withHeading(Math.toRadians(0));
        final Pose PGP_ROW_TAIL_POSE = new Pose(121.547, 60).withHeading(Math.toRadians(0));
        final Pose PGP_ROW_EXIT_POSE = new Pose(106, 60).withHeading(Math.toRadians(0));
        final Pose PGP_ROW_EXIT_CONTROL_POINT = new Pose( 71, 72);

        final Pose GPP_APPROACH_CONTROL_POINT_POSE = new Pose(80, 36);
        final Pose GPP_ROW_HEAD_POSE = new Pose(92, 36).withHeading(Math.toRadians(0));
        final Pose GPP_ROW_TAIL_POSE = new Pose(121.210, 36).withHeading(Math.toRadians(0));
        final Pose GPP_ROW_EXIT_POSE = new Pose(106, 36).withHeading(Math.toRadians(0));
        final Pose GPP_ROW_EXIT_CONTROL_POINT = new Pose( 73, 48);

        START_EJECT_SORT_AUTO = follower.pathBuilder()
                .addPath(stayAt(START_PARALLEL_TO_GOAL_POSE))
                .setConstantHeadingInterpolation(START_PARALLEL_TO_GOAL_POSE.getHeading())
                .addParametricCallback(0.0, () -> auto.setRunAtEnd(() -> follower.followPath(READ_TO_REORIENT), "START_EJECT_SORT_AUTO->READ_TO_REORIENT"))
                .build();

        // Read obelisk and then reorient the robot
        READ_TO_REORIENT = follower.pathBuilder()
                .addPath(new BezierLine(START_PARALLEL_TO_GOAL_POSE, READ_OBELISK_POSE))
                    .setLinearHeadingInterpolation(START_PARALLEL_TO_GOAL_POSE.getHeading(), READ_OBELISK_POSE.getHeading())
                    .addParametricCallback(0, () -> auto.setState(PathAuto.AutoState.GET_TAG_ID)) // start reading
                .addPath(new BezierLine(READ_OBELISK_POSE, SHOOTING_POSE)) // to sort position)
                    .setLinearHeadingInterpolation(READ_OBELISK_POSE.getHeading(), SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, () -> auto.setState(PathAuto.AutoState.GET_TAG_ID)) // start reading
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, auto::reorient)
                    .addParametricCallback(0, () -> {
                            switch (auto.gamestate) {
                                case GPP:
                                    auto.setNextPath(GPP_EJECT_PATH_1, "GPP_EJECT_PATH_1 from REORIENT");
                                    break;
                                case PGP:
                                    auto.setNextPath(PGP_EJECT_PATH_1, "PGP_EJECT_PATH_1 from REORIENT");
                                    break;
                                case PPG:
                                    auto.setNextPath(PPG_EJECT_PATH_1, "PPG_EJECT_PATH_1 from REORIENT");
                                    break;
                                default:
                            }
                        })
                .build();

        // Eject one ball from the preload
        GPP_EJECT_PATH_1 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> auto.setNextPath(GPP_EJECT_PATH_2, ""))
                    .addParametricCallback(0.0, auto::eject)
                .build();

        // Spin up for the preload shot
        GPP_EJECT_PATH_2 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> auto.spinUp("from GPP_EJECT_PATH_2"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(() -> follower.followPath(GPP_EJECT_PATH_2b), "from GPP_EJECT_PATH_2 goto GPP_EJECT_PATH2b"))

                .build();

        // Shoot the remaining 2 preload balls then collect PGP
        GPP_EJECT_PATH_2b = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> afterCollectPGP.set(GPP_EJECT_PATH_3))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(auto::launchBalls2, "from GPP_EJECT_PATH_2b schedule LAUNCH_BALLS_2 at end"))
                    .addParametricCallback(0.0, () -> auto.setNextPath(COLLECT_PGP_ROW, "from GPP_EJECT_PATH_2b schedule COLLECT_PPG_ROW as nextPath"))

                .build();


        // Shoot all 3 PGP, then collect PPG
        GPP_EJECT_PATH_3 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, auto::launchBalls3)
                    .addParametricCallback(0.0, () -> afterCollectPPG.set(GPP_EJECT_PATH_4))
                    .addParametricCallback(0.0, () -> auto.setNextPath(COLLECT_PPG_ROW, "from GPP_EJECT_PATH_2b schedule COLLECT_PPG_ROW as nextPath"))

                .build();

        // Eject 1 from PGP
        GPP_EJECT_PATH_4 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, () -> auto.setNextPath(GPP_EJECT_PATH_4b, "GPP_EJECT_PATH_4b"))
                    .addParametricCallback(0, auto::eject)
                .build();

        // Spin up for the preload shot
        GPP_EJECT_PATH_4b = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> auto.spinUp("from GPP_EJECT_PATH_4b"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(() -> follower.followPath(GPP_EJECT_PATH_4c), "GPP_EJECT_PATH_4b->GPP_EJECT_PATH_4c"))

                .build();

        // Spin up and launch the remaining preload shot
        GPP_EJECT_PATH_4c = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, auto::launchBalls2)
                    .addParametricCallback(0, () -> afterCollectGPP.set(GPP_EJECT_PATH_5))
                    .addParametricCallback(0, () -> auto.setNextPath(COLLECT_GPP_ROW, "COLLECT_GPP_ROW"))
                .build();

        GPP_EJECT_PATH_5 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, auto::eject)
                    .addParametricCallback(0, () -> auto.setNextPath(GPP_EJECT_PATH_5b, "GPP_EJECT_PATH_5b"))
                .build();

        GPP_EJECT_PATH_5b = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0.0, ()  -> {
                        auto.spinUp("from GPP_EJECT_PATH_5b");
                        auto.launchBalls2();
                    })
                .build();

        PGP_EJECT_PATH_1 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> afterCollectPGP.set(GPP_EJECT_PATH_3))
                    .addParametricCallback(0.0, () -> auto.spinUp("from PGP_EJECT_PATH_1"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(auto::launchBalls3, "from PGP_EJECT_PATH_1 schedule auto::launchBalls3 at end"))
                    .addParametricCallback(0.0, () -> auto.setNextPath(COLLECT_PGP_ROW, "from PGP_EJECT_PATH_1 schedule COLLECT_PGP_ROW as nextPath"))

                .build();

        
        // Launch one ball from the preload
        PPG_EJECT_PATH_1 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0.0, () -> auto.setNextPath(PPG_EJECT_PATH_1b, "from PPG_EJECT_PATH_1 schedule GPP_EJECT_PATH_1b at end"))
                    .addParametricCallback(0.0, () -> auto.spinUp("from PPG_EJECT_PATH_1"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(auto::launchBalls1, "from PPG_EJECT_PATH_1 schedule auto::launchBalls1 at end"))
                .build();
    
        // Eject one ball from the preload
        PPG_EJECT_PATH_1b = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0.0, auto::eject)
                    .addParametricCallback(0.0, () -> auto.setNextPath(PPG_EJECT_PATH_2, "from PPG_EJECT_PATH_1b"))
                .build();

        // Spin up for the preload shot
        PPG_EJECT_PATH_2 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> auto.spinUp("from PPG_EJECT_PATH_2"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(() -> follower.followPath(PPG_EJECT_PATH_2b), "from PPG_EJECT_PATH_2 goto PPG_EJECT_PATH2b"))

                .build();

        // Shoot the remaining preload ball then collect GPP
        PPG_EJECT_PATH_2b = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> afterCollectGPP.set(PPG_EJECT_PATH_3))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(auto::launchBalls1, "from PPG_EJECT_PATH_2b schedule LAUNCH_BALLS_2 at end"))
                    .addParametricCallback(0.0, () -> auto.setNextPath(COLLECT_GPP_ROW, "from PPG_EJECT_PATH_2b schedule COLLECT_GPP_ROW as nextPath"))

                .build();


        // Shoot all 3 PGP, then collect PPG
        PPG_EJECT_PATH_3 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, auto::launchBalls3)
                    .addParametricCallback(0.0, () -> afterCollectPGP.set(PPG_EJECT_PATH_4))
                    .addParametricCallback(0.0, () -> auto.setNextPath(COLLECT_PGP_ROW, "from PPG_EJECT_PATH_2b schedule COLLECT_PGP_ROW as nextPath"))

                .build();

        // Eject 1 from PGP
        PPG_EJECT_PATH_4 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, () -> auto.setNextPath(PPG_EJECT_PATH_4b, "PPG_EJECT_PATH_4b"))
                    .addParametricCallback(0, auto::eject)
                .build();

        // Spin up for the preload shot
        PPG_EJECT_PATH_4b = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())

                    .addParametricCallback(0.0, () -> auto.spinUp("from PPG_EJECT_PATH_4b"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(() -> follower.followPath(PPG_EJECT_PATH_4c), "PPG_EJECT_PATH_4b->PPG_EJECT_PATH_4c"))

                .build();

        // Spin up and launch the remaining preload shot
        PPG_EJECT_PATH_4c = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, auto::launchBalls2)
                    .addParametricCallback(0, () -> afterCollectPPG.set(PPG_EJECT_PATH_5))
                    .addParametricCallback(0, () -> auto.setNextPath(COLLECT_PPG_ROW, "COLLECT_PPG_ROW"))
                .build();

        PPG_EJECT_PATH_5 = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, auto::eject)
                    .addParametricCallback(0, () -> auto.setNextPath(PPG_EJECT_PATH_5b, "PPG_EJECT_PATH_5b"))
                .build();

        PPG_EJECT_PATH_5b = follower.pathBuilder()
                .addPath(stayAt(SHOOTING_POSE))
                    .setConstantHeadingInterpolation(SHOOTING_POSE.getHeading())
                    .addParametricCallback(0.0, ()  -> {
                        auto.spinUp("from PPG_EJECT_PATH_5b");
                        auto.launchBalls2();
                    })
                .build();


        COLLECT_PPG_ROW = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOTING_POSE, PPG_APPROACH_CONTROL_POINT_POSE, PPG_ROW_HEAD_POSE))
                    .setLinearHeadingInterpolation(SHOOTING_POSE.getHeading(), PPG_ROW_HEAD_POSE.getHeading())
                    .addParametricCallback(0, RobotContainer.LOADER::intake)
                .addPath(new BezierLine(PPG_ROW_HEAD_POSE, PPG_ROW_TAIL_POSE))
                    .addParametricCallback(0, () -> auto.setFollowerMaxPower(PICKUP_SPEED))
                    .setLinearHeadingInterpolation(PPG_ROW_HEAD_POSE.getHeading(), PPG_ROW_TAIL_POSE.getHeading())
                .addPath(new BezierLine(PPG_ROW_TAIL_POSE, SHOOTING_POSE))
                    .setLinearHeadingInterpolation(PPG_ROW_TAIL_POSE.getHeading(), SHOOTING_POSE.getHeading())
                    .addParametricCallback(0, () -> auto.setFollowerMaxPower(1))
                .addPath(stayAt(SHOOTING_POSE))
                    .addParametricCallback(.95, RobotContainer.LOADER::cancelIntake)
                    .addParametricCallback(0, () -> auto.spinUp("COLLECT_PPG_ROW"))
                    .addParametricCallback(0, () -> auto.setRunAtEnd(() -> follower.followPath(afterCollectPPG.get()), "Start path "+afterCollectPPG.get()+" after COLLECT_PPG_ROW" ))
                .build();

        COLLECT_PGP_ROW = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOTING_POSE, PGP_APPROACH_CONTROL_POINT_POSE, PGP_ROW_HEAD_POSE))
                    .setLinearHeadingInterpolation(SHOOTING_POSE.getHeading(), PGP_ROW_HEAD_POSE.getHeading())
                    .addParametricCallback(0.0, RobotContainer.LOADER::intake)
                .addPath(new BezierLine(PGP_ROW_HEAD_POSE, PGP_ROW_TAIL_POSE))
                    .setLinearHeadingInterpolation(PGP_ROW_HEAD_POSE.getHeading(), PGP_ROW_TAIL_POSE.getHeading())
                    .addParametricCallback(0, () -> auto.setFollowerMaxPower(PICKUP_SPEED))
                .addPath(new BezierLine(PGP_ROW_TAIL_POSE, PGP_ROW_EXIT_POSE))
                    .setLinearHeadingInterpolation(PGP_ROW_TAIL_POSE.getHeading(), PGP_ROW_EXIT_POSE.getHeading())
                    .addParametricCallback(0, () -> auto.setFollowerMaxPower(1))
                .addPath(new BezierCurve(PGP_ROW_EXIT_POSE, PGP_ROW_EXIT_CONTROL_POINT, SHOOTING_POSE))
                    .setLinearHeadingInterpolation(PGP_ROW_EXIT_POSE.getHeading(), SHOOTING_POSE.getHeading())
                    .addParametricCallback(0.0, () -> auto.setNextPath(afterCollectPGP.get(), "afterCollectGroup2.get() from COLLECT_GROUP_2"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(() -> auto.startNextPath("at end of COLLECT_GROUP_2"), "auto::startNextPath from COLLECT_GROUP_2"))
                    .addParametricCallback(0.0, RobotContainer.LOADER::cancelIntake)
                    .addParametricCallback(0.5, ()->auto.spinUp("COLLECT_GROUP_2"))
                .build();


        COLLECT_GPP_ROW = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOTING_POSE, GPP_APPROACH_CONTROL_POINT_POSE, GPP_ROW_HEAD_POSE))
                    .setLinearHeadingInterpolation(SHOOTING_POSE.getHeading(), GPP_ROW_HEAD_POSE.getHeading())
                    .addParametricCallback(0.0, RobotContainer.LOADER::intake)
                .addPath(new BezierLine(GPP_ROW_HEAD_POSE, GPP_ROW_TAIL_POSE))
                    .setLinearHeadingInterpolation(GPP_ROW_HEAD_POSE.getHeading(), GPP_ROW_TAIL_POSE.getHeading())
                    .addParametricCallback(0.0, () -> auto.setFollowerMaxPower(PICKUP_SPEED))
                .addPath(new BezierLine(GPP_ROW_TAIL_POSE, GPP_ROW_EXIT_POSE))
                    .setLinearHeadingInterpolation(GPP_ROW_TAIL_POSE.getHeading(), GPP_ROW_EXIT_POSE.getHeading())
                    .addParametricCallback(0.0, () -> auto.setFollowerMaxPower(1.0))
                .addPath(new BezierCurve(GPP_ROW_EXIT_POSE, GPP_ROW_EXIT_CONTROL_POINT, SHOOTING_POSE))
                    .setLinearHeadingInterpolation(GPP_ROW_EXIT_POSE.getHeading(), SHOOTING_POSE.getHeading())
                    .addParametricCallback(0.0, () -> auto.setNextPath(afterCollectGPP.get(), "afterCollectGroup3.get() from COLLECT_GROUP_3"))
                    .addParametricCallback(0.0, () -> auto.setRunAtEnd(() -> auto.startNextPath("at end of COLLECT_GROUP_3"), "auto::startNextPath from COLLECT_GROUP_3"))
                    .addParametricCallback(0.0, RobotContainer.LOADER::cancelIntake)
                    .addParametricCallback(0.5, ()->auto.spinUp("COLLECT_GROUP_3"))
                .build();


        NAIVE_AUTO_START = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(START_FACING_GOAL_POSE, SHOOTING_POSE)
                )
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(.00, () -> auto.spinUp("NATIVE_AUTO_START"))
                    .addParametricCallback(.00, () -> auto.setNextPath(REORIENT, "REORIENT from NAIVE_AUTO_START"))
                    .addParametricCallback(.00, () -> auto.setRunAtEnd(auto::launchBalls2, "auto::launchBalls2 from NAIVE_AUTO_START"))
                    .addParametricCallback(.00, () -> {
                        afterCollectPPG.set(COLLECT_PGP_ROW);
                        afterCollectPGP.set(COLLECT_GPP_ROW);
                        afterCollectGPP.set(GO_TO_SQUARE);
                    })
                .build();


        REORIENT = follower
                .pathBuilder()
                .addPath(stayAt(NAIVE_AUTO_START.endPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(.00, () -> {
                        switch (auto.gamestate) {
                            case GPP:
                                auto.setNextPath(GPP_EJECT_PATH_1, "GPP_EJECT_PATH_1 from REORIENT");
                                break;
                            case PGP:
                                auto.setNextPath(PGP_EJECT_PATH_1, "PGP_EJECT_PATH_1 from REORIENT");
                                break;
                            case PPG:
                                auto.setNextPath(PPG_EJECT_PATH_1, "PPG_EJECT_PATH_1 from REORIENT");
                                break;
                            default:
                        }
                        auto.reorient();
                    })
                .build();

        GO_TO_SQUARE = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SHOOTING_POSE, new Pose(105.341, 33.426))
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


        AtomicInteger lastGroup = new AtomicInteger();

        {
            CHECK_ORDER = follower
                    .pathBuilder()
                    //work
                    .addPath(stayAt(READ_TO_REORIENT.endPoint()))
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .addParametricCallback(0.2, () -> auto.spinUp("CHECK_ORDER"))
                    .addParametricCallback(0.5, () -> auto.setRunAtEnd(() ->
                            {
                                Log.i("20311", "Doing CHECK_ORDER with h" + auto.heldstate + " g" + auto.gamestate);
                                switch (auto.heldstate) {
                                    case PGP:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.launchBalls1();
                                                auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from CHECK_ORDER hPGP, gPPG");
                                                auto.heldstate = PathAuto.BallState.GP;
                                                break;
                                            case PGP:
                                                auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from CHECK_ORDER hPGP, gPGP");
                                                auto.launchBalls3();
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                break;
                                            case GPP:
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hPGP, gGPP");
                                                auto.startNextPath("from CHECK_ORDER hPGP, gPGP setRunAtEnd");
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hPGP, gGPP");
                                                auto.heldstate = PathAuto.BallState.GPP;
                                                break;
                                        }
                                        break;
                                    case GPP:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hGPP, gPPG");
                                                auto.startNextPath("from CHECK_ORDER hGPP, gPPG setRunAtEnd");
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hGPP, gPPG");
                                                auto.heldstate = PathAuto.BallState.PPG;
                                                break;
                                            case PGP:
                                                Log.i("20311", "ASSERTION ---- Shouldn't get to hGPP, gPGP");
                                                break;
                                            case GPP:
                                                auto.launchBalls3();
                                                auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from CHECK_ORDER hGPP, gGPP");
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                break;
                                        }
                                        break;
                                    case PPG:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.launchBalls3();
                                                //?
                                                auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from CHECK_ORDER hPPG, gPPG");
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                break;
                                            case PGP:
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hPPG, gPGP");
                                                auto.startNextPath("from CHECK_ORDER hPPG, gPGP setRunAtEnd");
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hPPG, gPGP");
                                                auto.heldstate = PathAuto.BallState.PGP;
                                                break;
                                            case GPP:
                                                Log.i("20311", "ASSERTION ---- Shouldn't get to hPPG, gGPP");
                                                break;
                                        }
                                        break;
                                    case GP:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER");
                                                auto.startNextPath("from CHECK_ORDER hGP, gPPG setRunAtEnd");
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER");
                                                auto.heldstate = PathAuto.BallState.PG;
                                                break;
                                            case PGP:
                                                Log.i("20311", "executing auto.launchBalls2() from CHECK_ORDER hGP, gPGP after end");
                                                auto.launchBalls2();
                                                auto.setRunAtEnd(auto::launchBalls2, "auto::launchBalls2 from CHECK_ORDER hGP, gPGP");
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                break;
                                            case GPP:
                                                Log.i("20311", "ASSERTION ---- Shouldn't get to hGP, gGPP");
                                                break;
                                        }
                                        break;
                                    case PG:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                Log.i("20311", "executing auto.launchBalls2() from CHECK_ORDER hPG, gPPG after end");
                                                auto.launchBalls2();
                                                auto.heldstate = PathAuto.BallState.EMPTY;
                                                break;
                                            case PGP:
                                                auto.setNextPath(SHOOT_TO_SORT, "SHOOT_TO_SORT from CHECK_ORDER hPG, gPGP");
                                                auto.startNextPath("from CHECK_ORDER hPG, gPGP setRunAtEnd");
                                                auto.setNextPath(SORT_TO_SHOOT, "SORT_TO_SHOOT from CHECK_ORDER hPG, gPGP");
                                                auto.heldstate = PathAuto.BallState.GP;
                                                break;
                                            case GPP:
                                                Log.i("20311", "ASSERTION ---- Shouldn't get to hPG, gGPP");
                                                break;
                                        }
                                        break;
                                    case EMPTY:
                                        switch (auto.gamestate) {
                                            case PPG:
                                                switch (lastGroup.get()) {
                                                    case 0:
                                                        lastGroup.set(3);
                                                        auto.setNextPath(COLLECT_GPP_ROW, "COLLECT_GROUP_3 from CHECK_ORDER hEMPTY, gPPG, 0");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPPG, 0");
                                                        auto.heldstate = PathAuto.BallState.PPG;
                                                        break;
                                                    case 3:
                                                        lastGroup.set(1);
                                                        auto.setNextPath(COLLECT_PPG_ROW, "COLLECT_PPG_ROW from CHECK_ORDER hEMPTY, gPPG, 3");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPPG, 3");
                                                        auto.heldstate = PathAuto.BallState.GPP;
                                                        break;
                                                    case 1:
                                                        // Lie about ball order so that we don't have to sort
                                                        lastGroup.set(2);
                                                        auto.setNextPath(COLLECT_PGP_ROW, "COLLECT_GROUP_2 from CHECK_ORDER hEMPTY, gPPG, 1");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPPG, 1");
                                                        auto.heldstate = auto.gamestate;
                                                        break;
                                                    case 2:
                                                        auto.setNextPath(GO_TO_SQUARE, "GO_TO_SQUARE from CHECK_ORDER hEMPTY, gPPG, 2");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPPG, 2");
                                                        break;
                                                }
                                                break;
                                            case PGP:
                                                switch (lastGroup.get()) {
                                                    case 0:
                                                        lastGroup.set(2);
                                                        auto.setNextPath(COLLECT_PGP_ROW, "COLLECT_GROUP_2 from CHECK_ORDER hEMPTY, gPGP, 0");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPGP, 0");
                                                        auto.heldstate = PathAuto.BallState.PGP;
                                                        break;
                                                    case 2:
                                                        lastGroup.set(3);
                                                        auto.setNextPath(COLLECT_GPP_ROW, "COLLECT_GROUP_3 from CHECK_ORDER hEMPTY, gPGP, 2");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPGP, 2");
                                                        auto.heldstate = PathAuto.BallState.PPG;
                                                        break;
                                                    case 3:
                                                        lastGroup.set(1);
                                                        auto.setNextPath(COLLECT_PPG_ROW, "COLLECT_PPG_ROW from CHECK_ORDER hEMPTY, gPGP, 3");
                                                        // Lie about ball order so that we don't have to sort
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPGP, 3");
                                                        auto.heldstate = auto.gamestate;
                                                        break;
                                                    case 1:
                                                        auto.setNextPath(GO_TO_SQUARE, "GO_TO_SQUARE from CHECK_ORDER hEMPTY, gPGP, 1");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPGP, 1");
                                                        break;
                                                }
                                                break;
                                            case GPP:
                                                switch (lastGroup.get()) {
                                                    case 0:
                                                        lastGroup.set(1);
                                                        auto.setNextPath(COLLECT_PPG_ROW, "COLLECT_PPG_ROW from CHECK_ORDER hEMPTY, gGPP, 0");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPP, 0");
                                                        auto.heldstate = PathAuto.BallState.GPP;
                                                        break;
                                                    case 1:
                                                        lastGroup.set(2);
                                                        auto.setNextPath(COLLECT_PGP_ROW, "COLLECT_GROUP_2 from CHECK_ORDER hEMPTY, gGPP, 1");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPP, 1");
                                                        auto.heldstate = PathAuto.BallState.PGP;
                                                        break;
                                                    case 2:
                                                        lastGroup.set(3);
                                                        auto.setNextPath(COLLECT_GPP_ROW, "COLLECT_GROUP_3 from CHECK_ORDER hEMPTY, gGPP, 2");
                                                        // Lie about ball order so that we don't have to sort
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPP, 2");
                                                        auto.heldstate = auto.gamestate;
                                                        break;
                                                    case 3:
                                                        auto.setNextPath(GO_TO_SQUARE, "GO_TO_SQUARE from CHECK_ORDER hEMPTY, gGPP, 3");
                                                        auto.startNextPath("from CHECK_ORDER hEMPTY, gPP, 3");
                                                        break;
                                                }
                                                break;
                                        }
                                        break;
                                }
                            }
                            , "complex behavior from CHECK_ORDER"))
                    .build();
        }

        SORT_SCAN_FORWARDS = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.236, 118.145), new Pose(126.274, 119.327)) // 116
                )
                    .setConstantHeadingInterpolation(Math.toRadians(20))
                    .addParametricCallback(0.5, () -> auto.setNextPath(SORT_SCAN_BACKWARDS, "SORT_SCAN_BACKWARDS from SORT_SCAN_FORWARDS"))
                    .addParametricCallback(0.5, () -> auto.setRunAtEnd(()->auto.startNextPath("from SORT_SCAN_BACKWARDS setRunAtEnd"), "auto::startNextPath from SORT_SCAN_FORWARDS"))
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
                        new BezierLine(new Pose(123.236, 118.145), new Pose(98, 98))
                )
                    .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(45))
                    .addParametricCallback(0.1, () -> auto.setNextPath(CHECK_ORDER, "CHECK_ORDER from SORT_TO_SHOOT"))
                    .addParametricCallback(0.2, () -> auto.setRunAtEnd(()->auto.startNextPath("from SORT_TO_SHOOT setRunAtEnd"), "auto::startNextPath from SORT_TO_SHOOT"))
                    .addParametricCallback(0.9, () -> auto.spinUp("SORT_TO_SHOOT Get ready to shoot"))
                .build();

        SHOOT_TO_SORT = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(98, 98), new Pose(123.236, 118.145))
                )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(20))
                    .addParametricCallback(0.1, () -> auto.spinHalf("SHOOT_TO_SORT slow spin for sorting"))
                    .addParametricCallback(0.2, () -> auto.setNextPath(SORT_TO_SHOOT, "SHOOT_TO_SORT from SHOOT_TO_SORT")) // set this to what hapens after cycling
                    .addParametricCallback(0.9, () -> auto.setRunAtEnd(auto::cycle1, "auto::sort from SHOOT_TO_SORT")) // rename this cyclen, have cyclen remember next path and change it to sort_scan_forwards there instead if you want to cycle, or restore and call next path if done.
                .build();


    }

    static Runnable andThen(Runnable ...runnables) {
        return () -> {for (Runnable runnable : runnables) runnable.run();};
    }
}
