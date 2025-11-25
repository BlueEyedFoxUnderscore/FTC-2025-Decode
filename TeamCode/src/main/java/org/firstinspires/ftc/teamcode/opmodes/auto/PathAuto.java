package org.firstinspires.ftc.teamcode.opmodes.auto;

import static java.lang.StrictMath.PI;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

@Autonomous
public class PathAuto extends LinearOpMode {
    boolean done = false;
    Follower follower;
    private Limelight3A camera; //any camera here
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose().withY(20); //Put the target location here
    private boolean acceptedPose = false;
    private boolean wasCalled = false;

    private enum AutoState {
        READY,
        SHOOTING,
    }

    private AutoState state = AutoState.READY;


    @Override
    public void runOpMode() throws InterruptedException {

        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117.834, 129.651).withHeading(Math.toRadians(36))); //set your starting pose
        camera.start();
        RobotContainer.init(hardwareMap, telemetry);
        Gate isStoppedGate = new Gate(() -> follower.getVelocity().getMagnitude() < 10);
        Drawing.init();
        try {
            Drawing.drawDebug(follower);
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
        //waitForStart();
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
        }

        PathChain path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(121.71629542790153, 124.07971864009379), new Pose(91.66705744431418, 101.28956623681125))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))
                .addParametricCallback(0, this::spinUp)
                .addParametricCallback(1, this::launchBalls)
                .addPath(
                        new BezierLine(new Pose(91.66705744431418, 101.28956623681125), new Pose(92.00468933177022, 83.22626025791324))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        follower.followPath(path1);

        while ((!done) && opModeIsActive()) {
            switch (state) {
                case READY:
                    break;
                case SHOOTING:
                    if(RobotContainer.LOADER.doneFiring()) {
                        spinDown();
                        follower.resumePathFollowing();
                        state = AutoState.READY;
                    }
                    break;
                default:
                    int i = 0 / 0;
                    break;
            }

            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
            telemetry.update();

            isStoppedGate.update();
            RobotContainer.FLYWHEEL.update();
            follower.update();
            spinUp();
            RobotContainer.LOADER.update();
            if (isStoppedGate.trueForAtLeast(2)) {
                isStoppedGate.reset();
                follower.setPose(getRobotPoseFromCamera());
                telemetry.addData("ConvertedCameraPose", getRobotPoseFromCamera().toString());
                telemetry.addData("BotPose", follower.getPose().toString());
            }
            telemetry.addData("Is accepted", acceptedPose);
            telemetry.addData("Was called", wasCalled);
            telemetry.addData("Can be called", isStoppedGate.trueForAtLeast(2));
            telemetry.addData("Velocity", follower.getVelocity().getMagnitude());


//            if (!following) {
//                follower.followPath(
//                        follower.pathBuilder()
//                                .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
//                                .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
//                                .addParametricCallback(0, RobotContainer.LOADER::intake)
//                                .addParametricCallback(1, RobotContainer.LOADER::cancelIntake)
//                                .build()
//                );
//            }
            //This uses the aprilTag to relocalize your robot
            //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos

        }
    }

    private static final double INCHES_PER_METER = 39.3701;
    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        LLResult result = camera.getLatestResult();
        wasCalled = true;
        if (result != null) {
            if (result.isValid()) {
                acceptedPose = true;
                Pose3D botpose = result.getBotpose();
                Position position = botpose.getPosition();
                YawPitchRollAngles orientation = botpose.getOrientation();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("CameraPose", botpose.toString());
                return new Pose(
                        72.0+(position.y * INCHES_PER_METER),
                        72.0-(position.x * INCHES_PER_METER),
                        orientation.getYaw(AngleUnit.RADIANS) - PI/2,
                        PedroCoordinates.INSTANCE);
            }
        }
        return follower.getPose();
    }
    //*/

    private void spinUp() {
        RobotContainer.FLYWHEEL.setRequested(2800, 2400);
    }

    private void spinDown() {
        RobotContainer.FLYWHEEL.setRequested(0, 0);
    }

    private void launchBalls() {
        RobotContainer.LOADER.setLoaded(3);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        state = AutoState.SHOOTING;
        follower.pausePathFollowing();
    }
}

class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );
    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }
    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);
        sendPacket();
    }
    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }
    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }
    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }
    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }
    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);
        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }
    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }
    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
