package org.firstinspires.ftc.teamcode.opmodes.auto;

import static java.lang.StrictMath.PI;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

import java.util.LinkedList;

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
        RESUME,
        SHOOTING,
        SAMPLE_TAGS,
        SAMPLE_TAGS_2,
        SAMPLE_TAGS_3,
        READY, SAMPLE_TAGS_4;
    }

    private static final class Paths {
        public static final Path BACK_UP_FROM_START = new Path(new BezierLine (new Pose(121.716, 124.080), new Pose(95.381,  104.835)));
        public static final Path GROUP_APPROACH_1   = new Path(new BezierCurve(new Pose(95.381,  104.835), new Pose(85.758,  82.382), new Pose(97.069,  83.564)));
        public static final Path GROUP_PICKUP_1     = new Path(new BezierLine (new Pose(97.069,  83.564),  new Pose(120.093, 83.564)));
        public static final Path GROUP_SHOOT_1      = new Path(new BezierLine (new Pose(120.093, 83.564),  new Pose(97.069,  104.835)));
        public static final Path GROUP_APPROACH_2   = new Path(new BezierCurve(new Pose(97.069,  104.835), new Pose(86.771,  59.254), new Pose(102.134, 59.254)));
        public static final Path GROUP_PICKUP_2     = new Path(new BezierLine (new Pose(102.134, 59.254),  new Pose(120.586, 59.254)));
        public static final Path GROUP_SHOOT_2_A    = new Path(new BezierLine (new Pose(120.586, 59.254),  new Pose(110.586, 59.254)));
        public static final Path GROUP_SHOOT_2      = new Path(new BezierLine (new Pose(110.586, 59.254),  new Pose(95.381,  104.835)));
        public static final Path GROUP_APPROACH_3   = new Path(new BezierCurve(new Pose(95.381,  104.835), new Pose(88.797,  35.620), new Pose(101.627, 35.620)));
        public static final Path GROUP_PICKUP_3     = new Path(new BezierLine (new Pose(101.627, 35.620),  new Pose(125.093, 35.620)));
        public static final Path GROUP_SHOOT_3      = new Path(new BezierLine (new Pose(125.093, 35.620),  new Pose(95.550,  104.666)));
        public static final Path GO_TO_SQUARE       = new Path(new BezierLine (new Pose(95.550,  104.666), new Pose(105.341, 33.257)));
    }

    private AutoState state = AutoState.RESUME;


    @Override
    public void runOpMode() throws InterruptedException {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.7, 124.1, Math.toRadians(36))); //set your starting pose
        follower.activateAllPIDFs();
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

        PathChain path1 = follower.pathBuilder()
                .addPath(Paths.BACK_UP_FROM_START)
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(0, this::spinUp)
                    .addParametricCallback(0.99, this::launchBalls2)
                .addPath(stayAt(Paths.BACK_UP_FROM_START.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .addParametricCallback(0, this::reorient)
                .addPath(Paths.GROUP_APPROACH_1)
                    .setLinearHeadingInterpolation(Math.toRadians(36), 0)
                    .addParametricCallback(0, RobotContainer.LOADER::intake)
                    .addParametricCallback(0.9, () -> this.setFollowerMaxPower(0.5))
                .addPath(Paths.GROUP_PICKUP_1)
                    .setConstantHeadingInterpolation(0)
                    .addParametricCallback(0.99, () -> this.setFollowerMaxPower(1.0))
                .addPath(Paths.GROUP_SHOOT_1)
                    .setLinearHeadingInterpolation(0, Math.toRadians(36))
                    .addParametricCallback(0, this::spinUp)
                    .addParametricCallback(0.9, RobotContainer.LOADER::cancelIntake)
                    .addParametricCallback(0.99, this::launchBalls3)
                .addPath(Paths.GROUP_APPROACH_2)
                    .setLinearHeadingInterpolation(Math.toRadians(36), 0)
                    .addParametricCallback(0.1, RobotContainer.LOADER::cancelLaunch)
                    .addParametricCallback(0.2, RobotContainer.LOADER::intake)
                    .addParametricCallback(0.9, () -> this.setFollowerMaxPower(0.5))
                .addPath(Paths.GROUP_PICKUP_2)
                    .setConstantHeadingInterpolation(0)
                    .addParametricCallback(0.99, () -> this.setFollowerMaxPower(1.0))
                .addPath(Paths.GROUP_SHOOT_2_A)
                    .setConstantHeadingInterpolation(0)
                    .addParametricCallback(0.2, this::spinUp)
                .addPath(Paths.GROUP_SHOOT_2)
                    .setLinearHeadingInterpolation(0, Math.toRadians(36))
                    .addParametricCallback(0.8, RobotContainer.LOADER::cancelIntake)
                    .addParametricCallback(0.99, this::launchBalls3)
                .build();

        follower.followPath(path1);

        LinkedList<Pose> samples = new LinkedList<>();
        ElapsedTime elapsedTime = new ElapsedTime();

        int successfulRecogs = 0;

        while ((!done) && opModeIsActive()) {

            follower.update();

            ///  S t a t e m a c h i n e
            switch (state) {
                case RESUME:
                    follower.resumePathFollowing();
                    state = AutoState.READY;
                case READY:
                    break;
                case SHOOTING:
                    if (RobotContainer.LOADER.doneFiring()) {
                        spinDown();
                        RobotContainer.LOADER.cancelLaunch();
                        state = AutoState.RESUME;
                    }
                    break;
                case SAMPLE_TAGS:
                    elapsedTime.reset();
                    samples.clear();
                    state = AutoState.SAMPLE_TAGS_2;
                case SAMPLE_TAGS_2: /// Get apriltag samples
                    if (elapsedTime.seconds() > 2) {
                        elapsedTime.reset();
                        state = AutoState.SAMPLE_TAGS_3;
                    }
                    break;
                case SAMPLE_TAGS_3: /// Get apriltag samples
                    addSampleIfAvailable(samples);
                    if (elapsedTime.seconds() > 0.6) {
                        state = AutoState.SAMPLE_TAGS_4;
                    }
                    else break;
                case SAMPLE_TAGS_4:
                    if(samples.size() >= 8) {
                        follower.setPose(getAverageOfBest(samples, 5));
                        ++successfulRecogs;
                    }
                    state = AutoState.RESUME;
                    break;
                default: int fucksUpTheProgram = 0 / 0;
            }

            isStoppedGate.update();
            if (isStoppedGate.trueForAtLeast(2)) {
                isStoppedGate.reset();
                follower.setPose(getRobotPoseFromCamera());
                // telemetry.addData("ConvertedCameraPose", getRobotPoseFromCamera().toString());
                // telemetry.addData("BotPose", follower.getPose().toString());
            }

            RobotContainer.update();

            ///  Telemetry
            // telemetry.addData("Is accepted", acceptedPose);
            // telemetry.addData("Was called", wasCalled);
            // telemetry.addData("Can be called", isStoppedGate.trueForAtLeast(2));
            // telemetry.addData("Velocity", follower.getVelocity().getMagnitude());
            telemetry.addData("Successful localizations", successfulRecogs);
            telemetry.addData("AutoState", state.name());
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
            telemetry.update();
        }
    }

    private void spinUp() {
        RobotContainer.FLYWHEEL.setRequested(2600, 2400); //2800 was original
    }

    private void spinDown() {
        RobotContainer.FLYWHEEL.setRequested(0, 0);
    }

    private void holdEndOfPath() {
        follower.pausePathFollowing();
        follower.holdPoint(follower.getCurrentPath().endPose());
    }

    private void launchBalls2() {
        holdEndOfPath();
        RobotContainer.LOADER.setLoaded(2);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        state = AutoState.SHOOTING;
    }

    private void launchBalls3() {
        holdEndOfPath();
        RobotContainer.LOADER.setLoaded(3);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        state = AutoState.SHOOTING;
    }

    private void reorient() {
        holdEndOfPath();
        state = AutoState.SAMPLE_TAGS;
    }

    private BezierLine stayAt(Pose location) {
        return new BezierLine(location, location.withY(location.getY() + 1./100.));
    }
    private static final double INCHES_PER_METER = 39.3701;

    private void requestRecalibration() {

    }

    private boolean isIn(LinkedList<Pose> samples, Pose checkAgainstPose) {
        for (Pose checkForPose: samples) {
            if (checkForPose.roughlyEquals(checkAgainstPose, 0)) return true;
        }
        return false;
    }

    private double lastSample = 0;

    private void addSampleIfAvailable(LinkedList<Pose> poses) {
        LLResult result = camera.getLatestResult();
        if (result.isValid() & result.getTimestamp() != lastSample) {
            lastSample = result.getTimestamp();
            Pose3D botpose = result.getBotpose();
            Position position = botpose.getPosition();
            YawPitchRollAngles orientation = botpose.getOrientation();
            poses.add(new Pose(
                    72.0+(position.y * INCHES_PER_METER),
                    72.0-(position.x * INCHES_PER_METER),
                    orientation.getYaw(AngleUnit.RADIANS) - PI/2,
                    PedroCoordinates.INSTANCE));
        }
    }

    private double getSquaredError(Pose pose1, Pose pose2) {
        return  ((pose1.getX() - pose2.getX()) * (pose1.getX() - pose2.getX())) +
                ((pose1.getY() - pose2.getY()) * (pose1.getY() - pose2.getY())) +
                ((pose1.getHeading() - pose2.getHeading()) * (pose1.getHeading() - pose2.getHeading()));
    }

    private void removeWorst(LinkedList<Pose> poses) {
        Pose worst = null; // if null print warning
        double worstDifference = 0;
        for (Pose pose1: poses) {
            double difference = 0;
            for (Pose pose2: poses) {
                difference += getSquaredError(pose1, pose2);
            }
            if (difference > worstDifference) {
                worst = pose1;
                worstDifference = difference;
            }
        }
        if (worst == null) {
            telemetry.addLine("WARNING: received identical poses.");
            worst = poses.getFirst();
        }
        poses.remove(worst);
    }

    private Pose averagePose(LinkedList<Pose> poses) {
        double x = 0;
        double y = 0;
        double h = 0;
        for (Pose pose: poses) {
            x += pose.getX();
            y += pose.getY();
            h += pose.getHeading();
        }
        double l = poses.size();
        return new Pose(x/l, y/l, h/l);
    }

    private Pose getAverageOfBest(LinkedList<Pose> poses, int best) {
        while (poses.size() > best) {
            removeWorst(poses);
        }
        return averagePose(poses);
    }

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

    private void setFollowerMaxPower(double power) {
        follower.setMaxPower(power);
    }
    //*/
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
