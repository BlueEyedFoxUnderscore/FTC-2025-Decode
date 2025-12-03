package org.firstinspires.ftc.teamcode.opmodes.auto;

import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.max;
import static java.lang.StrictMath.nextAfter;

import android.util.Log;

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
import com.pedropathing.util.PoseHistory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

    public void launchBalls1() {
        RobotContainer.LOADER.setLoaded(1);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        state = AutoState.SHOOTING;
    }

    public  enum AutoState {SHOOTING, SAMPLE_TAGS, SAMPLE_TAGS_2, SAMPLE_TAGS_3, READY, GET_TAG_ID, GET_TAG_ID_2, CYCLING, CYCLING_2, CYCLING_3, CYCLING_READY, SAMPLE_TAGS_4}
    public enum BallState {PPG, PGP, GP, PG, EMPTY, GPP}

    private AutoState state = AutoState.READY;
    private AutoState laststate = null;

    String runAtEndName=null;
    public void setRunAtEnd(Runnable runAtEnd, String name) {
        Log.i("20311", "at end: " + name);
        Log.i("20311", "   Game ball order: " + gamestate);
        Log.i("20311", "   Held ball order: " + heldstate);
        Log.i("20311", "   Auto state machine state: " + state);
        this.runAtEnd = runAtEnd;
        this.runAtEndName = name;
    }

    private Runnable runAtEnd = null, tempRun = null; // () -> {};
    BallState gamestate = null;
    BallState heldstate = BallState.PGP;
    int tag;
    int ppg = 0;
    int pgp = 0;
    int gpp = 0;

    private PathChain afterCycle = null;


    @Override
    public void runOpMode() throws InterruptedException {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        // follower.setStartingPose(new Pose(121.7, 124.1, Math.toRadians(36))); //set your starting pose
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
        int mediumBeep = hardwareMap.appContext.getResources().getIdentifier("beep", "raw", hardwareMap.appContext.getPackageName());
        int lowBattery = hardwareMap.appContext.getResources().getIdentifier("lowbattery", "raw", hardwareMap.appContext.getPackageName());

        RedAuto.init(follower, this);
        setNextPath(RedAuto.READ_TO_REORIENT, "READ_TO_REORIENT from chosen AUTO");
        if(hardwareMap.get(VoltageSensor.class, "Control Hub").getVoltage() < 12.5) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, lowBattery);
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, lowBattery);
            telemetry.addLine("");
            telemetry.addLine("            WARNING BATTERY VOLTAGE IS TOO");
            telemetry.addLine("░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░");
            telemetry.addLine("░░██░░░░░░░░█████░░░░██░░░░░██░░");
            telemetry.addLine("░░██░░░░░░░██░░░██░░░██░░█░░██░░");
            telemetry.addLine("░░██░░░░░░██░░░░░██░░██░███░██░░");
            telemetry.addLine("░░██░░░░░░░██░░░██░░░████░████░░");
            telemetry.addLine("░░███████░░░█████░░░░███░░░███░░");
            telemetry.addLine("░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░");
            telemetry.addLine("");
        }
        Log.i("20311", "Battery tested");
        while (!isStarted() && !isStopRequested()) {
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
            telemetry.update();
        }
        follower.setStartingPose(nextPath.firstPath().getPose(0).withHeading(nextPath.firstPath().getHeadingGoal(0)));
        startNextPath("Initialize");
        //follower.followPath(RedAuto.APRIL_TEST);

        LinkedList<Pose> samples = new LinkedList<>();
        ElapsedTime elapsedTime = new ElapsedTime();

        int successfulRecogs = 0;
        int maxsamples=0;
        while ((!done) && opModeIsActive()) {
            //telemetry.addData("runAtEnd", runAtEnd != null? runAtEnd.toString(): "null");
            //telemetry.addData("nextPath", nextPath != null? nextPath.toString(): "null");
            follower.update();
            if (runAtEnd != null && !follower.isBusy()) {
                tempRun = runAtEnd;
                runAtEnd = null;
                Log.i("20311", "EXECUTING " + runAtEndName);
                tempRun.run();
            }

            ///  S t a t e m a c h i n e
            if(laststate != state) {
                laststate=state;
                Log.i("20311", "AUTOSTATE CHANGED TO: "+state);
            }
            switch (state) {
                case READY:
                    break;
                case SHOOTING:
                    if (RobotContainer.LOADER.doneFiring()) {
                        RobotContainer.LOADER.cancelLaunch();
                        spinDown("state machine SHOOTING");
                        startNextPath("autostatemachine = SHOOTING");
                        state = AutoState.READY;
                    }
                    break;
                case GET_TAG_ID:
                    elapsedTime.reset();
                    samples.clear();
                    gpp = pgp = ppg = 0;
                    state = AutoState.GET_TAG_ID_2;
                    break;
                case GET_TAG_ID_2:
                    LLResult result = camera.getLatestResult();
                    if (!result.getFiducialResults().isEmpty()) {
                        switch (result.getFiducialResults().get(0).getFiducialId()) {
                            case 21:
                                gpp += 1;
                                break;
                            case 22:
                                pgp += 1;
                                break;
                            case 23:
                                ppg += 1;
                                break;
                        }
                    }
                    if (elapsedTime.seconds() > 0.5) {
                        if (Math.max(Math.max(gpp, pgp), ppg) == gpp) {
                            gamestate = BallState.GPP;
                        }
                        if (Math.max(Math.max(gpp, pgp), ppg) == pgp) {
                            gamestate = BallState.PGP;
                        }
                        if (Math.max(Math.max(gpp, pgp), ppg) == ppg) {
                            gamestate = BallState.PPG;
                        }
                        state = AutoState.READY;
                    }
                    break;
                case SAMPLE_TAGS:
                    setFollowerMaxPower(0);
                    elapsedTime.reset();
                    samples.clear();
                    state = AutoState.SAMPLE_TAGS_2;
                case SAMPLE_TAGS_2: /// Get apriltag samples
                    if (elapsedTime.seconds() > 0.3) {
                        elapsedTime.reset();
                        state = AutoState.SAMPLE_TAGS_3;
                        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mediumBeep);
                    }
                    break;
                case SAMPLE_TAGS_3: /// Get apriltag samples
                    addSampleIfAvailable(samples);
                    if (elapsedTime.seconds() > 0.5) {
                        state = AutoState.SAMPLE_TAGS_4;
                        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mediumBeep);
                    }
                    else break;
                case SAMPLE_TAGS_4:
                    if(samples.size() >= 8) {
                        follower.setPose(getAverageOfBest(samples, 5));
                        ++successfulRecogs;
                    }
                    startNextPath("autostatemachine = SAMPLE_TAGS_4");
                    state = AutoState.READY;
                    setFollowerMaxPower(1);
                    break;
                case CYCLING_READY:
                    if (!follower.isBusy()) {
                        if (cycleCount > 1) {
                            --cycleCount;
                            state = AutoState.CYCLING;
                        } else {
                            nextPath = afterCycle;
                            state = AutoState.READY;
                            startNextPath("autostatemachine = CYCLING_READY");
                        }
                    }
                    break;
                case CYCLING:
                    state = AutoState.CYCLING_2;
                    RobotContainer.LOADER.launch();
                    break;
                case CYCLING_2:
                    if (RobotContainer.LOADER.doneFiring()) {
                        RobotContainer.LOADER.cancelLaunch();
                        RobotContainer.LOADER.intake();
                        spinDown("State machine CYCLING_2");
                        elapsedTime.reset();
                        state = AutoState.CYCLING_3;
                        nextPath = RedAuto.SORT_SCAN_FORWARDS;
                    }
                    break;
                case CYCLING_3:
                    if (elapsedTime.seconds() > 1) {
                        state = AutoState.CYCLING_READY;
                        startNextPath("autostatemachine = CYCLING_3");
                    }
                    break;
                default: int fucksUpTheProgram = 0 / 0;
            }

            telemetry.addData("b a l l", gamestate != null? gamestate.name(): "null");
            telemetry.addData("ppg", ppg);
            telemetry.addData("pgp", pgp);
            telemetry.addData("gpp", gpp);

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
            if (samples.size() > maxsamples) maxsamples = samples.size();
            telemetry.addData("Apriltag Samples", maxsamples);
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
            telemetry.update();
        }

    }

    public void setState(AutoState state) {
        this.state = state;
    }

    public void setNextPath(PathChain nextPath, String name) {
        this.nextPath = nextPath;
        Log.i("20311", "next path: " + name);
        Log.i("20311", "   Game ball order: " + gamestate);
        Log.i("20311", "   Held ball order: " + heldstate);
        Log.i("20311", "   Auto state machine state: " + state);
        nextPathName=name;
    }

    private PathChain nextPath;
    String nextPathName=null;

    void startNextPath(String note) {
        if (nextPath != null) {
            Log.i("20311", "PATHCHAIN: "+nextPathName+" started by startNextPath("+note+")");
            follower.followPath(nextPath);
            nextPath = null;
        }
        else Log.i("20311", "NO NEXT PATHCHAIN after startNextPath("+note+")");
    }

    void spinUp(String note) {
        RobotContainer.FLYWHEEL.setRequested(2850, note);
    }

    void spinDown(String note) {
        RobotContainer.FLYWHEEL.setRequested(0, note);
    }

    void spinHalf(String note) {
        RobotContainer.FLYWHEEL.setRequested(900, note);
    }

    int cycleCount = 1;

    void cycle1() {
        RobotContainer.LOADER.setLoaded(1);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        cycleCount = 1;
        afterCycle = nextPath;
        state = AutoState.CYCLING;
    }

    void cycle2() {
        RobotContainer.LOADER.setLoaded(1);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        cycleCount = 2;
        afterCycle = nextPath;
        state = AutoState.CYCLING;
    }

    void launchBalls2() {
        RobotContainer.LOADER.setLoaded(2);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        state = AutoState.SHOOTING;
    }

    void launchBalls3() {
        RobotContainer.LOADER.setLoaded(3);
        RobotContainer.LOADER.resetShots();
        RobotContainer.LOADER.launch();
        state = AutoState.SHOOTING;
        Log.i("20311", "EXECUTING auto::launchBalls3");
    }

    void reorient() {
        state = AutoState.SAMPLE_TAGS;
    }

    BezierLine stayAt(Pose location) {
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
        if (result.isValid() && result.getTimestamp() != lastSample) {
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

    void setFollowerMaxPower(double power) {
        follower.setMaxPower(power);
    }
    //*/

    void cancelIntake() {

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
