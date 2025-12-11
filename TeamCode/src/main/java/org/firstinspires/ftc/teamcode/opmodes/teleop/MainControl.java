package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.atan2;
import static java.lang.StrictMath.max;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.LinkedList;


@TeleOp//(name = "***FTC OpMode 2", group = "TeleOp")

public class MainControl extends OpMode {
    int mediumBeep;
    int lowBattery;
    FtcDashboard dash;
    ElapsedTime imuTimeout = new ElapsedTime();
    private Follower follower;
    static TelemetryManager telemetryM;


    //GoBildaPinpointDriver odometry1;
    //GoBildaPinpointDriver odometry2;

    //private MotorGroup shooterGroup;
    private boolean canright = true;
    private boolean canleft = true;
    private final boolean canup = true;
    private final boolean candown = true;
    @Override
    public void init_loop() {
        follower.update();
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    @Override
    public void init() {
        mediumBeep = hardwareMap.appContext.getResources().getIdentifier("beep", "raw", hardwareMap.appContext.getPackageName());
        lowBattery = hardwareMap.appContext.getResources().getIdentifier("lowbattery", "raw", hardwareMap.appContext.getPackageName());
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
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.start();
        dash = FtcDashboard.getInstance();

        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(new Pose());


        follower.startTeleopDrive(true);
        follower.update();
        imuTimeout.reset();
        RobotContainer.init(hardwareMap, telemetry);

        telemetry.addLine("Initialized - waiting for IMU calibration...");
        telemetry.update();
        Drawing.init();

        follower.setStartingPose(/*Storage.ORIENTPOSE == null?*/ new Pose(0, 0, 0)/*: Storage.ORIENTPOSE*/);

        //offset = -Storage.ORIENTPOSE.getHeading();

        //odometry1.setOffsets(-84.0, -168.0, DistanceUnit.MM); // T U N E   T H E S E
        //odometry1.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //odometry2.setOffsets(-84.0, -168.0, DistanceUnit.MM); // T U N E   T H E S E
        //odometry2.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    private double flywheelSpeed = 2600;
    TelemetryPacket packet;
    double offset = PI;

    double testp = 65;

    ElapsedTime looptime = new ElapsedTime();
    @Override
    public void loop() {

        packet = new TelemetryPacket();
        updateDrive();
        telemetry.addData("Requested Speed", flywheelSpeed);
        telemetry.addData("Flywheel Speed 1", RobotContainer.FLYWHEEL.getSpeed1());
        telemetry.addData("Flywheel stable", flywheelStable());
        packet.put("Flywheel Speed 1", RobotContainer.FLYWHEEL.getSpeed1());
        packet.put("Flywheel Speed 2", RobotContainer.FLYWHEEL.getSpeed2());
        packet.put("flywheel PID P", testp);
        packet.put("loop time", looptime.milliseconds());
        looptime.reset();
        Drawing.drawDebug(follower);
        // Drawing.sendPacket();
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
        telemetry.addLine(String.valueOf(RobotContainer.FLYWHEEL.getSpeed1()));
        telemetry.clear();


        dash.sendTelemetryPacket(packet);
//        gate.setPosition(gamepad1.right_trigger);
    }

    private final LinkedList<Pose> samples = new LinkedList<>();
    private final ElapsedTime elapsedTime = new ElapsedTime();

    void setFollowerMaxPower(double maxPower) {
        follower.setMaxPower(maxPower);
    }


    private double lastSample = 0;
    private Limelight3A camera;
    private static final double INCHES_PER_METER = 39.3701;

    private boolean resultIsValid(LLResult result) {
        for (LLResultTypes.FiducialResult results: result.getFiducialResults()) {
            if (!(results.getFiducialId() == 20 || results.getFiducialId() == 24)) return false;
        }
        return true;
    }

    private void addSampleIfAvailable(LinkedList<Pose> poses) {
        LLResult result = camera.getLatestResult();
        if (!result.getFiducialResults().isEmpty()) if (result.isValid() && result.getTimestamp() != lastSample && resultIsValid(result)) {
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




    AprilState aprilState = AprilState.READY;

    enum AprilState {SAMPLE_TAGS, SAMPLE_TAGS_2, SAMPLE_TAGS_3, SAMPLE_TAGS_4, READY}

    boolean shouldSetHoldPointInitial = true, shouldSetHoldPointSecondary = true;
    ElapsedTime holdTimeout = new ElapsedTime();
    double lastHeading = 0;

    PIDController headingController = new PIDController(1.5, 0, 0.1);
    boolean square = false;
    boolean toGo = false;

    public static final Pose BLUE_PARKING = new Pose((72d+24d)+(18/2)+(18-16)-.5, (24d)+18d/2d, Math.toRadians(90));
    public static final Pose RED_PARKING = new Pose((72d-24d)-(18/2)-(18-16)+.5, (24d)+18d/2d, Math.toRadians(90));

    private void updateDrive() {
        double headingCorrection = headingController.calculate(-AngleUnit.normalizeRadians(atan2(144-9 - follower.getPose().getY(),144-9 - follower.getPose().getX())-follower.getHeading()));
        if (gamepad1.squareWasPressed()) {
            follower.holdPoint(RED_PARKING);
            square = true;
            aprilState = AprilState.READY;
            Log.i("20311", "SQUARE ON due to SQUARE WAS PRESSED");
        }

        if (gamepad1.squareWasReleased()) {
            follower.startTeleopDrive(true);
            square = false;
            Log.i("20311", "SQUARE OFF due to SQUARE WAS RELEASED");
        }

        if (Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01 || gamepad1.right_trigger > 0.1) {
            square = false;
            Log.i("20311", "SQUARE OFF due to STICK MOVEMENT");
        }

        //telemetry.addData("Square On", square);

        if (!square) {
            //telemetry.addData("Can Drive", (Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01 || gamepad1.right_trigger > 0.1));
            //telemetry.addData("left stick y", gamepad1.left_stick_y);
            //telemetry.addData("left stick x", gamepad1.left_stick_x);
            if (Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01 || gamepad1.right_trigger > 0.1) {
                aprilState = AprilState.READY;
                if (!shouldSetHoldPointInitial) follower.startTeleopDrive(true);
                //telemetry.addLine("Driving!");
                follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                 gamepad1.right_trigger > 0.1 ? headingCorrection : -gamepad1.right_stick_x/*-gamepad1
                 .right_stick_x*/, false, offset);
                shouldSetHoldPointInitial = shouldSetHoldPointSecondary = true;
            } else {
                if (shouldSetHoldPointInitial) {
                    holdTimeout.reset();
                    lastHeading = follower.getVelocity().getTheta();
                    shouldSetHoldPointInitial = false;
                    follower.holdPoint(follower.getPose());
                } else if ((holdTimeout.seconds() > 0.2) && shouldSetHoldPointSecondary) {
                    shouldSetHoldPointSecondary = false;
                    follower.holdPoint(follower.getPose());
                    aprilState = AprilState.SAMPLE_TAGS;
                }
            }

            if (gamepad1.dpad_right) {
                if (canright) {
                    canright=false;
                    flywheelSpeed += 100;
                }
            } else canright = true;
            if (gamepad1.dpad_left) {
                if (canleft) {
                    canleft = false;
                    flywheelSpeed -= 100;
                }
            } else canleft = true;
            if (gamepad1.dpadUpWasPressed()) {
                flywheelSpeed += 20;
            }
            if (gamepad1.dpadDownWasPressed()) {
                flywheelSpeed -= 20;
            }
        } else {
            if (gamepad1.dpadRightWasPressed()) {
                follower.setPose(follower.getPose().withY(follower.getPose().getY() + 0.2));
                follower.holdPoint(RED_PARKING);
            }

            if (gamepad1.dpadLeftWasPressed()) {
                follower.setPose(follower.getPose().withY(follower.getPose().getY() - 0.2));
                follower.holdPoint(RED_PARKING);
            }

            if (gamepad1.dpadDownWasPressed()) {
                follower.setPose(follower.getPose().withX(follower.getPose().getX() + 0.2));
                follower.holdPoint(RED_PARKING);
            }

            if (gamepad1.dpadUpWasPressed()) {
                follower.setPose(follower.getPose().withX(follower.getPose().getX() - 0.2));
                follower.holdPoint(RED_PARKING);
            }
        }
        switch (aprilState) {
            case READY:
                elapsedTime.reset();
                samples.clear();
                break;
            case SAMPLE_TAGS:
                elapsedTime.reset();
                samples.clear();
                aprilState = AprilState.SAMPLE_TAGS_2;
            case SAMPLE_TAGS_2: /// Get apriltag samples
                elapsedTime.reset();
                aprilState = AprilState.SAMPLE_TAGS_3;
                break;
            case SAMPLE_TAGS_3: /// Get apriltag samples
                addSampleIfAvailable(samples);
                if (samples.size() >= 8) {
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mediumBeep);
                    Pose average = getAverageOfBest(samples, 5);
                    follower.setPose(average);
                    follower.holdPoint(average);
                    aprilState = AprilState.READY;
                }
                else break;
        }
        //telemetry.addData("toGo", toGo);
        //telemetry.addData("joyX", gamepad1.left_stick_x);
        //telemetry.addData("joyY", gamepad1.left_stick_y);
        //telemetry.addData("spin", gamepad1.right_stick_x);
        //telemetry.addData("shouldHold", shouldSetHoldPointInitial);
        telemetry.addData("velocity", follower.getVelocity().getMagnitude());
        //telemetry.addData("joystickIsSwizzle", !(Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01));
        if(distanceIndex >-1) {
            telemetry.addLine("calculated flywheel speed: "+ calculatedFlywheel +"@" + distanceToTarget);
            telemetry.addLine("using range "+ KNOWN_TARGET_SPEEDS[distanceIndex]+"@"+ KNOWN_TARGET_DISTANCES[distanceIndex]+" and "+ KNOWN_TARGET_SPEEDS[distanceIndex +1]+"@"+ KNOWN_TARGET_DISTANCES[distanceIndex +1]);
        }
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

    private void spinUp(String note) {
        RobotContainer.FLYWHEEL.setRequested(flywheelSpeed, note);
    }
    // 2800 2400

    private static final Pose GOAL_POSE = new Pose(144-9, 144-9);

    private double getDistanceToFlywheel () {
        return follower.getPose().distanceFrom(GOAL_POSE) - (1d/2d * 17.2);
    }

    double calculatedFlywheel;
    int distanceIndex =-1;
    double distanceToTarget;
    private static final int[] KNOWN_TARGET_DISTANCES = {  18,   20,   24,   44,   60,   76,   90,  103,  108,  117,  137};
    private static final int[] KNOWN_TARGET_SPEEDS =    {2370, 2360, 2290, 2440, 2620, 2760, 2910, 3060, 3160, 3300, 3500};
    private void spinByDistance (String note) {
        distanceToTarget = getDistanceToFlywheel();
        for (int i = 0; i < KNOWN_TARGET_DISTANCES.length - 1; ++i) {
            if (distanceToTarget > KNOWN_TARGET_DISTANCES[i]) {
                distanceIndex = i;
            }
        }
        if(distanceIndex >-1) {
            calculatedFlywheel = (distanceToTarget - KNOWN_TARGET_DISTANCES[distanceIndex]) / (KNOWN_TARGET_DISTANCES[distanceIndex + 1] - KNOWN_TARGET_DISTANCES[distanceIndex]) * (KNOWN_TARGET_SPEEDS[distanceIndex + 1] - KNOWN_TARGET_SPEEDS[distanceIndex]) + KNOWN_TARGET_SPEEDS[distanceIndex];
            RobotContainer.FLYWHEEL.setRequested(calculatedFlywheel, "spinByDistance");
            Log.w("20311", "from `MainControl::spinByDistance`: Out of bounds request with note " + note);
        }
    }

    private void spinDown(String note) {
        RobotContainer.FLYWHEEL.setRequested(0, note);
    }

    private void cancelLaunch() {
        RobotContainer.LOADER.cancelLaunch();
    }

    private boolean flywheelStable() {
        return RobotContainer.FLYWHEEL.isStable();
    }

    boolean rightTriggerWasPressed = false;

    boolean rightTriggerWasPulled() {
        if (gamepad1.right_trigger > 0.1) {
            if (!rightTriggerWasPressed) {
                rightTriggerWasPressed = true;
                return true;
            }
        } else rightTriggerWasPressed = false;
        return false;
    }
    private void updateLoader() {
        if (gamepad1.right_bumper) intake();
        else cancelIntake();
        if (
                (gamepad1.right_trigger > 0.1 && (
                    Math.abs(gamepad1.left_stick_x) > 0.1 ||
                    Math.abs(gamepad1.left_stick_y) > 0.1 ||
                    Math.abs(gamepad1.right_stick_x) > 0.1
                )) ||
                (gamepad1.left_bumper && (
                    Math.abs(gamepad1.left_stick_x) > 0.1 ||
                    Math.abs(gamepad1.left_stick_y) > 0.1 ||
                    Math.abs(gamepad1.right_stick_x) > 0.1))) {
            spinByDistance("Right trigger pressed");
        }

        if (gamepad1.leftBumperWasPressed() | rightTriggerWasPulled()) {
            spinByDistance("Right trigger pressed");
            cancelIntake();
            launch();
        }

        if (!gamepad1.left_bumper && gamepad1.right_trigger < 0.1) {
            spinDown("Left bumper released");
            cancelLaunch();
        }
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
