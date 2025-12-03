package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.atan2;
import static java.lang.StrictMath.max;

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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.opmodes.auto.PathAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import com.acmerobotics.dashboard.FtcDashboard;

import java.lang.annotation.Annotation;
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
    private boolean canright = true, canleft = true, canup = true, candown = true;
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
        follower.setStartingPose(new Pose());


        follower.startTeleopDrive(true);
        follower.update();
        imuTimeout.reset();
        RobotContainer.init(hardwareMap, telemetry);

        telemetry.addLine("Initialized - waiting for IMU calibration...");
        telemetry.update();
        Drawing.init();

        follower.setStartingPose(new Pose(0, 0, 0));

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
            testp += 5.;
            RobotContainer.FLYWHEEL.testp(testp);
        }
        if (gamepad1.dpadDownWasPressed()) {
            testp -= 5.;
            RobotContainer.FLYWHEEL.testp(testp);
        }

        dash.sendTelemetryPacket(packet);
//        gate.setPosition(gamepad1.right_trigger);
    }

    private LinkedList<Pose> samples = new LinkedList<>();
    private ElapsedTime elapsedTime = new ElapsedTime();

    void setFollowerMaxPower(double maxPower) {
        follower.setMaxPower(maxPower);
    }


    private double lastSample = 0;
    private Limelight3A camera;
    private static final double INCHES_PER_METER = 39.3701;

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




    AprilState aprilState = AprilState.READY;

    enum AprilState {SAMPLE_TAGS, SAMPLE_TAGS_2, SAMPLE_TAGS_3, SAMPLE_TAGS_4, READY;}

    boolean shouldSetHoldPointInitial = true, shouldSetHoldPointSecondary = true;
    ElapsedTime holdTimeout = new ElapsedTime();
    double lastHeading = 0;

    PIDController headingController = new PIDController(1, 0, 0);
    boolean square = false;
    boolean toGo = false;
    private void updateDrive() {
        if (gamepad1.square && !square) {
            square = true;
            toGo = !toGo;
        }

        double v = headingController.calculate(-AngleUnit.normalizeRadians(atan2(144 - 6 - follower.getPose().getY(),144 - 6 - follower.getPose().getX())-follower.getHeading()));
        if (!(Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01 || gamepad1.right_trigger > 0.1)) {
            if (shouldSetHoldPointInitial) {
                holdTimeout.reset();
                lastHeading = follower.getVelocity().getTheta();
                shouldSetHoldPointInitial = false;
                follower.holdPoint(toGo?new Pose(105d+2, 33d+.25):follower.getPose());
            } else if ((holdTimeout.seconds() > 0.2) && shouldSetHoldPointSecondary) {
                shouldSetHoldPointSecondary = false;
                follower.holdPoint(toGo?new Pose(105d+2, 33d+.25):follower.getPose());
                aprilState = AprilState.SAMPLE_TAGS;
            }
        } else {
            aprilState = AprilState.READY;
            if (!shouldSetHoldPointInitial) follower.startTeleopDrive(true);
            follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger > 0.1? v: -gamepad1.right_stick_x/*-gamepad1.right_stick_x*/, false, offset);
            shouldSetHoldPointInitial = shouldSetHoldPointSecondary = true;
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
                if (elapsedTime.seconds() > 0.3) {
                    elapsedTime.reset();
                    aprilState = AprilState.SAMPLE_TAGS_3;
                }
                break;
            case SAMPLE_TAGS_3: /// Get apriltag samples
                addSampleIfAvailable(samples);
                if (samples.size() >= 15) {
                    aprilState = AprilState.SAMPLE_TAGS_4;
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mediumBeep);
                    Pose average = getAverageOfBest(samples, 5);
                    follower.setPose(average);
                    follower.holdPoint(average);
                    aprilState = AprilState.READY;
                }
                else break;
        }
        telemetry.addData("toGo", toGo);
        telemetry.addData("joyX", gamepad1.left_stick_x);
        telemetry.addData("joyY", gamepad1.left_stick_y);
        telemetry.addData("spin", gamepad1.right_stick_x);
        telemetry.addData("shouldHold", shouldSetHoldPointInitial);
        telemetry.addData("velocity", follower.getVelocity().getMagnitude() < 0.1);
        telemetry.addData("joystickIsSwizzle", !(Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01));
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
        RobotContainer.FLYWHEEL.setRequested(flywheelSpeed, note); // 2800 2400
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

    private void updateLoader() {
        if (gamepad1.right_bumper) intake();
        else cancelIntake();
        if (gamepad1.leftBumperWasPressed()) {
            spinUp("Left bumper pressed");
            launch();
        }
        if (gamepad1.leftBumperWasReleased()) {
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
