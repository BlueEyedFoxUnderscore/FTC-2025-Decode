package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.DecimalFormat;
import java.util.ArrayDeque;
import java.util.LinkedList;
import java.util.List;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp//(name = "***FTC OpMode 2", group = "TeleOp")
public class AprilTagMapper extends OpMode {
    private Limelight3A camera; //any camera here

    @Override
    public final void init() {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(0);
        camera.start();
    }


    @Override
    public final void init_loop() {
        telemetry.addLine("LIMELIGHT FIELD MAP UPDATER");
        telemetry.addLine("");
        telemetry.addLine(
        "This utility updates your Limelights's field map "
        +"by adding, removing, or modifying tags individually. "
        +"AprilTag locations are calculated by combining a selected "
        +"robot field pose with an AprilTag measurement taken in the "
        +"robot's coordinate system. The robot is placed so any "
        +"corner is aligned to any tile intersection and so the robot "
        +"faces and is parallel to any wall.  The tag must be close "
        +"to and visible to the Limelight.  For the purposes of this "
        +"utility, the front of the robot is the side the camera looks "
        +"out of.  Assumes pipeline 0.");
        telemetry.addLine("");
        telemetry.addLine(
        "WARNING: The position of the Limelight relative "
        +"to the robot must be correctly set in the LimeLight's web "
        +"interface.  Any error there will propagate into the "
        +"AprilTag's calculated position.");
    }

    private final ElapsedTime writeElapsed = new ElapsedTime();
    private final ElapsedTime removeElapsed = new ElapsedTime();

    @Override
    public final void start() {
        writeElapsed.reset();
        removeElapsed.reset();
    }

    private final int MAX_MODE_ID = 5;
    private int tag_id = 0;
    private int mod_id = 0;
    private int wheel_id = 0;
    private int facing_id = 0;
    private double rear_to_front = 17.2;
    private double left_to_right = 16;
    private int xtilecenter = 0;
    private int ytilecenter = 0;

    private double known_pose_x=0;
    private double known_pose_y=0;
    private double known_pose_theta=0;

    private double tag_pose_x=0;
    private double tag_pose_y=0;
    private double tag_pose_theta=0;

    DecimalFormat df = new DecimalFormat("00.000");
    private static final double INCHES_PER_METER = 39.3701;
    double reported_pose_x =0d, reported_pose_y =0d, reported_pose_z =0d, reported_pose_theta =0d;
    double reported_robottotag_x =0d, reported_robottotag_y =0d, reported_robottotag_z =0d, reported_robottotag_theta =0d;
    ArrayDeque<Double> reported_pose_x_list = new ArrayDeque<Double>();
    ArrayDeque<Double> reported_pose_y_list = new ArrayDeque<Double>();
    ArrayDeque<Double> reported_pose_z_list = new ArrayDeque<Double>();
    ArrayDeque<Double> reported_pose_theta_list = new ArrayDeque<Double>();
    ArrayDeque<Double> reported_robottotag_x_list = new ArrayDeque<Double>();
    ArrayDeque<Double> reported_robottotag_y_list = new ArrayDeque<Double>();
    ArrayDeque<Double> reported_robottotag_z_list = new ArrayDeque<Double>();
    ArrayDeque<Double> reported_robottotag_theta_list = new ArrayDeque<Double>();
    private double lastSample = 0;

    @Override
    public final void loop() {
        LLResult result = camera.getLatestResult();
        if (result.isValid() && result.getTimestamp() != lastSample) {
            lastSample = result.getTimestamp();
            Pose3D botpose = result.getBotpose();
            Pose3D tagposeRobot = null;
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                // For now, just take the first detected tag; you can pick "best" later if you want
                LLResultTypes.FiducialResult firstTag = fiducials.get(0);
                tagposeRobot = firstTag.getTargetPoseRobotSpace();
            }

            Position position = botpose.getPosition();
            YawPitchRollAngles orientation = botpose.getOrientation();
            reported_pose_x_list.add(position.x * INCHES_PER_METER);
            reported_pose_y_list.add(position.y * INCHES_PER_METER);
            reported_pose_z_list.add(position.z * INCHES_PER_METER);
            reported_pose_theta_list.add(orientation.getYaw(AngleUnit.DEGREES));
            if (reported_pose_x_list.size() > 50) {
                reported_pose_x_list.removeFirst();
                reported_pose_y_list.removeFirst();
                reported_pose_z_list.removeFirst();
                reported_pose_theta_list.removeFirst();
            }
            reported_pose_x = 0;
            for (double v : reported_pose_x_list) {
                reported_pose_x += v;
            }
            reported_pose_y = 0;
            for (double v : reported_pose_y_list) {
                reported_pose_y += v;
            }
            reported_pose_z = 0;
            for (double v : reported_pose_z_list) {
                reported_pose_z += v;
            }
            reported_pose_theta = 0;
            for (double v : reported_pose_theta_list) {
                reported_pose_theta += v;
            }
            reported_pose_x /= reported_pose_x_list.size();
            reported_pose_y /= reported_pose_y_list.size();
            reported_pose_z /= reported_pose_z_list.size();
            reported_pose_theta /= reported_pose_theta_list.size();

            if (tagposeRobot != null) {
                position = tagposeRobot.getPosition();
                orientation = tagposeRobot.getOrientation();
                // ftc x (forward) is apriltag sensor z
                reported_robottotag_x_list.add(position.z * INCHES_PER_METER);
                // ftc y (left) is apriltag sensor -x
                reported_robottotag_y_list.add(-position.x * INCHES_PER_METER);
                // ftc y (left) is apriltag sensor -x
                reported_robottotag_z_list.add(position.y * INCHES_PER_METER);
                // ftc ccw is negative rotation about y, which is negative pitch
                reported_robottotag_theta_list.add(-orientation.getPitch(AngleUnit.DEGREES));
                if (reported_robottotag_x_list.size() > 50) {
                    reported_robottotag_x_list.removeFirst();
                    reported_robottotag_y_list.removeFirst();
                    reported_robottotag_z_list.removeFirst();
                    reported_robottotag_theta_list.removeFirst();
                }
                reported_robottotag_x = 0;
                for (double v : reported_robottotag_x_list) {
                    reported_robottotag_x += v;
                }
                reported_robottotag_y = 0;
                for (double v : reported_robottotag_y_list) {
                    reported_robottotag_y += v;
                }
                reported_robottotag_z = 0;
                for (double v : reported_robottotag_y_list) {
                    reported_robottotag_z += v;
                }
                reported_robottotag_theta = 0;
                for (double v : reported_robottotag_theta_list) {
                    reported_robottotag_theta += v;
                }
                reported_robottotag_x /= reported_robottotag_x_list.size();
                reported_robottotag_y /= reported_robottotag_y_list.size();
                reported_robottotag_z /= reported_robottotag_z_list.size();
                reported_robottotag_theta /= reported_robottotag_theta_list.size();
            }
        }
        if((!gamepad1.a && !gamepad2.a)||(gamepad1.x||gamepad2.x)) writeElapsed.reset();
        if((!gamepad1.x && !gamepad2.x)||(gamepad1.a||gamepad2.a)) removeElapsed.reset();
        int writeSecondsLeft = (int) (5.9d - writeElapsed.seconds());
        int writeSecondsLeftBlink = (int) ((5.9d - writeElapsed.seconds())*4);
        int removeSecondsLeft = (int) (5.9d - removeElapsed.seconds());
        int removeSecondsLeftBlink = (int) ((5.9d - removeElapsed.seconds())*4);
        telemetry.addData("      CURRENTLY MODIFYING APRILTAG ID# ", tag_id);
        telemetry.addLine("       L/R Bumper: change AprilTag ID to modify");
        if (writeSecondsLeft>0)
            telemetry.addLine(writeSecondsLeftBlink%2==1?"             HOLD A for "+writeSecondsLeft+" sec. to record this tag":"");
        else
            telemetry.addLine("                               TAG WRITTEN");
        if (removeSecondsLeft>0)
            telemetry.addLine(removeSecondsLeftBlink%2==1?"            HOLD X for "+removeSecondsLeft+" sec. to remove this tag":"");
        else
            telemetry.addLine("                              TAG REMOVED");
        telemetry.addLine("---------------------------------------------------------------------------");
        telemetry.addLine("      U/D Dpad:  menu     R/L Dpad: change value");
        telemetry.addLine("");
        telemetry.addData((mod_id==0?"[X]":"[   ]")+" Robot length rear-to-front (inches)", rear_to_front);
        telemetry.addData((mod_id==1?"[X]":"[   ]")+" Robot width left-to-right (inches)", left_to_right);
        telemetry.addData((mod_id==2?"[X]":"[   ]")+" # tiles robot corner is +x from field center", xtilecenter);
        telemetry.addData((mod_id==3?"[X]":"[   ]")+" # tiles robot corner is +y from field center", ytilecenter);
        switch (wheel_id) {
            case 0:
                telemetry.addLine((mod_id==4?"[X]":"[   ]")+" FRONT RIGHT corner of robot on intersection.");
                break;
            case 1:
                telemetry.addLine((mod_id==4?"[X]":"[   ]")+" FRONT LEFT corner of robot on intersection.");
                break;
            case 2:
                telemetry.addLine((mod_id==4?"[X]":"[   ]")+" REAR RIGHT corner of robot on intersection.");
                break;
            case 3:
                telemetry.addLine((mod_id==4?"[X]":"[   ]")+" REAR LEFT corner of robot on intersection.");
                break;
        }
        switch (facing_id) {
            case 0:
                telemetry.addLine((mod_id==5?"[X]":"[   ]")+" Robot placed so forward is +X");
                break;
            case 1:
                telemetry.addLine((mod_id==5?"[X]":"[   ]")+" Robot placed so forward is -X");
                break;
            case 2:
                telemetry.addLine((mod_id==5?"[X]":"[   ]")+" Robot placed so forward is +Y");
                break;
            case 3:
                telemetry.addLine((mod_id==5?"[X]":"[   ]")+" Robot placed so forward is -Y");
                break;
        }
        telemetry.addLine("");
        double xsign = 1;
        double ysign = 1;
        switch(facing_id) {
            case 0: // +x
                switch(wheel_id) {
                    case 0: // Front Right
                        xsign = -1; // front
                        ysign =  1; // right
                        break;
                    case 1: // Front Left
                        xsign = -1; // front
                        ysign = -1; // left
                        break;
                    case 2: // Rear Right
                        xsign =  1; // rear
                        ysign =  1; // right
                        break;
                    case 3: // Rear Left
                        xsign =  1; // rear
                        ysign = -1; // left
                        break;
                }
                break;
            case 1: // -x
                switch(wheel_id) {
                    case 0: // Front Right
                        xsign =  1; // front
                        ysign = -1; // right
                        break;
                    case 1: // Front Left
                        xsign =  1; // front
                        ysign =  1; // left
                        break;
                    case 2: // Rear Right
                        xsign = -1; // rear
                        ysign = -1; // right
                        break;
                    case 3: // Rear Left
                        xsign = -1; // rear
                        ysign = +1; // left
                        break;
                }
                break;
            case 2: // +y
                switch(wheel_id) {
                    case 0: // Front Right
                        ysign = -1; // front
                        xsign = -1; // right
                        break;
                    case 1: // Front Left
                        ysign = -1; // front
                        xsign =  1; // left
                        break;
                    case 2: // Rear Right
                        ysign =  1; // rear
                        xsign = -1; // right
                        break;
                    case 3: // Rear Left
                        ysign =  1; // rear
                        xsign =  1; // left
                        break;
                }
                break;
            case 3: // -y
                switch(wheel_id) {
                    case 0: // Front Right
                        ysign =  1; // front
                        xsign =  1; // right
                        break;
                    case 1: // Front Left
                        ysign =  1; // front
                        xsign = -1; // left
                        break;
                    case 2: // Rear Right
                        ysign = -1; // rear
                        xsign =  1; // right
                        break;
                    case 3: // Rear Left
                        ysign = -1; // rear
                        xsign = -1; // left
                        break;
                }
                break;
        }
        known_pose_x = 24d*xtilecenter+xsign*(facing_id<2?rear_to_front/2d:left_to_right/2d);
        known_pose_y = 24d*ytilecenter+ysign*(facing_id<2?left_to_right/2d:rear_to_front/2d);
        switch (facing_id) {
            case 0:
                known_pose_theta=0;
                break;
            case 1:
                known_pose_theta=180;
                break;
            case 2:
                known_pose_theta=90;
                break;
            case 3:
                known_pose_theta=270;
                break;
        }

        double calculated_robottotag_x = known_pose_x + reported_robottotag_x * Math.cos(Math.toRadians(known_pose_theta)) - reported_robottotag_y * Math.sin(Math.toRadians(known_pose_theta));
        double calculated_robottotag_y = known_pose_y + reported_robottotag_x * Math.sin(Math.toRadians(known_pose_theta)) + reported_robottotag_y * Math.cos(Math.toRadians(known_pose_theta));

        // Tag heading in field frame
        double calculated_robottotag_theta = known_pose_theta + reported_robottotag_theta + 180;
        // normalize to [0,360) just to keep it sane
        calculated_robottotag_theta = (((calculated_robottotag_theta % 360) + 360) % 360);
        calculated_robottotag_theta = calculated_robottotag_theta>180d?calculated_robottotag_theta-360d:calculated_robottotag_theta;
        calculated_robottotag_x /= INCHES_PER_METER;
        calculated_robottotag_y /= INCHES_PER_METER;
        double calculated_robottotag_z = reported_robottotag_z/INCHES_PER_METER;

        telemetry.addLine("Selected: (" + df.format(known_pose_x) +"in, " + df.format(known_pose_y) + "in, 0in) @ "+df.format(known_pose_theta));
        telemetry.addLine("Seen: (" + df.format(reported_pose_x) +"in, " + df.format(reported_pose_y) + "in, " + df.format(reported_pose_z) + "in) @ "+df.format(reported_pose_theta));
        telemetry.addLine("The observed tag field pose for fmap is");
        telemetry.addLine("(" + df.format(calculated_robottotag_x) +"m, " + df.format(calculated_robottotag_y) + "m, "+df.format(calculated_robottotag_z)+"m) @ "+df.format(calculated_robottotag_theta)    );
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
            mod_id=mod_id<MAX_MODE_ID?mod_id+1:0;
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
            mod_id=mod_id>0?mod_id-1:MAX_MODE_ID;

        if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())
            tag_id=tag_id<586?tag_id+1:0;
        if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed())
            tag_id=tag_id>0?tag_id-1:586;

        switch (mod_id) {
            case 0:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    if(rear_to_front<24) rear_to_front+=.1;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    if(rear_to_front>0) rear_to_front-=.1;
                break;
            case 1:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    if(left_to_right<24) left_to_right+=.1;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    if(left_to_right>0) left_to_right-=.1;
                break;
            case 2:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    xtilecenter=xtilecenter<2?xtilecenter+1:-2;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    xtilecenter=xtilecenter>-2?xtilecenter-1:2;
                break;
            case 3:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    ytilecenter=ytilecenter<2?ytilecenter+1:-2;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    ytilecenter=ytilecenter>-2?ytilecenter-1:2;
                break;
            case 4:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    wheel_id=wheel_id<3?wheel_id+1:0;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    wheel_id=wheel_id>0?wheel_id-1:3;
                break;
            case 5:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    facing_id=facing_id<3?facing_id+1:0;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    facing_id=facing_id>0?facing_id-1:3;
                break;
        }
    }

    @Override
    public final void stop() {
        //camera.uploadFieldmap(new LLFieldMap(), 1);
    }
}


