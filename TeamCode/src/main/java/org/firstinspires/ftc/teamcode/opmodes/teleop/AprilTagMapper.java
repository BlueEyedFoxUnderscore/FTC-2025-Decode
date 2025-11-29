package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileInputStream;
import java.text.DecimalFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.TreeMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.json.JSONArray;
import org.json.JSONObject;


@TeleOp//(name = "***FTC OpMode 2", group = "TeleOp")
public class AprilTagMapper extends OpMode {
    private Limelight3A camera; //any camera here
    TreeMap<Integer, LLFieldMap.Fiducial> currentFudicials;



    @Override
    public final void init() {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(0);
        camera.start();
        File dir = new File("/storage/emulated/0/FIRST/settings");
        if (dir.exists() && dir.isDirectory()) {
            osFiles = dir.listFiles();
            if (osFiles != null) {
                for (File f : osFiles) {
                    if (f.isFile() && f.getName().toLowerCase().endsWith(".fmap")) {
                        fMaps.add(f.getName());   // or f.getName()
                    }
                }
            }
        }
        currentFudicials=readFMap(fMaps.get(0));
        loadFiducial(currentFudicials.firstEntry().getValue());
    }

    private void loadFiducial(LLFieldMap.Fiducial fiducial)
    {
        if (fiducial == null) {
            file_pose_x = 0;
            file_pose_y = 0;
            file_pose_z = 0;
            file_pose_theta = 0;
        }
        else {
            tag_id = fiducial.getId();
            file_pose_x = fiducial.getTransform().get(3);
            file_pose_y = fiducial.getTransform().get(7);
            file_pose_z = fiducial.getTransform().get(11);
            double r00 = fiducial.getTransform().get(0);
            double r10 = fiducial.getTransform().get(4);

            file_pose_theta = Math.toDegrees(Math.atan2(r10, r00));
        }
    }

    private final TreeMap<Integer, LLFieldMap.Fiducial> readFMap(String filename)
    {
        TreeMap<Integer, LLFieldMap.Fiducial> fiducials = new TreeMap<>();
        JSONObject json = null;
        String path = "/storage/emulated/0/FIRST/settings/" + filename;

        // ---- Load JSON from file ----
        try (FileInputStream fis = new FileInputStream(path);
             Scanner s = new Scanner(fis, "UTF-8").useDelimiter("\\A"))
        {
            String jsonTxt = s.hasNext() ? s.next() : "";
            json = new JSONObject(jsonTxt);

        } catch (Exception ignored) { }

        if (json != null)
        {
            try {
                // ----- Parse fiducials -----

                JSONArray arr = json.optJSONArray("fiducials");
                if (arr != null)
                {
                    for (int i = 0; i < arr.length(); i++)
                    {
                        JSONObject fObj = arr.optJSONObject(i);
                        if (fObj == null) continue;

                        // ----- EXACTLY Limelight’s Fiducial(JSONObject json) logic -----
                        int id = fObj.optInt("id", -1);
                        double size = fObj.optDouble("size", 165.1);
                        String family = fObj.optString("family", "apriltag3_36h11_classic");

                        ArrayList<Double> transform = new ArrayList<>();
                        JSONArray tArr = fObj.optJSONArray("transform");
                        if (tArr != null) {
                            for (int j = 0; j < tArr.length(); j++) {
                                transform.add(tArr.optDouble(j, 0.0));
                            }
                        }

                        boolean unique = fObj.optBoolean("unique", true);

                        fiducials.put(
                                id,
                                new LLFieldMap.Fiducial(id, size, family, transform, unique)
                        );
                    }
                }

            } catch (Exception ignored) { }
        }

        return fiducials;
    }

    @Override
    public final void init_loop() {
        telemetry.addLine(
        "This utility updates Limelights field maps. Tag "
        +"poses are calculated by composing a chosen robot field "
        +"with an AprilTag measurement taken with the robot placed "
        +"at the chosen pose. Choose poses so the tag is you are "
        +"updating is close to and visible to the Limelight. For "
        +"the purposes of this utility, the front of the robot is "
        +"DEFINED TO BE the side of the robot that the camera "
        +"looks out of.");
        telemetry.addLine("");
        telemetry.addLine("Connecting via ADB then load fmap(s) using");
        telemetry.addLine("");
        telemetry.addLine("adb push mapname.fmap /storage/emulated/0/FIRST/settings/mapname.fmap");
        telemetry.addLine("");
        telemetry.addLine(
        "WARNING: The Limelight's offset in the web interface "
        +"must be set to be relative to the x/y center of the robot. Any "
        +"deviation will show up as error in the AprilTag's calculated "
        +"position.");
    }

    private final ElapsedTime commitTagElapsed = new ElapsedTime();
    private final ElapsedTime removeTagElapsed = new ElapsedTime();
    File[] osFiles;
    List<String> fMaps = new ArrayList<>();
    private int fmapIndex =0;
    @Override
    public final void start() {
        commitTagElapsed.reset();
        removeTagElapsed.reset();
    }

    private final int MAX_MODE_ID = 5;
    private int tag_id = 0;
    private enum MODE {
        FMAP,
        TAG_ID,
        ROBOT_LENGTH,
        ROBOT_WIDTH,
        TILE_X,
        TILE_Y,
        ROBOT_CORNER,
        ROBOT_HEADING
    }
    private MODE mode = MODE.values()[0];
    private int heading_id = 0;
    private int facing_id = 0;
    private double rear_to_front = 17.2;
    private double left_to_right = 16;
    private int xtilecenter = 0;
    private int ytilecenter = 0;

    private double known_pose_x=0;
    private double known_pose_y=0;
    private double known_pose_theta=0;

    private double file_pose_x=0;
    private double file_pose_y=0;
    private double file_pose_z=0;
    private double file_pose_theta=0;

    DecimalFormat df_inches = new DecimalFormat("00.00");
    DecimalFormat df_meters = new DecimalFormat("0.000");
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
        if((!gamepad1.b && !gamepad2.b)||(gamepad1.a||gamepad2.a)) removeTagElapsed.reset();
        int removeTagSecondsLeft = (int) (5.9d - removeTagElapsed.seconds());
        int removeTagFileBlink = (int) ((5.9d - removeTagElapsed.seconds())*4);
        if((!gamepad1.a && !gamepad2.a)||(gamepad1.b||gamepad2.b)) commitTagElapsed.reset();
        int commitTagSecondsLeft = (int) (5.9d - commitTagElapsed.seconds());
        int commitTagBlink = (int) ((5.9d - commitTagElapsed.seconds())*4);
        if (removeTagSecondsLeft>0)
            telemetry.addLine(removeTagFileBlink%2==1?"                 HOLD A for "+removeTagSecondsLeft+" sec. to update tag":"");
        else {
            telemetry.addLine("The tag has been saved to the internal file and the file has been sent to the Limelight. "
                    + "You can download the modified file using:");
            telemetry.addLine("");
            telemetry.addLine("adb pull /storage/emulated/0/FIRST/settings/"+fMaps.get(fmapIndex));
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
        }
        if (commitTagSecondsLeft>0)
            telemetry.addLine(commitTagBlink%2==1?"                 HOLD B for "+commitTagSecondsLeft+" sec. to remove tag":"");
        else{
            telemetry.addLine("The tag has been REMOVED to the internal file and the file has been sent to the Limelight. "
                    +"You can download the modified file using:");
            telemetry.addLine("");
            telemetry.addLine("adb pull /storage/emulated/0/FIRST/settings/"+fMaps.get(fmapIndex));
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
        }

        telemetry.addLine("       L/R Bumpers: Select ID for NEW AprilTag");
        telemetry.addLine("      U/D Dpad:  menu     R/L Dpad: change value");
        telemetry.addLine("");

        telemetry.addData((mode == MODE.FMAP?"[X]":"[   ]")+" Field Map", fMaps.get(fmapIndex));
        telemetry.addData((mode == MODE.TAG_ID?"[X]":"[   ]")+" AprilTag ID", tag_id);
        telemetry.addData((mode == MODE.ROBOT_LENGTH?"[X]":"[   ]")+" Robot length rear-to-front (inches)", rear_to_front);
        telemetry.addData((mode == MODE.ROBOT_WIDTH ?"[X]":"[   ]")+" Robot width left-to-right (inches)", left_to_right);
        telemetry.addData((mode == MODE.TILE_X      ?"[X]":"[   ]")+" # tiles robot corner is +x from field center", xtilecenter);
        telemetry.addData((mode == MODE.TILE_Y      ?"[X]":"[   ]")+" # tiles robot corner is +y from field center", ytilecenter);
        switch (heading_id) {
            case 0:
                telemetry.addLine((mode == MODE.ROBOT_CORNER?"[X]":"[   ]")+" FRONT RIGHT corner of robot on intersection.");
                break;
            case 1:
                telemetry.addLine((mode == MODE.ROBOT_CORNER?"[X]":"[   ]")+" FRONT LEFT corner of robot on intersection.");
                break;
            case 2:
                telemetry.addLine((mode == MODE.ROBOT_CORNER?"[X]":"[   ]")+" REAR RIGHT corner of robot on intersection.");
                break;
            case 3:
                telemetry.addLine((mode == MODE.ROBOT_CORNER?"[X]":"[   ]")+" REAR LEFT corner of robot on intersection.");
                break;
        }
        switch (facing_id) {
            case 0:
                telemetry.addLine((mode == MODE.ROBOT_HEADING?"[X]":"[   ]")+" Robot placed so forward is +X");
                break;
            case 1:
                telemetry.addLine((mode == MODE.ROBOT_HEADING?"[X]":"[   ]")+" Robot placed so forward is -X");
                break;
            case 2:
                telemetry.addLine((mode == MODE.ROBOT_HEADING?"[X]":"[   ]")+" Robot placed so forward is +Y");
                break;
            case 3:
                telemetry.addLine((mode == MODE.ROBOT_HEADING?"[X]":"[   ]")+" Robot placed so forward is -Y");
                break;
        }
        telemetry.addLine("");
        double xsign = 1;
        double ysign = 1;
        switch(facing_id) {
            case 0: // +x
                switch(heading_id) {
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
                switch(heading_id) {
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
                switch(heading_id) {
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
                switch(heading_id) {
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

        telemetry.addLine("BotSelect: (" + df_inches.format(known_pose_x) +"in, " + df_inches.format(known_pose_y) + "in, 0in)@"+ df_inches.format(known_pose_theta));
        telemetry.addLine("BotViewed: (" + df_inches.format(reported_pose_x) +"in, " + df_inches.format(reported_pose_y) + "in, " + df_inches.format(reported_pose_z) + "in)@"+ df_inches.format(reported_pose_theta));
        telemetry.addLine("TagFMAP: (" + df_meters.format(file_pose_x) +"m, " + df_meters.format(file_pose_y) + "m, "+ df_inches.format(file_pose_z)+"m)@"+ df_inches.format(file_pose_theta)    );
        telemetry.addLine("TagViewed: (" + df_meters.format(calculated_robottotag_x) +"m, " + df_meters.format(calculated_robottotag_y) + "m, "+ df_inches.format(calculated_robottotag_z)+"m)@"+ df_inches.format(calculated_robottotag_theta)    );
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
            mode = mode.ordinal() < MODE.values().length-1 ? MODE.values()[mode.ordinal()+1]:MODE.values()[0];
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
            mode = mode.ordinal() >0 ? MODE.values()[mode.ordinal()-1]:MODE.values()[MODE.values().length-1];

        if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())
            {
                tag_id = tag_id < 586 ? tag_id+1 : 0;
                loadFiducial(currentFudicials.get(tag_id));
            }
        if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed())
            {
                tag_id = tag_id > 0 ? tag_id-1 : 586;
                loadFiducial(currentFudicials.get(tag_id));
            }

        switch (mode) {
            case FMAP:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
                    fmapIndex = fmapIndex < (fMaps.size() - 1) ? fmapIndex + 1 : 0;
                    currentFudicials = readFMap(fMaps.get(fmapIndex));
                    loadFiducial(currentFudicials.firstEntry().getValue());
                }
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
                    fmapIndex=fmapIndex>0?fmapIndex-1:(fMaps.size()-1);
                    currentFudicials = readFMap(fMaps.get(fmapIndex));
                    loadFiducial(currentFudicials.firstEntry().getValue());
                }
                break;
            case TAG_ID:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    if (currentFudicials.higherEntry(tag_id)!=null) loadFiducial(currentFudicials.higherEntry(tag_id).getValue());
                    else loadFiducial(currentFudicials.firstEntry().getValue());
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    if (currentFudicials.lowerEntry(tag_id)!=null) loadFiducial(currentFudicials.lowerEntry(tag_id).getValue());
                    else loadFiducial(currentFudicials.lastEntry().getValue());
                break;
            case ROBOT_LENGTH:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    if(rear_to_front<24) rear_to_front+=.1;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    if(rear_to_front>0) rear_to_front-=.1;
                break;
            case ROBOT_WIDTH:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    if(left_to_right<24) left_to_right+=.1;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    if(left_to_right>0) left_to_right-=.1;
                break;
            case TILE_X:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    xtilecenter=xtilecenter<2?xtilecenter+1:-2;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    xtilecenter=xtilecenter>-2?xtilecenter-1:2;
                break;
            case TILE_Y:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    ytilecenter=ytilecenter<2?ytilecenter+1:-2;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    ytilecenter=ytilecenter>-2?ytilecenter-1:2;
                break;
            case ROBOT_CORNER:
                if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
                    heading_id = heading_id <3? heading_id +1:0;
                if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
                    heading_id = heading_id >0? heading_id -1:3;
                break;
            case ROBOT_HEADING:
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


