package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotContainer {
    private static Servo gate;
    private static DcMotorEx intake, transfer, shooter1, shooter2;

    public static FlywheelSubsystem FLYWHEEL;
    public static GateSubsystem GATE;
    public static LoaderSubsystem LOADER;
    private static boolean initialized = false;
    public static void init(HardwareMap hardwareMap, Telemetry telemetry) {
        gate = hardwareMap.get(Servo.class, "gate");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        FLYWHEEL = new FlywheelSubsystem(shooter1, shooter2, telemetry);
        GATE = new GateSubsystem(gate);
        LOADER = new LoaderSubsystem(transfer, GATE, intake, telemetry, FLYWHEEL);
        LOADER.setFlywheel(FLYWHEEL);
        FLYWHEEL.setReady(LOADER::mayChangeSpeeds);
        initialized = true;
    }

    public static void update() {
        FLYWHEEL.update();
        LOADER.update();
    }
}
