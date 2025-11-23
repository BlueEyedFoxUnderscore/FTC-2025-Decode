package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GateSubsystem {
    Servo gate;
    private static final double OPEN = 51./300., CLOSED = 150./300.;
    ElapsedTime elapsedTime = new ElapsedTime();

    public GateSubsystem(Servo gate) {
        this.gate = gate;
    }

    public boolean isStable() {
        return elapsedTime.seconds() > 0.20;
    }

    public void open() {
        gate.setPosition(OPEN);
        elapsedTime.reset();
    }

    public void close() {
        gate.setPosition(CLOSED);
        elapsedTime.reset();
    }
}
