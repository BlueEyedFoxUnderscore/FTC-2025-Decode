package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class Gate {
    public Telemetry debugTelemetry = null;

    private final ElapsedTime time = new ElapsedTime();
    private final BooleanSupplier gate;
    public Gate(BooleanSupplier gate) {
        this.gate = gate;
    }


    public void update() {
        if (!gate.getAsBoolean()) reset();
    }

    public void reset() {
        time.reset();
    }

    public boolean trueForAtLeast(double required) {
        if(debugTelemetry != null) {
            debugTelemetry.addData("Time", time.seconds());
            debugTelemetry.addData("Till", required);
        }
        return time.seconds() >= required;
    }
}
