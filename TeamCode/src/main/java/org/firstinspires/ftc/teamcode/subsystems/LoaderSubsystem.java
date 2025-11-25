package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Gate;

public class LoaderSubsystem {
    private final Telemetry telemetry;
    private int previousPositionTransfer;
    private final Gate counter;

    /**
     * Starts the launch sequence.
     * <br/>
     * <font color="#CF8E6D">WARNING</font>: Once started, this cannot be stopped.
     */
    public void launch() {
        request = LoaderState.LAUNCH;
    }

    public void cancelLaunch() {
        if(request == LoaderState.LAUNCH) request = LoaderState.READY;
    }

    public void cancelIntake() {
        if(request == LoaderState.INTAKE) request = LoaderState.READY;
    }

    private enum LoaderState {
        /**
         * Initialize the subsystem. Should only be used once.
         */
        INITIALIZE,
        /**
         *
         */
        READY,
        /**
         *
         */
        PRE_INTAKE,
        /**
         *
         */
        INTAKE,
        /**
         *
         */
        WAIT_FOR_STOP_THEN_REGRESS_INTAKE,
        /**
         *
         */
        WAIT_THEN_REGRESS_TRANSFER,
        /**
         *
         */
        LAUNCH,
        /**
         *
         */
        INVALID,
        /**
         *
         */
        LAUNCH_WAIT,
        /**
         *
         */
        ADVANCE,
        /**
         *
         */
        WAIT_REGRESS_THEN_OPEN_GATE,
        /**
         *
         */
        GATE_WAIT,
        /**
         *
         */
        REGRESS_GATE_WAIT_STABLE, LAUNCH_NEXT, LAUNCH_WAIT_LONG, PRE_LAUNCH,
    }
    private final DcMotorEx transfer;
    private final DcMotorEx intake;
    /**
     * Gate subsystem for this shooter
     */
    private final GateSubsystem gate;

    /**
     * Object for measuring elapsed time during regression sequence.
     */
    private final ElapsedTime elapsedTime = new ElapsedTime();

    private LoaderState state = LoaderState.INITIALIZE;
    private int shots = 0;

    /**
     * Constructs a new Flywheel object using all the different required motors.
     * @param transfer The transfer motor.
     * @param gate The gate servo.
     */
    public LoaderSubsystem(DcMotorEx transfer, GateSubsystem gate, DcMotorEx intake, Telemetry telemetry, FlywheelSubsystem flywheel) {
        this.transfer = transfer;
        this.gate = gate;
        this.intake = intake;
        this.telemetry = telemetry;
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        setGateOpen(true);
        this.counter = new Gate(flywheel::isStable);
        counter.debugTelemetry = telemetry;
    }

    /**
     * Does nothing. Returns nothing. For use after chained methods to indicate that the chain is done.
     */
    void dispose() {}

    /**
     * Sets the (binary) position of the gate.
     * This overload takes a boolean to determine the position of the gate.
     * @param open Whether the gate is up or down.
     */
    public void setGateOpen(boolean open) {
        if (open) gate.open();
        else gate.close();
    }

    LoaderState request = LoaderState.INVALID;

    public void intake() {
        request = LoaderState.INTAKE;
    }
    /**
     * Updates all values of the gate and runs the state machine forward.
     */
    public void update() {
        switch(state) {
            case INITIALIZE:
                gate.open();
                state = LoaderState.READY;
                break;
            case READY:
                if (request == LoaderState.INTAKE) {
                    gate.close();
                    state = LoaderState.PRE_INTAKE;
                }
                if (request == LoaderState.LAUNCH) {
                    state = LoaderState.PRE_LAUNCH;
                    counter.reset();
                }
                break;
            case PRE_INTAKE:
                telemetry.addData("is stable", gate.isStable());
                if (gate.isStable()) {
                    state = LoaderState.INTAKE;
                    resetShots();
                    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setPower(0.6);
                    transfer.setPower(0.6);
                }
                break;
            case INTAKE:
                if (request != LoaderState.INTAKE) {
                    intake.setPower(0);
                    transfer.setPower(0);
                    elapsedTime.reset();
                    state = LoaderState.WAIT_FOR_STOP_THEN_REGRESS_INTAKE;
                }
                break;
            case WAIT_FOR_STOP_THEN_REGRESS_INTAKE:
                if (elapsedTime.seconds() > .05) {
                    intake.setTargetPosition(intake.getCurrentPosition() - (int)(145.0 * .25)); //0.25
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setPower(1);
                    elapsedTime.reset();
                    state = LoaderState.WAIT_THEN_REGRESS_TRANSFER;
                }
                break;
            case WAIT_THEN_REGRESS_TRANSFER:
                if (elapsedTime.seconds() > .1) {
                    transfer.setTargetPosition(transfer.getCurrentPosition() - (int)(537 * .25));
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.setPower(1);
                    state = LoaderState.WAIT_REGRESS_THEN_OPEN_GATE;
                }
                break;
            case WAIT_REGRESS_THEN_OPEN_GATE:
                if (elapsedTime.seconds() > .2) {
                    gate.open();
                    state = LoaderState.REGRESS_GATE_WAIT_STABLE;
                }
                break;
            case REGRESS_GATE_WAIT_STABLE:
                if (gate.isStable()) {
                    state = LoaderState.READY;
                }
                break;

            case PRE_LAUNCH:
                counter.update();
                if (counter.trueForAtLeast(0.2)) state = LoaderState.LAUNCH;
                if (request == LoaderState.READY) state = LoaderState.READY;
                break;
            case LAUNCH:
                //intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setTargetPosition(transfer.getCurrentPosition() + 537 / 1);
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(1);
                elapsedTime.reset();
                state = LoaderState.LAUNCH_WAIT;
                counter.reset();
                break;
            case LAUNCH_WAIT:
                counter.update();
                if (elapsedTime.seconds() > 0.10 && counter.trueForAtLeast(0.10)) { // 0.20
                    if (request == LoaderState.READY) {
                        gate.close();
                        state = LoaderState.GATE_WAIT;
                    }
                    else {
                        state = LoaderState.LAUNCH_NEXT;
                        shots += 1;
                    }
                }
                break;
            case LAUNCH_NEXT:
                transfer.setTargetPosition(transfer.getCurrentPosition() + of(537 * 1.5));
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setTargetPosition(intake.getCurrentPosition() + of(145 * 2));
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(1);
                intake.setPower(1);
                elapsedTime.reset();
                counter.reset();
                state = LoaderState.LAUNCH_WAIT_LONG;
                break;
            case LAUNCH_WAIT_LONG:
                counter.update();
                if (elapsedTime.seconds() > 0.30 && counter.trueForAtLeast(0.20)) { // 0.40
                    if (request == LoaderState.READY) {
                        gate.close();
                        state = LoaderState.GATE_WAIT;
                    }
                    else {
                        state = LoaderState.LAUNCH_NEXT;
                        shots += 1;
                    }
                }
                break;
            case GATE_WAIT:
                if (gate.isStable()) {
                    transfer.setPower(0);
                    intake.setPower(0);
                    state = LoaderState.ADVANCE;
                    elapsedTime.reset();
                }
                break;
            case ADVANCE:
                transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transfer.setPower(0.7);
                intake.setPower(0.7);
                if (elapsedTime.seconds() > 0.45) {
                    intake.setPower(0);
                    transfer.setPower(0);
                    elapsedTime.reset();
                    state = LoaderState.WAIT_FOR_STOP_THEN_REGRESS_INTAKE;
                }
                break;
        }
        telemetry.addLine(state.name());
    }

    public int getShots() {
        return shots;
    }

    private int loaded;

    public void setLoaded(int loaded) {
        this.loaded = loaded;
    }

    public boolean doneFiring() {
        return shots > loaded;
    }

    public void resetShots() {
        shots = 0;
    }

    private int of(double x) {
        return (int) x;
    }

}
