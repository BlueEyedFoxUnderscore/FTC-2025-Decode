package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Gate;

public class LoaderSubsystem {
    private final Telemetry telemetry;
    private int previousPositionTransfer;
    private final Gate isFlywheelStableGate;

    /**
     * Starts the launch sequence.
     * <br/>
     * <font color="#CF8E6D">WARNING</font>: Once started, this cannot be stopped.
     */
    public void launch() {
        request = LoaderState.LAUNCH;
    }

    public void cancelLaunch() {
        if (request == LoaderState.LAUNCH) request = LoaderState.READY;
    }

    public void cancelIntake() {
        if (request == LoaderState.INTAKE) request = LoaderState.READY;
    }

    public boolean mayChangeSpeeds() {
        return (
                this.state != LoaderState.LAUNCH_WAIT
            && this.state != LoaderState.LAUNCH_WAIT_LONG
            && this.state != LoaderState.LAUNCH
            && this.state != LoaderState.LAUNCH_NEXT
            && this.state != LoaderState.INTAKE
            && this.state != LoaderState.WAIT_FOR_STOP_THEN_REGRESS_INTAKE
            && this.state != LoaderState.WAIT_THEN_REGRESS_TRANSFER
            && this.state != LoaderState.WAIT_REGRESS_THEN_OPEN_GATE
            && this.state != LoaderState.REGRESS_GATE_WAIT_STABLE);
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
        REGRESS_GATE_WAIT_STABLE, LAUNCH_NEXT, LAUNCH_WAIT_LONG, PRE_LAUNCH, CYCLE, PRE_LAUNCH_WAIT,
    }
    private final DcMotorEx transfer;
    private final DcMotorEx intake;
    private static final double cycleSpeed = 850;
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
        //transfer.setPositionPIDFCoefficients(kPx);

        Log.i("20311", "Transfer velocity pid: "+transfer.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        Log.i("20311", "Transfer position pid: "+transfer.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());
        Log.i("20311", "Intake velocity pid: "+intake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        Log.i("20311", "Intake position pid: "+intake.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

        transfer.setVelocityPIDFCoefficients(0, 0, 0, 20);
        transfer.setPositionPIDFCoefficients(10);
        intake.setVelocityPIDFCoefficients(0, 0, 0, 30);
        intake.setPositionPIDFCoefficients(10);

        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0, MotorControlAlgorithm.PIDF));
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        setGateOpen(true);
        this.isFlywheelStableGate = new Gate(flywheel::isStable);
        isFlywheelStableGate.debugTelemetry = telemetry;
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

    LoaderState request = LoaderState.READY;

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
            case READY:
                if (request == LoaderState.INTAKE) {
                    gate.close();
                    state = LoaderState.PRE_INTAKE;
                }
                if (request == LoaderState.LAUNCH) {
                    state = LoaderState.PRE_LAUNCH;
                    isFlywheelStableGate.reset();
                }
                break;
            case PRE_INTAKE:
                //telemetry.addData("is stable", gate.isStable());
                if (gate.isStable()) {
                    state = LoaderState.INTAKE;
                    resetShots();
                    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setPower(0.6);
                    transfer.setPower(0.3);
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
                    intake.setTargetPosition(intake.getCurrentPosition() - (int)(145.0 * .15)); //0.25
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setPower(1);
                    elapsedTime.reset();
                    state = LoaderState.WAIT_THEN_REGRESS_TRANSFER;
                }
                break;
            case WAIT_THEN_REGRESS_TRANSFER:
                if (elapsedTime.seconds() > .1) {
                    transfer.setTargetPosition(transfer.getCurrentPosition() - (int)(537 * .20));
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.setPower(1);
                    state = LoaderState.WAIT_REGRESS_THEN_OPEN_GATE;
                }
                break;
            case WAIT_REGRESS_THEN_OPEN_GATE:
                if (elapsedTime.seconds() > .4) {
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
                state=LoaderState.PRE_LAUNCH_WAIT;
                // Only so we can spin up the flywheel is this state needed
            case PRE_LAUNCH_WAIT:
                isFlywheelStableGate.update();
                if (isFlywheelStableGate.trueForAtLeast(0.25)) state = LoaderState.LAUNCH;
                if (request != LoaderState.LAUNCH) state = LoaderState.READY;
                break;
            case LAUNCH:
                transfer.setTargetPosition(transfer.getCurrentPosition() + (int)(537 * 0.8));
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setTargetPosition(intake.getCurrentPosition() + (int)(145 * 0.8));
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(1);
                intake.setPower(1);
                elapsedTime.reset();
                state = LoaderState.LAUNCH_WAIT;
                isFlywheelStableGate.reset();
                shots += 1;
                Log.i("20311", "Shots = "+shots);
                break;
            case LAUNCH_WAIT:
                isFlywheelStableGate.update();
                if (elapsedTime.seconds() > 0.7) { // 0.20
                    if (isFlywheelStableGate.trueForAtLeast(0.25)) state = LoaderState.LAUNCH_NEXT;
                    if (request != LoaderState.LAUNCH) {
                        state = LoaderState.READY;
                    }
                }
                break;
            case LAUNCH_NEXT:
                transfer.setTargetPosition(transfer.getCurrentPosition() + (int)(537 * 1.0));
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setTargetPosition(intake.getCurrentPosition() + (int)(145 * 1.0));
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elapsedTime.reset();
                isFlywheelStableGate.reset();
                shots += 1;
                Log.i("20311", "Shots = "+shots);
                state = LoaderState.LAUNCH_WAIT_LONG;
                break;
            case LAUNCH_WAIT_LONG:
                isFlywheelStableGate.update();
                if (elapsedTime.seconds() > 0.70) {
                    if (isFlywheelStableGate.trueForAtLeast(0.2)) { // 0.40
                        state = LoaderState.LAUNCH_NEXT;
                    }
                    if (request != LoaderState.LAUNCH) {
                        state = LoaderState.READY;
                    }
                }
                break;
            default:
                telemetry.addData("Invalid state!", state.name());
        }
        telemetry.addData("CURRENT LOADER STATE: ", state.name());
        telemetry.addData("REQUEST LOADER STATE: ", request.name());
    }

    public int getShots() {
        return shots;
    }

    private int loaded;

    public void setLoaded(int loaded) {
        this.loaded = loaded;
        Log.i("20311", "Loaded "+loaded+" balls.");
    }

    public boolean doneFiring() {
        return shots >= loaded;
    }

    public void resetShots() {
        Log.i("20311", "Reset shot count.");
        shots = 0;
    }

    private int of(double x) {
        return (int) x;
    }

}
