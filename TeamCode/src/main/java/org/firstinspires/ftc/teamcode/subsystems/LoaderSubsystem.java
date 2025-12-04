package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Gate;

public class LoaderSubsystem {
    private final Telemetry telemetry;
    private int previousPositionTransfer;
    private final Gate isFlywheelStableGate;

    public Gamepad gamepad1 = null;

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
        if (request == LoaderState.INTAKING) request = LoaderState.READY;
    }

    public boolean mayChangeSpeeds() {
        return (
                this.state != LoaderState.LAUNCH_WAIT
            && this.state != LoaderState.LAUNCH_WAIT_LONG
            && this.state != LoaderState.LAUNCH
            && this.state != LoaderState.LAUNCH2
            && this.state != LoaderState.LAUNCH_NEXT
            && this.state != LoaderState.INTAKING
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
        INTAKE,
        /**
         *
         */
        INTAKING,
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
        REGRESS_GATE_WAIT_STABLE, LAUNCH_NEXT, LAUNCH_WAIT_LONG, PRE_LAUNCH, CYCLE, PRE_LAUNCH_WAIT, LAUNCH2,
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


        intake.setVelocityPIDFCoefficients(0, 0, -5, 25 );
        intake.setPositionPIDFCoefficients(10);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.15);
        // test Intake pid
        // intake.setTargetPosition(intake.getCurrentPosition());
        // intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // intake.setPower(1);

        transfer.setVelocityPIDFCoefficients(0, 0, -5, 15);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setPositionPIDFCoefficients(10);
        transfer.setTargetPosition(transfer.getCurrentPosition());
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(1);
        // test Trnsfer pid
        // transfer.setTargetPosition(transfer.getCurrentPosition() + (int)(537 * 2));


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
        request = LoaderState.INTAKING;
    }
    /**
     * Updates all values of the gate and runs the state machine forward.
     */
    public void update() {
        if (gamepad1 != null) {
            if (gamepad1.dpadUpWasPressed()) {
                intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients());
            }
        }
        switch(state) {
            case INITIALIZE:
                gate.open();
                state = LoaderState.READY;
            case READY:
                if (request == LoaderState.INTAKING) {
                    gate.close();
                    state = LoaderState.INTAKE;
                }
                if (request == LoaderState.LAUNCH) {
                    state = LoaderState.PRE_LAUNCH;
                    isFlywheelStableGate.reset();
                }
                break;
            case INTAKE:
                //telemetry.addData("is stable", gate.isStable());
                if (gate.isStable()) {
                    state = LoaderState.INTAKING;
                    resetShots();
                    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setPower(.7);
                    transfer.setPower(0.15);
                }
                break;
            case INTAKING:
                if (request != LoaderState.INTAKING) {
                    intake.setPower(0);
                    transfer.setPower(0);
                    elapsedTime.reset();
                    state = LoaderState.WAIT_FOR_STOP_THEN_REGRESS_INTAKE;
                }
                break;
            case WAIT_FOR_STOP_THEN_REGRESS_INTAKE:
                if (elapsedTime.seconds() > .05) {
                    intake.setTargetPosition(intake.getCurrentPosition() - (int)(145.0 * (.1))); //0.25
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setPower(1);
                    elapsedTime.reset();
                    state = LoaderState.WAIT_THEN_REGRESS_TRANSFER;
                }
                break;
            case WAIT_THEN_REGRESS_TRANSFER:
                if (elapsedTime.seconds() > .2) {
                    transfer.setTargetPosition(transfer.getCurrentPosition() - (int)(537.0 * (.1)));
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.setPower(1);
                    state = LoaderState.WAIT_REGRESS_THEN_OPEN_GATE;
                }
                break;
            case WAIT_REGRESS_THEN_OPEN_GATE:
                if (elapsedTime.seconds() > .6) {
                    gate.open();
                    state = LoaderState.REGRESS_GATE_WAIT_STABLE;
                    intake.setPower(0.15);
                    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setPower(0.15);
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
                if (isFlywheelStableGate.trueForAtLeast(0.2)) state = LoaderState.LAUNCH;
                if (request != LoaderState.LAUNCH) state = LoaderState.READY;
                break;
            case LAUNCH:
                transfer.setTargetPosition(transfer.getCurrentPosition() + (int)(537 * 2));
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(1);
                elapsedTime.reset();
                state = LoaderState.LAUNCH2;
                isFlywheelStableGate.reset();
                Log.i("20311", "Shots = "+shots);
                break;
            case LAUNCH2:
                if (elapsedTime.seconds() > 0.3) { // 0.20
                    shots += 1;
                    transfer.setTargetPosition(transfer.getCurrentPosition() + (int) (537 * .9));
                    transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setTargetPosition(intake.getCurrentPosition() + (int) (145 * .9));
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.setPower(1);
                    intake.setPower(1);
                    elapsedTime.reset();
                    state = LoaderState.LAUNCH_WAIT;
                    isFlywheelStableGate.reset();
                    Log.i("20311", "Shots = " + shots);
                }
                break;
            case LAUNCH_WAIT:
                isFlywheelStableGate.update();
                if (elapsedTime.seconds() > 0.4) { // 0.20
                    if (isFlywheelStableGate.trueForAtLeast(0.2)) state = LoaderState.LAUNCH;
                    if (request != LoaderState.LAUNCH) {
                        state = LoaderState.READY;
                    }
                }
                break;
            case LAUNCH_NEXT:
                transfer.setTargetPosition(transfer.getCurrentPosition() + (int)(537 * 1.1));
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setTargetPosition(intake.getCurrentPosition() + (int)(145 * 1.1));
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elapsedTime.reset();
                isFlywheelStableGate.reset();
                Log.i("20311", "Shots = "+shots);
                state = LoaderState.LAUNCH_WAIT_LONG;
                break;
            case LAUNCH_WAIT_LONG:
                isFlywheelStableGate.update();
                if (elapsedTime.seconds() > 0.70) {
                    shots += 1;
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
