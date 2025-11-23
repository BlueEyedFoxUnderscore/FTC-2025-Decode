package org.firstinspires.ftc.teamcode.opmodes.teleop;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp//(name = "***FTC OpMode 2", group = "TeleOp")

public class MainControl extends OpMode {
    VoltageSensor voltageSensor;
    LoaderSubsystem shooter;
    Flywheel flywheel;
    DcMotorEx leftFront, rightFront, leftBack, rightBack, intake, transfer;
    DcMotorEx shooter1, shooter2;
    Servo gate;
    IMU imu;
    double headingOffset = 0;
    boolean imuReady = false;
    ElapsedTime imuTimeout = new ElapsedTime();

    // Motor direction constants (adjust if wiring differs)
    DcMotor.Direction LF_DIR = DcMotor.Direction.FORWARD;
    DcMotor.Direction RF_DIR = DcMotor.Direction.REVERSE;
    DcMotor.Direction LR_DIR = DcMotor.Direction.FORWARD;
    DcMotor.Direction RR_DIR = DcMotor.Direction.REVERSE;
    private GateSubsystem gateSubsystem;

    //GoBildaPinpointDriver odometry1;
    //GoBildaPinpointDriver odometry2;

    //private MotorGroup shooterGroup;

    @Override
    public void init() {
        // Initialize odometry
        //odometry1 = hardwareMap.get(GoBildaPinpointDriver.class,"odo1");
        //odometry2 = hardwareMap.get(GoBildaPinpointDriver.class,"odo2");

        // Initialize wheels
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set directions
        leftFront.setDirection(LF_DIR);
        rightFront.setDirection(RF_DIR);
        leftBack.setDirection(LR_DIR);
        rightBack.setDirection(RR_DIR);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "slave1");
        gate = hardwareMap.get(Servo.class, "gate");

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        gateSubsystem = new GateSubsystem(gate);
        shooter = new LoaderSubsystem(transfer, gateSubsystem, intake, telemetry);
        flywheel = new Flywheel(shooter1, shooter2, telemetry, voltageSensor);

        imu = hardwareMap.get(IMU.class, "imu");
        // Initialize IMU directly
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        );
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        imuTimeout.reset();

        telemetry.addLine("Initialized - waiting for IMU calibration...");
        telemetry.update();
        transfer.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(13, 0, 0, 0));
        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(30, 0,0, 0));
        transfer.setTargetPosition(0);
        intake.setTargetPosition(0);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //odometry1.setOffsets(-84.0, -168.0, DistanceUnit.MM); // T U N E   T H E S E
        //odometry1.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //odometry2.setOffsets(-84.0, -168.0, DistanceUnit.MM); // T U N E   T H E S E
        //odometry2.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    private double offset = 0;

    public void loop() {
        //odometry1.update();
        //odometry2.update();

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.y) {
            offset = getRawHeading();
        }

        double botHeading = getRawHeading() - offset;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        if (gamepad1.right_bumper) {
            shooter.intake();
        } else {
            shooter.cancelIntake();
        }
        if (gamepad1.left_bumper) {
            flywheel.setRequested(2800, 2400);
            if (flywheel.isStable()) shooter.launch();
        } else {
            flywheel.setRequested(0, 0);
            shooter.cancelLaunch();
        }

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

        shooter.update();
        flywheel.update();

        telemetry.addLine(String.valueOf(gamepad1.right_trigger));
        telemetry.update();
        telemetry.addLine(String.valueOf(flywheel.getSpeed()));
        telemetry.clear();
//        gate.setPosition(gamepad1.right_trigger);
    }

    /**
     * Sets the speed of both motors.
     * @param speed Requested speed.
     * @deprecated
     */
    public void setFlywheelSpeed(double speed) {
        shooter1.setVelocity(speed);
        shooter2.setVelocity(speed);
    }

    int count = 0;

    /**
     * Gets the current absolute heading of the robot in radians.
     * Uses XYZ order.
     * @return The absolute heading of the robot in radians.
     */
    private double getRawHeading() {
        Orientation angles = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );

        return angles.thirdAngle;
    }
}

class GateSubsystem {
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

class LoaderSubsystem {
    private final Telemetry telemetry;
    private int previousPositionTransfer;

    /**
     * Starts the launch sequence.
     * <br/>
     * <font color="#CF8E6D">WARNING</font>: Once started, this cannot be stopped.
     */
    void launch() {
        request = LoaderState.LAUNCH;
    }

    void cancelLaunch() {
        if(request == LoaderState.LAUNCH) request = LoaderState.READY;
    }

    void cancelIntake() {
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
        REGRESS_GATE_WAIT_STABLE, LAUNCH_NEXT, LAUNCH_WAIT_LONG,
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

    /**
     * Constructs a new Flywheel object using all the different required motors.
     * @param transfer The transfer motor.
     * @param gate The gate servo.
     */
    public LoaderSubsystem(DcMotorEx transfer, GateSubsystem gate, DcMotorEx intake, Telemetry telemetry) {
        this.transfer = transfer;
        this.gate = gate;
        this.intake = intake;
        this.telemetry = telemetry;
        setGateOpen(true);
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
                    state = LoaderState.LAUNCH;
                }
                break;
            case PRE_INTAKE:
                telemetry.addData("is stable", gate.isStable());
                if (gate.isStable()) {
                    state = LoaderState.INTAKE;
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

                //
            case LAUNCH:
                //intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setTargetPosition(transfer.getCurrentPosition() + 537 / 1);
                transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                transfer.setPower(1);
                elapsedTime.reset();
                state = LoaderState.LAUNCH_WAIT;
                break;
            case LAUNCH_WAIT:
                if (elapsedTime.seconds() > 0.20) { // 0.20
                    if (request == LoaderState.READY) {
                        gate.close();
                        state = LoaderState.GATE_WAIT;
                    }
                    else state = LoaderState.LAUNCH_NEXT;
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
                state = LoaderState.LAUNCH_WAIT_LONG;
                break;
            case LAUNCH_WAIT_LONG:
                if (elapsedTime.seconds() > 0.20) { // 0.40
                    if (request == LoaderState.READY) {
                        gate.close();
                        state = LoaderState.GATE_WAIT;
                    }
                    else state = LoaderState.LAUNCH_NEXT;
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
    int of(double x) {
        return (int) x;
    }
}



class Flywheel {
    private final Telemetry telemetry;
    private final VoltageSensor voltageSensor;
    private final DcMotorEx shooter1, shooter2;
    /**
     * The current requested speed to be used on invocation of update()
     */
    private double speed = 0, threshold = 0;

    /**
     * Constructs a new flywheel.
     * @param shooter1 First flywheel motor. Interchangeable with shooter2.
     * @param shooter2 Second flywheel motor. Interchangeable with shooter1.
     */
    Flywheel(@NonNull DcMotorEx shooter1, @NonNull DcMotorEx shooter2, @NonNull Telemetry telemetry, @NonNull VoltageSensor voltageSensor) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;


        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        final double fCoeff = 1.0 / 28.0 * 6000.0 / 60.0 * 2.0 * 2800. / 1180. * 2750. / 2900.;
        shooter1.setVelocityPIDFCoefficients(100, 0, 0, fCoeff);
        shooter2.setVelocityPIDFCoefficients(100, 0, 0, fCoeff);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Unit indicator for Volts
     */
    final double VOLTS = 1;

    final double MAX_VOLTAGE = 12 * VOLTS; // nominal voltage

    /**
     * Updates the speed with respect to the current requested speed.
     */
    public void update() {
        if (speed == 0) {
            setPower(0);
        } /*else if (getSpeed() < threshold) {
            telemetry.addLine("Full power");
            setPower(1);
        }/**/ else {
            telemetry.addLine("Using PID");
            double requestedSpeed = speed;
            setVelocity(requestedSpeed);
            telemetry.addData("Requested velocity", String.valueOf(requestedSpeed));
            telemetry.addData("Adjusted velocity", String.valueOf(speed * voltageSensor.getVoltage()/MAX_VOLTAGE));
            telemetry.addData("Actual velocity", String.valueOf(getSpeed()));
        }
    }


    private static final double TPS_PER_RPM = 28. / 60.;

    /**
     * Sets the threshold and the speed. <br/>
     * The threshold is not relative to the speed. If the speed is above the threshold, it automatically switches to PID mode. Otherwise, it applies full power.
     * @param speed Requested speed, in RPM.
     * @param threshold Requested threshold, in RPM. This is not relative.
     */
    public void setRequested(double speed, double threshold) {
        this.speed = speed;
        this.threshold = threshold;
    }

    /**
     * Sets the threshold and the speed. <br/>
     * The threshold is not relative to the speed. If the speed is above the threshold, it automatically switches to PID mode. Otherwise, it applies full power.<br/>
     * This is the chained version of this method.
     * @param speed Requested speed, in RPM.
     * @param threshold Requested threshold, in RPM. This is not relative.
     * @return This object (for chaining)
     */
    public Flywheel setRequested_(double speed, double threshold) {
        this.speed = speed;
        this.threshold = threshold;
        return this;
    }

    /**
     * Sets the target velocity of both motors.
     * Also sets the run mode to DcMotor.RunMode.RUN_USING_ENCODER to be safe.
     * @param rpm The requested velocity, in RPM
     */
    private void setVelocity(double rpm) {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocity(toTPS(rpm));
        shooter2.setVelocity(toTPS(rpm));
    }

    /**
     * Sets the target velocity of both motors.
     * Also sets the run mode to DcMotor.RunMode.RUN_USING_ENCODER to be safe.<br/>
     * This is the chained version of this method.
     * @param rpm The requested velocity, in RPM
     * @return This object (for chaining)
     */
    private Flywheel setVelocity_(double rpm) {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocity(toTPS(rpm));
        shooter2.setVelocity(toTPS(rpm));
        return this;
    }

    /**
     * Sets the run mode (DcMotor.RunMode) of both motors.
     * @param runMode The desired run mode.
     */
    private void setRunMode(DcMotor.RunMode runMode){
        shooter1.setMode(runMode);
        shooter2.setMode(runMode);
    }

    /**
     * Sets the run mode (DcMotor.RunMode) of both motors.
     * This is the chained version of this method.<br/>
     * @param runMode The desired run mode.
     * @return This object (for chaining)
     */
    private Flywheel setRunMode_(DcMotor.RunMode runMode){
        shooter1.setMode(runMode);
        shooter2.setMode(runMode);
        return this;
    }

    /**
     * Sets the power applied to both motors.
     * Also sets the run mode of both motor to DcMotor.RunMode.RUN_WITHOUT_ENCODER to be safe.
     * @param power The requested power.
     */
    private void setPower(double power) {
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    /**
     * Sets the power applied to both motors.
     * Also sets the run mode of both motors to DcMotor.RunMode.RUN_WITHOUT_ENCODER to be safe.<br/>
     * This is the chained version of this method.
     *
     * @param power The requested power.
     * @return This object (for chaining).
     */
    private Flywheel setPower_(double power) {
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setPower(power);
        shooter2.setPower(power);
        return this;
    }



    /**
     * Gets the effective speed of both motors, in RPM.
     * @return The (effective) speed of the faster motor.
     */
    public double getSpeed() {
        return toRPM(Math.max(shooter1.getVelocity(), shooter2.getVelocity()));
    }


    private int stabilityThreshold = 60;

    /**
     * Function to return whether the motor has stabilized (reached the target velocity)
     * @return Whether the difference between our requested and actual speeds is less than the stability threshold
     */
    public boolean isStable() {
        return Math.abs(speed  - getSpeed()) < stabilityThreshold;
    }


    /**
     * Takes in a number in rotations per minute and returns ticks per second.
     * @param RPM The input RPM
     * @return The input RPM, in TPS
     */
    private double toTPS(double RPM) {
        return RPM * TPS_PER_RPM;
    }


    /**
     * Takes in a number in ticks per second and returns rotations per minute.
     * @param TPS The input TPS
     * @return The input TPS, in RPM
     */
    private double toRPM(double TPS) {
        return TPS / TPS_PER_RPM;
    }


    void dispose() {}
}