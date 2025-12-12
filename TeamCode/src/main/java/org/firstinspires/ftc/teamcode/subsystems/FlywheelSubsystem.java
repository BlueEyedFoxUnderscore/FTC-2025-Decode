package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class FlywheelSubsystem {
    private static boolean loggingEnabled = false;

    private final Telemetry telemetry;
    private final DcMotorEx shooter1, shooter2;
    private BooleanSupplier isLoaderPermittingSpeedChanges;
    /**
     * The current requested speed to be used on invocation of update()
     */
    private double speed = 0, threshold = 0;

    /**
     * Constructs a new flywheel.
     * @param shooter1 First flywheel motor. Interchangeable with shooter2.
     * @param shooter2 Second flywheel motor. Interchangeable with shooter1.
     */
    final double fCoeff = 14.9244385071*1.01; // 16.072472238457042;
    private int stabilityThreshold = 80;
    public FlywheelSubsystem(@NonNull DcMotorEx shooter1, @NonNull DcMotorEx shooter2, @NonNull Telemetry telemetry) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;


        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setVelocityPIDFCoefficients(85, .10, 0, fCoeff);
        shooter2.setVelocityPIDFCoefficients(85, .10, 0, fCoeff);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void testp(double p)
    {
        shooter1.setVelocityPIDFCoefficients(p, 0, 0, fCoeff);
        shooter2.setVelocityPIDFCoefficients(p, 0, 0, fCoeff);
    }
    public void setReady(BooleanSupplier ready) {
        isLoaderPermittingSpeedChanges = ready;
    }

    /**
     * Unit indicator for Volts
     */
    final double VOLTS = 1;

    final double MAX_VOLTAGE = 12 * VOLTS; // nominal voltage

    /**
     * Updates the speed with respect to the current requested speed.
     */
     public boolean isSpeedChangeRequested() {return priorspeed!=speed && speed!=0.0;}
    double priorspeed=0;
    public void update() {
        if (isLoaderPermittingSpeedChanges.getAsBoolean()) {
            if(priorspeed != speed) {
            	priorspeed=speed;
            	if (loggingEnabled) Log.i("20311", "Speed was actually set to "+speed);
	            setVelocity(speed);
			}
        }
    }

//    public void update() {
 //       if (speed == 0) {
   //         setPower(0);
        //} else
            //telemetry.addLine("Using PID");
     //       if(isLoaderReadyToShootSupplier.getAsBoolean()) {
       //         setVelocity(speed);
         //   }
            //telemetry.addData("Requested velocity", String.valueOf(requestedSpeed));
            //telemetry.addData("Actual velocity", String.valueOf(getSpeed()));
       // }
  //  }



    private static final double TPS_PER_RPM = 28. / 60.;

    /**
     * Sets the threshold and the speed. <br/>
     * The threshold is not relative to the speed. If the speed is above the threshold, it automatically switches to PID mode. Otherwise, it applies full power.
     * @param speed Requested speed, in RPM.
     */

    public void setRequested(double speed, String name) {
        this.speed = speed;
        if (loggingEnabled) Log.i("20311", "FLYWHEEL REQUEST speed " + speed +" from "+ name);
    }



    /**
     * Sets the target velocity of both motors.
     * Also sets the run mode to DcMotor.RunMode.RUN_USING_ENCODER to be safe.
     * @param rpm The requested velocity, in RPM
     */
    private void setVelocity(double rpm) {
        //setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocity(toTPS(rpm));
        shooter2.setVelocity(toTPS(rpm));
    }


    /**
     * Sets the run mode (DcMotor.RunMode) of both motors.
     * @param runMode The desired run mode.
     */
    private void setRunMode(DcMotor.RunMode runMode){
        if (loggingEnabled) Log.i("20311", "shooter run mode set");
        shooter1.setMode(runMode);
        shooter2.setMode(runMode);
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
     * Gets the effective speed of both motors, in RPM.
     * @return The (effective) speed of the faster motor.
     */
    public double getSpeed1() {
        return toRPM(shooter1.getVelocity());
    }

    public double getSpeed2() {
        return toRPM(shooter2.getVelocity());
    }

    /**
     * Function to return whether the motor has stabilized (reached the target velocity)
     * @return Whether the difference between our requested and actual speeds is less than the stability threshold
     */
    public boolean isStable() {
        return Math.abs(speed  - getSpeed1()) < stabilityThreshold;
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
