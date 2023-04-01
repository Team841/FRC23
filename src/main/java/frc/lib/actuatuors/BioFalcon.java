package frc.lib.actuatuors;

import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import frc.lib.math.*;

import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

public class BioFalcon extends TalonFX {


    private final conversions _conversions = new conversions();
    private final double CountsPerRev = 2048;

    public BioFalcon(int CANid, TalonFXConfiguration config){

        super(CANid);

        // Factory Reset
        this.getConfigurator().apply(new TalonFXConfiguration());

        // Apply Configs
        this.getConfigurator().apply(config);

    }

    public void setBrakeMode(boolean brakeMode) {

        var configuration = this.getConfigurator();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        configuration.refresh(motorOutputConfigs);

        motorOutputConfigs.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        configuration.apply(motorOutputConfigs);
    }

    public void setMoveToAngleMM(double angle){
        this.setControl(new MotionMagicDutyCycle(_conversions.angleToCounts(angle, this.getGearRatio()), false, 0.0, 0, true));
    }

    public void moveDutyCycle(double speed){
        this.setControl(new DutyCycleOut(speed));
    }

    public boolean isAtPosition(double angle, double tolerance){
        return (Math.abs(this.getAngle() - angle) <= tolerance);
    }

    public void resetFeedbackSensor(){
        this.setRotorPosition(0);
    }

    public double getDutyCycleOutput(){
        return this.getDutyCycle().getValue();
    }

    public double getAngle(){
        return _conversions.countsToAngle(this.getRotorPosition().getValue(), this.getGearRatio());
    }

    public double getCounts(){
        return this.getRotorPosition().getValue();
    }

    public double getGearRatio(){
        var configuration = this.getConfigurator();
        FeedbackConfigs feedback = new FeedbackConfigs();

        configuration.refresh(feedback);

        return feedback.SensorToMechanismRatio;
    }






}
