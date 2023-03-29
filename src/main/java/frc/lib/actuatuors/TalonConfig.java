package frc.lib.actuatuors;

import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;


public class TalonConfig {


    public TalonFXConfiguration getConfiguraton(MotorOutputConfigs _MotorOutputConfigs, CurrentLimitsConfigs _CurrentLimitConfigs, MotionMagicConfigs _MotionMagicConfigs, Slot0Configs slot0){

        var configuration = new TalonFXConfiguration();

        configuration.MotorOutput = _MotorOutputConfigs;
        configuration.CurrentLimits = _CurrentLimitConfigs;
        configuration.MotionMagic = _MotionMagicConfigs;
        configuration.Slot0 = slot0;

        return configuration;
    }


    public MotionMagicConfigs configMotionMagic(double _MMCV, double _MMA, double _MMJ){

        // MMCV: Motion Magic Cruise Velocity, MMA: Motion Magic Acceleration, MMJ: Motion Magic Jerk

        var MotionMagicConfigs = new MotionMagicConfigs();

        MotionMagicConfigs.MotionMagicCruiseVelocity = _MMCV;
        MotionMagicConfigs.MotionMagicAcceleration = _MMA;
        MotionMagicConfigs.MotionMagicJerk = _MMJ;

        return MotionMagicConfigs;
    }

    public CurrentLimitsConfigs configCurrentLimit(double SupplyCurrentLimit, double SupplyCurrentThreshold, double SupplyTimeThreshold){

        var CurrentLimitConfigs = new CurrentLimitsConfigs();

        CurrentLimitConfigs.SupplyCurrentLimitEnable = true;
        CurrentLimitConfigs.SupplyCurrentLimit = SupplyCurrentLimit;
        CurrentLimitConfigs.SupplyCurrentThreshold = SupplyCurrentThreshold;
        CurrentLimitConfigs.SupplyTimeThreshold = SupplyTimeThreshold;

        return CurrentLimitConfigs;
    }

    public MotorOutputConfigs configMotorOutput(boolean _brakeMode, boolean _inverted, double _maxOutput, double _minOutput){

        var MotorOutputConfigs = new MotorOutputConfigs();

        if (_brakeMode){
            MotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        }
        else{
            MotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        }

        if (_inverted){
            MotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        }
        else{
            MotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        MotorOutputConfigs.PeakForwardDutyCycle = _maxOutput;
        MotorOutputConfigs.PeakReverseDutyCycle = _minOutput;

        return MotorOutputConfigs;
    }
}
