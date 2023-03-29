package frc.lib.actuatuors;

import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

public class BioFalcon extends TalonFX {

    public BioFalcon(int CANid, TalonFXConfiguration config){

        super(CANid);

        // Factory Reset
        this.getConfigurator().apply(new TalonFXConfiguration());

        // Apply Configs
        this.getConfigurator().apply(config);

    }

    public void setBrakeMode(boolean brakeMode){

        var configruation = this.getConfigurator();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        configruation.refresh(motorOutputConfigs);

        motorOutputConfigs.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        configruation.apply(motorOutputConfigs);
    }

    pu

}
