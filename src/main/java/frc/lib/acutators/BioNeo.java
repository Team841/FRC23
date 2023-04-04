package frc.lib.acutators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class BioNeo extends CANSparkMax {


    public BioNeo(int CANid, int CurrentLimit){
        super(CANid, MotorType.kBrushless);

        this.restoreFactoryDefaults();

        this.setSmartCurrentLimit(CurrentLimit);
        
    }


    public RelativeEncoder getMotorEncoder(){
        return this.getEncoder();
    }
}
