package frc.lib.acutators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class BioNeo extends CANSparkMax {

    RelativeEncoder ENCODER = this.getEncoder();

    public BioNeo(int CANid, int CurrentLimit){
        super(CANid, MotorType.kBrushless);

        this.restoreFactoryDefaults();

        this.setSmartCurrentLimit(CurrentLimit);



    }


    public RelativeEncoder getEncoder(){
        return ENCODER;
    }

    public void resetEncoder(){
        ENCODER.setPosition(0);
    }

}
