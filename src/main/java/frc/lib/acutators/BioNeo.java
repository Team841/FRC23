package frc.lib.acutators;

import com.revrobotics.CANSparkMax;

public class BioNeo extends CANSparkMax {

  public BioNeo(int CANid, int SmartCurrentLimit) {
    super(CANid, MotorType.kBrushless);

    this.restoreFactoryDefaults();

    this.setSmartCurrentLimit(SmartCurrentLimit);
  }
}
