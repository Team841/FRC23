package frc.lib.acutators;

import com.revrobotics.SparkMaxPIDController;

public class BioNeoConfigs {

  public void SLOT0(SparkMaxPIDController controller, Gains _gains) {
    controller.setP(_gains.kP, 0);
    controller.setI(_gains.kI, 0);
    controller.setD(_gains.kD, 0);
    controller.setFF(_gains.kF, 0);
    controller.setIZone(_gains.kIz, 0);
    controller.setSmartMotionMaxVelocity(_gains.SMMV, 0);
    controller.setSmartMotionMaxAccel(_gains.SMMA, 0);
    controller.setOutputRange(-1, 1);
  }

  public void SLOT1(SparkMaxPIDController controller, Gains _gains) {
    controller.setP(_gains.kP, 1);
    controller.setI(_gains.kI, 1);
    controller.setD(_gains.kD, 1);
    controller.setFF(_gains.kF, 1);
    controller.setIZone(_gains.kIz, 1);
    controller.setSmartMotionMaxVelocity(_gains.SMMV, 1);
    controller.setSmartMotionMaxAccel(_gains.SMMA, 1);
    controller.setOutputRange(-1, 1);
  }
}
