// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.C;
import edu.wpi.first.wpilibj.Compressor;
/* 
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid; */
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Claw extends SubsystemBase {
  /** Creates a new Claw. */

  Solenoid clawSolenoid_1 = new Solenoid(PneumaticsModuleType.REVPH, C.Claw.kSolenoidPortForward);
  Solenoid clawSolenoid_2 = new Solenoid(PneumaticsModuleType.REVPH, C.Claw.kSolenoidPortBackward);

  CANSparkMax clawNeo = new CANSparkMax(C.CANid.clawNeo, MotorType.kBrushless);

  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);


  public Claw() {

    clawNeo.restoreFactoryDefaults();

    clawNeo.setSmartCurrentLimit(C.Drive.currentLimit); //Current limit at number of amps 

    clawNeo.setIdleMode(IdleMode.kBrake);

  }

  public void setClawOpen() {
    clawSolenoid_1.set(false);
    clawSolenoid_2.set(true);
  }

  public void setClawClose() {
    clawSolenoid_1.set(true);
    clawSolenoid_2.set(false); 
  }

  public boolean isClawOpen() {
    return clawSolenoid_1.get();
  }

  public void toggleClawNeo(){
    if(clawNeo.get() == 0){
      clawNeo.set(0.5);
    } else {
      clawNeo.set(0);
    }
  }

  @Override
  public void periodic() {
    
  }
}
