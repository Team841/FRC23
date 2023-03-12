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
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Claw extends SubsystemBase {
  /** Creates a new Claw. */

  Solenoid clawSolenoid_1 = new Solenoid(PneumaticsModuleType.REVPH, C.Claw.kSolenoidPortForward);
  Solenoid clawSolenoid_2 = new Solenoid(PneumaticsModuleType.REVPH, C.Claw.kSolenoidPortBackward);

  // CANSparkMax clawNeo = new CANSparkMax(C.CANid.clawNeo, MotorType.kBrushless);

  WPI_TalonSRX clawTalon = new WPI_TalonSRX(C.CANid.clawTalon);

  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);


  public Claw() {

    /* 
    clawNeo.restoreFactoryDefaults();

    clawNeo.setSmartCurrentLimit(C.Drive.currentLimit); //Current limit at number of amps 

    clawNeo.setIdleMode(IdleMode.kBrake); */

    clawTalon.configFactoryDefault();

    clawTalon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,255);
    clawTalon.setStatusFramePeriod(StatusFrame.Status_1_General,40);
    clawTalon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
    clawTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255); 
    clawTalon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,255);
    clawTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
    clawTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,255);
    clawTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
    clawTalon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);

    clawTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

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

  public void toggleIntake(){
    /* 
    if(clawNeo.get() == 0){
      clawNeo.set(C.Claw.clawNeoPercentPower);
    } else {
      clawNeo.set(0);
    } */

    if(clawTalon.get()==0){
      clawTalon.set(C.Claw.clawTalonPercentPower);
    } else {
      clawTalon.set(0);
    }
  }

  @Override
  public void periodic() {
    
  }
}
