// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.C;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; 
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced; 
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

/* import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance; */
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Arm extends SubsystemBase {
  /** Creates a new Arm.  */

  private final TalonFX shoulderMotor_starboard = new TalonFX(C.CANid.shoulderMotor_Starboard);
  private final TalonFX shoulderMotor_port = new TalonFX(C.CANid.shoulderMotor_Port);
  private final TalonFX elbowMotor = new TalonFX(C.CANid.elbowMotor);
  public final DigitalInput shoulderHallSensor = new DigitalInput(C.Arm.shoulderHallChannel);

  public Arm() {

    shoulderMotor_starboard.configFactoryDefault();
    shoulderMotor_port.configFactoryDefault();
    elbowMotor.configFactoryDefault();

    shoulderMotor_starboard.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    shoulderMotor_port.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

    bandWithLimitMotorCAN(shoulderMotor_starboard);
    bandWithLimitMotorCAN(shoulderMotor_port);
    bandWithLimitMotorCAN(elbowMotor);

    armSetBrakeMode(true);

    /* Config the peak and nominal outputs */
    elbowMotor.configNominalOutputForward(C.Arm.elbow_maxOutput, C.Arm.kTimeoutMs);
    elbowMotor.configNominalOutputReverse(C.Arm.elbow_minOutput,C.Arm.kTimeoutMs);
    elbowMotor.configPeakOutputForward(C.Arm.elbow_maxOutput,C.Arm.kTimeoutMs);
    elbowMotor.configPeakOutputReverse(C.Arm.elbow_minOutput,C.Arm.kTimeoutMs);

    shoulderMotor_starboard.configNominalOutputForward(C.Arm.shoulder_maxOutput, C.Arm.kTimeoutMs);
    shoulderMotor_starboard.configNominalOutputReverse(C.Arm.shoulder_minOutput,C.Arm.kTimeoutMs);
    shoulderMotor_starboard.configPeakOutputForward(C.Arm.shoulder_maxOutput,C.Arm.kTimeoutMs);
    shoulderMotor_starboard.configPeakOutputReverse(C.Arm.shoulder_minOutput,C.Arm.kTimeoutMs);

    shoulderMotor_port.configNominalOutputForward(C.Arm.shoulder_maxOutput, C.Arm.kTimeoutMs);
    shoulderMotor_port.configNominalOutputReverse(C.Arm.shoulder_minOutput,C.Arm.kTimeoutMs);
    shoulderMotor_port.configPeakOutputForward(C.Arm.shoulder_maxOutput,C.Arm.kTimeoutMs);
    shoulderMotor_port.configPeakOutputReverse(C.Arm.shoulder_minOutput,C.Arm.kTimeoutMs);

    
    /* Config the sensor used for Primary PID and sensor direction */
    shoulderMotor_starboard.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, C.Arm.kPIDLoopIdx, C.Arm.kTimeoutMs);
    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, C.Arm.kPIDLoopIdx, C.Arm.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    shoulderMotor_starboard.setSensorPhase(C.Arm.kSensorPhase);
    elbowMotor.setSensorPhase(C.Arm.kSensorPhase);

    /* Config the Positon closed loop Gains in slotX */
    shoulderMotor_starboard.config_kF(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kff, C.Arm.kTimeoutMs);
    shoulderMotor_starboard.config_kP(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kp, C.Arm.kTimeoutMs);
    shoulderMotor_starboard.config_kI(C.Arm.kPIDLoopIdx, C.Arm.shoulder_ki, C.Arm.kTimeoutMs);
    shoulderMotor_starboard.config_kD(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kd, C.Arm.kTimeoutMs);
    shoulderMotor_starboard.config_IntegralZone(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kIz, C.Arm.kTimeoutMs);

    elbowMotor.config_kF(C.Arm.kPIDLoopIdx, C.Arm.elbow_kff, C.Arm.kTimeoutMs);
    elbowMotor.config_kP(C.Arm.kPIDLoopIdx, C.Arm.elbow_kp, C.Arm.kTimeoutMs);
    elbowMotor.config_kI(C.Arm.kPIDLoopIdx, C.Arm.elbow_ki, C.Arm.kTimeoutMs);
    elbowMotor.config_kD(C.Arm.kPIDLoopIdx, C.Arm.elbow_kd, C.Arm.kTimeoutMs);
    elbowMotor.config_IntegralZone(C.Arm.kPIDLoopIdx, C.Arm.elbow_kIz, C.Arm.kTimeoutMs);

    shoulderMotor_port.follow(shoulderMotor_starboard);
    shoulderMotor_port.setInverted(true);

  }

  private void bandWithLimitMotorCAN(TalonFX motor) {
    
    motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,255);
    motor.setStatusFramePeriod(StatusFrame.Status_1_General,40);
    motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255); 
    motor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,255);
    motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
    motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,255);
    motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
    motor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);

  }

  public void armSetBrakeMode(boolean brakeMode) {
    if (brakeMode) {
      shoulderMotor_starboard.setNeutralMode(NeutralMode.Brake);
      shoulderMotor_port.setNeutralMode(NeutralMode.Brake);
      elbowMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      shoulderMotor_starboard.setNeutralMode(NeutralMode.Coast);
      shoulderMotor_port.setNeutralMode(NeutralMode.Coast);
      elbowMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void moveShoulder(double speed) {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, speed);
  }
  
  public void moveElbow(double speed) {
    elbowMotor.set(ControlMode.PercentOutput, speed);
  }

  /*  */

                public void moveShoulderSlowUp() {
                  shoulderMotor_starboard.set(ControlMode.PercentOutput, C.Arm.testMove);
                }

                public void moveShoulderSlowDown() {
                  shoulderMotor_starboard.set(ControlMode.PercentOutput, -C.Arm.testMove);
                }

                public void stopShoulder() {
                  shoulderMotor_starboard.set(ControlMode.PercentOutput, 0);
                }
                
                public void moveElbowSlowUp() {
                  elbowMotor.set(ControlMode.PercentOutput, C.Arm.testMove);
                }

                public void moveElbowSlowDown() {
                  elbowMotor.set(ControlMode.PercentOutput, -C.Arm.testMove);
                }

                public void stopElbow() {
                  elbowMotor.set(ControlMode.PercentOutput, 0);
                }

  /*  */

  public void stopAllMotors() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, 0);
    elbowMotor.set(ControlMode.PercentOutput, 0);
  }

  public double angleToCounts(double angle) {
    return (angle / 360 * C.Arm.countsPerRev) / C.Arm.shoulderGearRatio;
  }

  public double countsToAngle(double counts) {
    return counts * C.Arm.shoulderGearRatio / C.Arm.countsPerRev * 360; // flipped from angletocounts
  }
  
  public void setJointAngles(double shoulder_angle, double elbow_angle){
    shoulderMotor_starboard.set(TalonFXControlMode.Position, angleToCounts(shoulder_angle));
    elbowMotor.set(TalonFXControlMode.Position, angleToCounts(elbow_angle));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shouldmotor", shoulderMotor_starboard.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbowmotor", elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ShoulderMotorOutput", shoulderMotor_starboard.getMotorOutputPercent());
    SmartDashboard.putNumber("ElbowMotorOutput", elbowMotor.getMotorOutputPercent());
    SmartDashboard.putBoolean("Hall Sensor", shoulderHallSensor.get());

    SmartDashboard.putNumber("shoulder Postion", countsToAngle(shoulderMotor_starboard.getSelectedSensorPosition()));
    SmartDashboard.putNumber("elbow Postion", countsToAngle(elbowMotor.getSelectedSensorPosition()));
  }


}


/*
 *  Pick from pickstation
 *  Pick up from ground
 *  scrore low and mid
 * 
 * 
 * 
 * 
 * 
 * 
 */