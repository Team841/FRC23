// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.C;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced; 
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private final TalonFX shoulderMotor = new TalonFX(C.CANid.shoulderMotor);
  private final TalonFX elbowMotor = new TalonFX(C.CANid.elbowMotor);

  public Arm() {

    shoulderMotor.configFactoryDefault();
    elbowMotor.configFactoryDefault();

    shoulderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

    bandWithLimitMotorCAN(shoulderMotor);
    bandWithLimitMotorCAN(elbowMotor);

    armSetBrakeMode(true);

    /* Config the sensor used for Primary PID and sensor direction */
    shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, C.Arm.kPIDLoopIdx, C.Arm.kTimeoutMs);
    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, C.Arm.kPIDLoopIdx, C.Arm.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    shoulderMotor.setSensorPhase(C.Arm.kSensorPhase);
    elbowMotor.setSensorPhase(C.Arm.kSensorPhase);

    /* Config the peak and nominal outputs */
    elbowMotor.configNominalOutputForward(C.Arm.elbow_maxOutput, C.Arm.kTimeoutMs);
    elbowMotor.configNominalOutputReverse(C.Arm.elbow_minOutput,C.Arm.kTimeoutMs);
    elbowMotor.configPeakOutputForward(C.Arm.elbow_maxOutput,C.Arm.kTimeoutMs);
    elbowMotor.configPeakOutputReverse(C.Arm.elbow_minOutput,C.Arm.kTimeoutMs);

    shoulderMotor.configNominalOutputForward(C.Arm.shoulder_maxOutput, C.Arm.kTimeoutMs);
    shoulderMotor.configNominalOutputReverse(C.Arm.shoulder_minOutput,C.Arm.kTimeoutMs);
    shoulderMotor.configPeakOutputForward(C.Arm.shoulder_maxOutput,C.Arm.kTimeoutMs);
    shoulderMotor.configPeakOutputReverse(C.Arm.shoulder_minOutput,C.Arm.kTimeoutMs);

    /* Config the Positon closed loop Gains in slotX */
    shoulderMotor.config_kF(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kff, C.Arm.kTimeoutMs);
    shoulderMotor.config_kP(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kp, C.Arm.kTimeoutMs);
    shoulderMotor.config_kI(C.Arm.kPIDLoopIdx, C.Arm.shoulder_ki, C.Arm.kTimeoutMs);
    shoulderMotor.config_kD(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kd, C.Arm.kTimeoutMs);
    shoulderMotor.config_IntegralZone(C.Arm.kPIDLoopIdx, C.Arm.shoulder_kIz, C.Arm.kTimeoutMs);

    elbowMotor.config_kF(C.Arm.kPIDLoopIdx, C.Arm.elbow_kff, C.Arm.kTimeoutMs);
    elbowMotor.config_kP(C.Arm.kPIDLoopIdx, C.Arm.elbow_kp, C.Arm.kTimeoutMs);
    elbowMotor.config_kI(C.Arm.kPIDLoopIdx, C.Arm.elbow_ki, C.Arm.kTimeoutMs);
    elbowMotor.config_kD(C.Arm.kPIDLoopIdx, C.Arm.elbow_kd, C.Arm.kTimeoutMs);
    elbowMotor.config_IntegralZone(C.Arm.kPIDLoopIdx, C.Arm.elbow_kIz, C.Arm.kTimeoutMs);

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
      shoulderMotor.setNeutralMode(NeutralMode.Brake);
      elbowMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      shoulderMotor.setNeutralMode(NeutralMode.Coast);
      elbowMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
