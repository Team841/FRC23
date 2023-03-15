// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.C;

import com.ctre.phoenix.motorcontrol.can.*;

import java.util.ResourceBundle.Control;

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



public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure.  */

  private final TalonFX shoulderMotor_starboard = new TalonFX(C.CANid.shoulderMotor_Starboard);
  private final TalonFX shoulderMotor_port = new TalonFX(C.CANid.shoulderMotor_Port);
  private final TalonFX elbowMotor = new TalonFX(C.CANid.elbowMotor);

  private final TalonSRX IntakeMotor = new TalonSRX(C.CANid.IntakeTalon); 

  public final DigitalInput shoulderHallSensor = new DigitalInput(C.Superstructure.shoulderHallChannel);

  enum ArmState{
    Home,
    Lowscore,
    Midscore,
    Highscore,
    Groundpickup,
    Substationpickup,
  }

  private ArmState armState = ArmState.Home;

  enum IntakeState{
    Cone,
    Cube,
    Empty,
  }

  private IntakeState intakeState = IntakeState.Empty;

  public Superstructure() {

    shoulderMotor_starboard.configFactoryDefault();
    shoulderMotor_port.configFactoryDefault();
    elbowMotor.configFactoryDefault();

    IntakeMotor.configFactoryDefault();

    shoulderMotor_starboard.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    shoulderMotor_port.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

    IntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

    bandWithLimitMotorCAN(shoulderMotor_starboard);
    bandWithLimitMotorCAN(shoulderMotor_port);
    bandWithLimitMotorCAN(elbowMotor);

    bandWithLimitMotorCAN(IntakeMotor);

    armSetBrakeMode(true);

    /* Config the peak and nominal outputs */
    elbowMotor.configNominalOutputForward(C.Superstructure.elbow_maxOutput, C.Superstructure.kTimeoutMs);
    elbowMotor.configNominalOutputReverse(C.Superstructure.elbow_minOutput,C.Superstructure.kTimeoutMs);
    elbowMotor.configPeakOutputForward(C.Superstructure.elbow_maxOutput,C.Superstructure.kTimeoutMs);
    elbowMotor.configPeakOutputReverse(C.Superstructure.elbow_minOutput,C.Superstructure.kTimeoutMs);

    shoulderMotor_starboard.configNominalOutputForward(C.Superstructure.shoulder_maxOutput, C.Superstructure.kTimeoutMs);
    shoulderMotor_starboard.configNominalOutputReverse(C.Superstructure.shoulder_minOutput,C.Superstructure.kTimeoutMs);
    shoulderMotor_starboard.configPeakOutputForward(C.Superstructure.shoulder_maxOutput,C.Superstructure.kTimeoutMs);
    shoulderMotor_starboard.configPeakOutputReverse(C.Superstructure.shoulder_minOutput,C.Superstructure.kTimeoutMs);

    shoulderMotor_port.configNominalOutputForward(C.Superstructure.shoulder_maxOutput, C.Superstructure.kTimeoutMs);
    shoulderMotor_port.configNominalOutputReverse(C.Superstructure.shoulder_minOutput,C.Superstructure.kTimeoutMs);
    shoulderMotor_port.configPeakOutputForward(C.Superstructure.shoulder_maxOutput,C.Superstructure.kTimeoutMs);
    shoulderMotor_port.configPeakOutputReverse(C.Superstructure.shoulder_minOutput,C.Superstructure.kTimeoutMs);

    /* Config the sensor used for Primary PID and sensor direction */
    shoulderMotor_starboard.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, C.Superstructure.kPIDLoopIdx, C.Superstructure.kTimeoutMs);
    elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, C.Superstructure.kPIDLoopIdx, C.Superstructure.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    shoulderMotor_starboard.setSensorPhase(C.Superstructure.kSensorPhase);
    elbowMotor.setSensorPhase(C.Superstructure.kSensorPhase);

    /* Config the Positon closed loop Gains in slotX */
    shoulderMotor_starboard.config_kF(C.Superstructure.kPIDLoopIdx, C.Superstructure.shoulder_kff, C.Superstructure.kTimeoutMs);
    shoulderMotor_starboard.config_kP(C.Superstructure.kPIDLoopIdx, C.Superstructure.shoulder_kp, C.Superstructure.kTimeoutMs);
    shoulderMotor_starboard.config_kI(C.Superstructure.kPIDLoopIdx, C.Superstructure.shoulder_ki, C.Superstructure.kTimeoutMs);
    shoulderMotor_starboard.config_kD(C.Superstructure.kPIDLoopIdx, C.Superstructure.shoulder_kd, C.Superstructure.kTimeoutMs);
    shoulderMotor_starboard.config_IntegralZone(C.Superstructure.kPIDLoopIdx, C.Superstructure.shoulder_kIz, C.Superstructure.kTimeoutMs);

    elbowMotor.config_kF(C.Superstructure.kPIDLoopIdx, C.Superstructure.elbow_kff, C.Superstructure.kTimeoutMs);
    elbowMotor.config_kP(C.Superstructure.kPIDLoopIdx, C.Superstructure.elbow_kp, C.Superstructure.kTimeoutMs);
    elbowMotor.config_kI(C.Superstructure.kPIDLoopIdx, C.Superstructure.elbow_ki, C.Superstructure.kTimeoutMs);
    elbowMotor.config_kD(C.Superstructure.kPIDLoopIdx, C.Superstructure.elbow_kd, C.Superstructure.kTimeoutMs);
    elbowMotor.config_IntegralZone(C.Superstructure.kPIDLoopIdx, C.Superstructure.elbow_kIz, C.Superstructure.kTimeoutMs);

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

  private void bandWithLimitMotorCAN(TalonSRX motor) {
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

  public void stopAllMotors() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, 0);
    elbowMotor.set(ControlMode.PercentOutput, 0);
  }

  public void toggleIntakeIn(){
    /* 
    if(clawNeo.get() == 0){
      clawNeo.set(C.Claw.clawNeoPercentPower);
    } else {
      clawNeo.set(0);
    } */

    if(IntakeMotor.getMotorOutputPercent()==0){
      IntakeMotor.set(ControlMode.PercentOutput, C.Superstructure.clawTalonPercentPower);
    } else {
      IntakeMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void toggleIntakeOut(){
    if(IntakeMotor.getMotorOutputPercent()==0){
      IntakeMotor.set(ControlMode.PercentOutput, -C.Superstructure.clawTalonPercentPower);
    } else {
      IntakeMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public double angleToCounts(double angle, double gearRatio) {
    return (angle / 360.0 * C.Superstructure.countsPerRev) / gearRatio;
  }

  public double countsToAngle(double counts, double gearRatio) {
    /* test */
    return counts * gearRatio / C.Superstructure.countsPerRev * 360.0; // flipped from angletocounts
  }
  
  public void setJointAngles(double shoulder_angle, double elbow_angle){
    shoulderMotor_starboard.set(TalonFXControlMode.Position, angleToCounts(shoulder_angle, C.Superstructure.shoulderGearRatio));
    elbowMotor.set(TalonFXControlMode.Position, angleToCounts(elbow_angle, C.Superstructure.elbowGearRatio));
  }
  public void testjoint(){
    setJointAngles(90, 0);
  }

  @Override
  public void periodic() {



    SmartDashboard.putNumber("Shouldmotor.DNG", shoulderMotor_starboard.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbowmotor.DNG", elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ShoulderMotorOutput", shoulderMotor_starboard.getMotorOutputPercent());
    SmartDashboard.putNumber("ElbowMotorOutput", elbowMotor.getMotorOutputPercent());
    SmartDashboard.putBoolean("Hall Sensor", shoulderHallSensor.get());

    SmartDashboard.putNumber("shoulder Postion", countsToAngle(shoulderMotor_starboard.getSelectedSensorPosition(), C.Superstructure.shoulderGearRatio));
    SmartDashboard.putNumber("elbow Postion", countsToAngle(elbowMotor.getSelectedSensorPosition(), C.Superstructure.elbowGearRatio));
  }

/* funky manual code */
  public void moveShoulderSlowUp() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, C.Superstructure.testMove);
  }

  public void moveShoulderSlowDown() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, -C.Superstructure.testMove);
  }

  public void stopShoulder() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, 0);
  }
    
  public void moveElbowSlowUp() {
    elbowMotor.set(ControlMode.PercentOutput, C.Superstructure.testMove);
  }

  public void moveElbowSlowDown() {
    elbowMotor.set(ControlMode.PercentOutput, -C.Superstructure.testMove);
  }

  public void stopElbow() {
    elbowMotor.set(ControlMode.PercentOutput, 0);
  }
/*  */

}


/*
 *  Pick from pickstation
 *  Pick up from ground
 *  scrore low and mid
 *  storage inside robot
 */