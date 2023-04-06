// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.C;

import com.ctre.phoenix.motorcontrol.can.*;

import java.security.Key;

import org.opencv.video.SparseOpticalFlow;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.SparkMaxRelativeEncoder;
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

  private final TalonFX IntakeMotor = new TalonFX(C.CANid.IntakeTalon); 

  DigitalInput ElbowIndexSensor = new DigitalInput(C.Superstructure.Elbow_Index_Channel);
  DigitalInput ShoulderIndexSensor = new DigitalInput(C.Superstructure.Shoulder_Index_Channel);
  
  private int TimerCounter = 0; 
  private boolean expired = true; 

  enum States{
    Home,
    coDriverMode,
    ExtendOut, 
    ExtendIn,
    PreExtractIn,
    MidScoreCube,
    TopScoreCube,
    TopSCoreCone,
    MidScoreCone,
    LowScore,
    MidScore,
    HighScore,
    LowPortal,
    HighPortal,
    Ground,
    Pickup,
    Manual
  }

  enum GamePiece{
    Cone,
    Cube,
    Empty
  }

  public States coDriveCommand = States.Home;
  private States pickup = States.Home;
  private States armState = States.Home;
  private GamePiece intakeState = GamePiece.Empty;
  private GamePiece toggle = GamePiece.Cone;

  public Superstructure() {

    shoulderMotor_starboard.configFactoryDefault();
    shoulderMotor_port.configFactoryDefault();
    elbowMotor.configFactoryDefault();

    IntakeMotor.configFactoryDefault();

    shoulderMotor_starboard.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    shoulderMotor_port.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

    IntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));

    /*
    bandWithLimitMotorCAN(shoulderMotor_starboard);
    bandWithLimitMotorCAN(shoulderMotor_port);
    bandWithLimitMotorCAN(elbowMotor);

    bandWithLimitMotorCAN(IntakeMotor); */

    armSetBrakeMode(false);

    IntakeMotor.setNeutralMode(NeutralMode.Brake);

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

    
    /* Set acceleration and vcruise velocity - see documentation */
    /*shoulderMotor_starboard.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    shoulderMotor_port.configMotionAcceleration(6000, Constants.kTimeoutMs);

    elbowMotor.configMotionCruiseVelocity(15000, c.ktimeouts);
    elbowMotor.configMotionAcceleration(1005, alfkjdlsa); */

    /* Set up motion Magic */
    shoulderMotor_starboard.configMotionAcceleration(C.Superstructure.shoulderAccelerationLimitv5);
    elbowMotor.configMotionAcceleration(C.Superstructure.elbowAccelerationLimitv5);
    shoulderMotor_starboard.configMotionCruiseVelocity(C.Superstructure.shoulderVelocityLimitv5);
    elbowMotor.configMotionCruiseVelocity(C.Superstructure.elbowVelocityLimitv5);

    //Set to zero if you want trapezoidal motion during motion magic. 
    shoulderMotor_starboard.configMotionSCurveStrength(C.Superstructure.shoulderSCurveStrengthv5);
    elbowMotor.configMotionSCurveStrength(C.Superstructure.elbowSCurveStrengthv5);


    /* Make the other motor a follow */
    shoulderMotor_port.follow(shoulderMotor_starboard);
    shoulderMotor_port.setInverted(true);

  }

  /**
   * Bandwith config motor CAN for TalonFX Motor
   * @param motor
   */
  private void bandWithLimitMotorCAN(TalonFX motor) {
    // Maybe this is needed for motion magic. 

    //Set relevant frame periods to be at least as fast as periodic rate
    motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,10); 
    motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,10);
    motor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,10);

    
    //Keep at default.
    //motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255); 
    //motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
    //motor.setStatusFramePeriod(StatusFrame.Status_1_General,40); 
    
    //slow it down
    motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
    motor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);
  }


  /**
   * Bandwith config motor CAN for TalonSRX Motor
   * @param motor
   */
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

  /**
   * Toggles break mode for the three superstucture motors
   * @param brakeMode
   */
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

  /**
   * Moves the shoulder at input speed
   * @param speed
   */
  public void moveShoulder(double speed) {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Moves the elbow at input speed
   * @param speed
   */
  public void moveElbow(double speed) {
    elbowMotor.set(ControlMode.PercentOutput, speed);
  }

  public void toggleIntakeIn(){ // pickup cone
    /* 
    if(clawNeo.get() == 0){
      clawNeo.set(C.Claw.clawNeoPercentPower);
    } else {
      clawNeo.set(0);
    } */

    if(IntakeMotor.getMotorOutputPercent()==0){
      IntakeMotor.set(ControlMode.PercentOutput, C.Superstructure.IntakeMotorTalonPercentPower);
    } else {
      IntakeMotor.set(ControlMode.PercentOutput, 0);
    }
  }
  
  public void IntakeCone(){
    IntakeMotor.set(ControlMode.PercentOutput, C.Superstructure.IntakeMotorTalonPercentPower);
  }

  public void SpitOutCone(){
    IntakeMotor.set(ControlMode.PercentOutput, -C.Superstructure.IntakeMotorTalonPercentPower);
  }

  public void setIntakeMotor(double speed){
    IntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotor(){
    IntakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void toggleIntakeOut(){
    if(IntakeMotor.getMotorOutputPercent()==0){
      IntakeMotor.set(ControlMode.PercentOutput, -C.Superstructure.IntakeMotorTalonPercentPower);
    } else {
      IntakeMotor.set(ControlMode.PercentOutput, 0);
    }
  }
  

  /**
   * Converts from angle to counts
   * @param angle
   * @param gearRatio
   * @return Falcon motor counts
   */
  public double angleToCounts(double angle, double gearRatio) {
    return (angle / 360.0 * C.Superstructure.countsPerRev) / gearRatio;
  }

  /**
   * Converts from counts to angle
   * @param counts
   * @param gearRatio
   * @return Angle of Falcon Motor
   */
  public double countsToAngle(double counts, double gearRatio) {
    /* test */
    return counts * gearRatio / C.Superstructure.countsPerRev * 360.0; // flipped from angletocounts
  }
  
  /**
   * Index 0 : Shoulder Angle, <p>
   * Index 1 : Elbow Angle, <p>
   * Sets the angle for the joints on the superstructure
   * @param angles
   */
  public void setJointAngles(double[] angles){
    shoulderMotor_starboard.set(TalonFXControlMode.Position, angleToCounts(angles[0] + offset_angle[0], C.Superstructure.shoulderGearRatio));
    elbowMotor.set(TalonFXControlMode.Position, angleToCounts(angles[1] + offset_angle[1], C.Superstructure.elbowGearRatio));
  }

  /**
   * Calculates if an angle is at the input angle
   * @param angle
   * @return if shoulder motor is at the angle specified
   */
  public boolean isShoulderAtPosition(double angle){
    return (Math.abs(countsToAngle(shoulderMotor_starboard.getSelectedSensorPosition(), C.Superstructure.shoulderGearRatio) - angle) <= Math.abs(C.Superstructure.shoulder_tolerance));
  }

  /**
   * Calculates if an angle is at the input angle
   * @param angle
   * @return if shoulder motor is at the angle specified
   */
  public boolean isElbowAtPosition(double angle){
    return (Math.abs(countsToAngle(elbowMotor.getSelectedSensorPosition(), C.Superstructure.shoulderGearRatio) - angle) <= Math.abs(C.Superstructure.elbow_tolerance));
  }

  /**
   * Calculates if the current Joint Position is at inputs pair of angles. 
   * @param angles
   * @return If the joints are at the angle
   */
  public boolean isAtPosition(double[] angles){
    SmartDashboard.putBoolean("shoulder in Position?",isShoulderAtPosition(angles[0]+offset_angle[0]));
    SmartDashboard.putBoolean("elbow in Position?", isElbowAtPosition(angles[1]+offset_angle[1]));
    return isShoulderAtPosition(angles[0]) & isElbowAtPosition(angles[1]);
  }

  /**
   * If the current draw on the motor exceed the threshold, then there is a game piece
   * It returns what game piece is in the intake based on toggle. Toggle will be updated by the driver. 
   * @return GamePiece.Cone or GamePiece.Cube
   */
  public GamePiece GetIntakeSensor(double thresh){
    if (IntakeMotor.getSupplyCurrent() < thresh){
      return GamePiece.Empty;
    }
    else{
      if (toggle == GamePiece.Cone){
        return GamePiece.Cone;
      }
      else{
        return GamePiece.Cube;
      }
    } 
  }

  public void testjoint(){
    double[] test  = {27,-16.5};
    // double[] test = {40,-186};
    setJointAngles(test);
  }

  public void undoTestJoin(){
    double[] test = {-5,0};
    setJointAngles(test);
  }

  public void togglePiece(){
    if (toggle == GamePiece.Cone){
      toggle = GamePiece.Cube;
      return;
    }
    toggle = GamePiece.Cone;
  }

  public void setPiece(GamePiece piece){
    toggle = (piece == GamePiece.Cone) ? GamePiece.Cone : GamePiece.Cube;
  }

  public void setCone(){
    GamePiece y = GamePiece.Cone;
    setPiece(y);
  }

  public void setCube(){
    GamePiece y = GamePiece.Cube;
    setPiece(y);
  }

  public boolean getElbowIndexSensor(){
    return !ElbowIndexSensor.get();
  }
  public boolean getShoulderIndexSensor(){
    return !ShoulderIndexSensor.get();
  }
/********************************************************************************************************************************************************* */
  @Override
  public void periodic() {

    switch (armState) {
      case Manual -> {
        if (coDriveCommand != States.Manual){
          armState = States.Home;
        }
      }
      case Home -> {

        //reset position when Elbow index is seen.
        if(getElbowIndexSensor()){
          //reset Elbow Motor Position to 0;
          elbowMotor.setSelectedSensorPosition(0);
          elbowMotor.setNeutralMode(NeutralMode.Brake);
          // resetElbowOffset();
        }

        //reset position when shoulder Index is seen.
        if(getShoulderIndexSensor()){
          //reset Shoulder Motor Position to 0;
          shoulderMotor_starboard.setSelectedSensorPosition(0);
          shoulderMotor_starboard.setNeutralMode(NeutralMode.Brake);
          shoulderMotor_port.setNeutralMode(NeutralMode.Brake);
          // resetShoulderOffset();
        }
        
        pickup = States.Home;

        setJointAngles(C.Superstructure.StateMachinePositions.Home);

        if (isAtPosition(C.Superstructure.StateMachinePositions.Home)) {
          
          
          //if codriver starts adjusting manually then go to codriver state
          if (offset_angle[0] != 0.0 | offset_angle[1] != 0.0){
            armState = States.coDriverMode;
            coDriveCommand = States.coDriverMode;

          } 
          
          // If codriver command has changed from home, lets go to the different states. 
          else if (coDriveCommand != States.Home){
              //armState = States.ExtendOut;
            if (coDriveCommand == States.Ground) {
              armState = States.ExtendOut;
              setTimeOut(10000);
              
          }
          else if (coDriveCommand == States.MidScoreCube){
            armState = States.ExtendOut;
          }
          else if (coDriveCommand == States.TopScoreCube){
            armState = States.ExtendOut;
          }
          else if (coDriveCommand == States.TopSCoreCone){
            armState = States.ExtendOut;
          }
          else if (coDriveCommand == States.MidScoreCone){
            armState = States.ExtendOut;
          }
        } 
      }
    }
      case coDriverMode ->{
        setJointAngles(C.Superstructure.StateMachinePositions.coDriverMode);

        if (coDriveCommand != States.coDriverMode){
          armState = States.ExtendIn;
        }
      }
      case ExtendOut -> { 
      
        setJointAngles(C.Superstructure.StateMachinePositions.ExtendOut);
  
        if (isAtPosition(C.Superstructure.StateMachinePositions.ExtendOut)) {
          armState = coDriveCommand; 
        } 
      }
      case ExtendIn -> {
  
        resetElbowOffset();
        resetShoulderOffset();
        setJointAngles(C.Superstructure.StateMachinePositions.ExtendIn);
        if (isAtPosition(C.Superstructure.StateMachinePositions.ExtendIn)) {
          armState = States.Home; 
          coDriveCommand = States.Home;
        }

      }
      case PreExtractIn -> {

        resetElbowOffset();
        resetShoulderOffset();
        setJointAngles(C.Superstructure.StateMachinePositions.PreExtractIn);
        if (isAtPosition(C.Superstructure.StateMachinePositions.PreExtractIn)) {
          armState = States.ExtendIn;

        }
      }
      case MidScoreCube -> {
        setJointAngles(C.Superstructure.StateMachinePositions.MidScoreCube);
      
      if (coDriveCommand != States.MidScoreCube) {
        armState = States.ExtendIn;
      }
    }
      case TopScoreCube -> {
        setJointAngles(C.Superstructure.StateMachinePositions.TopScoreCube);
        if (coDriveCommand != States.TopScoreCube) {
          armState = States.PreExtractIn;
        }
      }
      case TopSCoreCone -> {
        setJointAngles(C.Superstructure.StateMachinePositions.TopSCoreCone);
        if (coDriveCommand != States.TopSCoreCone) {
          armState = States.PreExtractIn; 
        }
      }
      case MidScoreCone -> {
        setJointAngles(C.Superstructure.StateMachinePositions.MidScoreCone);
        if (coDriveCommand != States.MidScoreCone) {
          armState = States.ExtendIn;
        }
      }
      
      case Ground -> {
        double thresh =0;
        
        thresh = IntakeMotor.getMotorOutputPercent() > 0 ? C.Superstructure.IntakeConeCThresh : C.Superstructure.IntakeCubeCThresh;

        setJointAngles(C.Superstructure.StateMachinePositions.Ground);
  
        if (GetIntakeSensor(thresh) != GamePiece.Empty) {
          //IntakeMotor.set(ControlMode.PercentOutput, 0);
          armState = coDriveCommand = States.ExtendIn;
        } else if (coDriveCommand != States.Ground) {
          armState = States.ExtendIn;
        }
      }
      
      
      default -> armState = States.Home;
    }

    // count global timer
    countDown();

    /* Get superstructure data */
    SmartDashboard.putNumber("Shouldmotor.DNG", shoulderMotor_starboard.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbowmotor.DNG", elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ShoulderMotorOutput", shoulderMotor_starboard.getMotorOutputPercent());
    SmartDashboard.putNumber("ElbowMotorOutput", elbowMotor.getMotorOutputPercent());
    SmartDashboard.putString("SuperStructure State", armState.name());
    SmartDashboard.putNumber("shoulder Postion", countsToAngle(shoulderMotor_starboard.getSelectedSensorPosition(), C.Superstructure.shoulderGearRatio));
    SmartDashboard.putNumber("elbow Postion", countsToAngle(elbowMotor.getSelectedSensorPosition(), C.Superstructure.elbowGearRatio));

    SmartDashboard.putString("CodriverCommand", coDriveCommand.toString());
    SmartDashboard.putNumber("Shoulder Velocity counts per 100ms", shoulderMotor_starboard.getSelectedSensorVelocity());
    SmartDashboard.putNumber("SHoulder following Veloctiy", shoulderMotor_port.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Elbow Velocity counts per 100ms", elbowMotor.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("Sh_Index", getShoulderIndexSensor());
    SmartDashboard.putBoolean("Elbow Index", getElbowIndexSensor());
  }
/*************************************************************************************************************************************************************************** */
  /** return current state */
  public States getCurrentState(){
    return armState;
  }

  /** Sets the statemachine global timeout */
  public void setTimeOut(double ms){
    TimerCounter = (int) ms/20; 
  }
  /** Countdown for the timer */
  public void countDown(){
    if (TimerCounter != 0){
      TimerCounter -= 1;
      expired = false;
    }
    else{
      expired = true;
    }
  }
  /** Checks to see if timer is timed out */
  public boolean isTimedOut(){
    return expired;
  }
  /** method for button to use as instant command to set the state to home  */
  public void buttonHome() {
    coDriveCommand = States.ExtendIn;
  }

  /** method for button to use as instant command to set the state to */
  public void buttonGround() {
    coDriveCommand = States.Ground;
 ;
  }
  public void buttonMidScoreCube(){
    coDriveCommand = States.MidScoreCube;
  }
  public void buttonTopScoreCube() {
    coDriveCommand = States.TopScoreCube;
  }
  public void buttonTopScoreCone(){
    coDriveCommand = States.TopSCoreCone;
  }
  public void buttonMidScoreCone(){
    coDriveCommand = States.MidScoreCone;
  }
/* funky manual code */
  public void moveShoulderSlowUp() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, C.Superstructure.testMove);
    armState = States.Manual;
  }

  public void moveShoulderSlowDown() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, -C.Superstructure.testMove);
    armState = States.Manual;
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

  /**
   * Stops all the joints
   */
  public void stopJoints(){
    shoulderMotor_starboard.set(TalonFXControlMode.PercentOutput, 0);
    elbowMotor.set(TalonFXControlMode.PercentOutput, 0);
    armState = States.Manual;
  }
/*  */

/***************************************************************************
   * *************************************************************************
   * *************************************************************************
   */
  private double offset_angle[] = { 0, 0 };
  /**
   * increments the elbow offset by the increment number
   * 
   * @param increment
   */
  public void incrementElbowOffset(double increment) {
    offset_angle[1] += increment;
  }
  /**
   * increments the elbow offset by the increment number
   * 
   * @param increment
   */
  public void incrementShoulderOffset(double increment) {
    offset_angle[0] += increment;
  }
  /**
   * resets offset at shoulder
   */
  public void resetShoulderOffset() {
    offset_angle[0] = 0.0;
  }
  /**
   * resets offset at shoulder
   */
  public void resetElbowOffset() {
    offset_angle[1] = 0.0;
  }
  /**
   * Uses joysticks. If driver passes threshold it will increment and decrment
   * elbow and shoulder offsets.
   * 
   * @param joystick_left
   * @param joystick_right
   */
  public void updatejointoffsets(double shoulder_joystick, double elbow_joystick) {
    // If threshould joystick is exceeded update offset
    if (Math.abs(shoulder_joystick) > 0.5) {
      // if positive, increment offset
      if (shoulder_joystick > 0) {
        incrementShoulderOffset(C.Superstructure.offset_incrment_constant);
      }
      // if negative, decriment offset
      else {
        incrementShoulderOffset(-C.Superstructure.offset_incrment_constant);
      }
    }
    // If threshould joystick is exceeded update offset
    if (Math.abs(elbow_joystick) > 0.5) {
      // if positive, increment offset
      if (elbow_joystick > 0) {
        incrementElbowOffset(C.Superstructure.offset_incrment_constant);
      }
      // if negative, decriment offset
      else {
        incrementElbowOffset(-C.Superstructure.offset_incrment_constant);
      }
    }
  }
  /***************************************************************************
   * *************************************************************************
   * *************************************************************************
   */


}


/*
 *  Pick from pickstation
 *  Pick up from Ground
 *  scrore low and mid
 *  storage inside robot
 */