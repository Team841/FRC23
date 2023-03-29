// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.actuatuors.BioFalcon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.C;

import com.ctre.phoenix.motorcontrol.can.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

/* import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance; */
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure.  */

  private final TalonSRX IntakeMotor = new TalonSRX(C.CANid.IntakeTalon);

  private final BioFalcon shoulderMotor_starboard = new BioFalcon(C.CANid.shoulderMotor_Starboard, C.Superstructure.Shoulder.Shoulder_starboard_config);
  private final BioFalcon shoulderMotor_port = new BioFalcon(C.CANid.shoulderMotor_Port, C.Superstructure.Shoulder.Shoulder_port_config);

  DigitalInput ElbowIndexSensor = new DigitalInput(C.Superstructure.Elbow.SensorIndex);
  DigitalInput ShoulderIndexSensor = new DigitalInput(C.Superstructure.Shoulder.sensorIndex);
  
  private int TimerCounter = 0; 
  private boolean expired = true; 

  enum States{
    Home,
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

  public States coDriveCommand = States.Manual;
  private States pickup = States.Home;
  private States armState = States.Manual;
  private GamePiece intakeState = GamePiece.Empty;
  private GamePiece toggle = GamePiece.Cone;

  public Superstructure() {
    shoulderMotor_port.
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
  public int moveElbow(int speed) {
    elbowMotor.set(ControlMode.PercentOutput, speed);
    return ~speed & 1;
  }

  public void toggleIntakeIn(){ // pickup cone
    /* 
    if(clawNeo.get() == 0){
      clawNeo.set(C.Claw.clawNeoPercentPower);
    } else {
      clawNeo.set(0);
    } */

    if(IntakeMotor.getMotorOutputPercent()==0){
      IntakeMotor.set(ControlMode.PercentOutput, C.Superstructure.Intake.TalonPercentPower + 0.2);
    } else {
      IntakeMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setIntakeMotor(double speed){
    IntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void toggleIntakeOut(){
    if(IntakeMotor.getMotorOutputPercent()==0){
      IntakeMotor.set(ControlMode.PercentOutput, -C.Superstructure.Intake.TalonPercentPower);
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
    shoulderMotor_starboard.set(TalonFXControlMode.Position, angleToCounts(angles[0], C.Superstructure.Shoulder.gearRatio));
    elbowMotor.set(TalonFXControlMode.Position, angleToCounts(angles[1], C.Superstructure.Elbow.gearRatio));
  }

  /**
   * Calculates if an angle is at the input angle
   * @param angle
   * @return if shoulder motor is at the angle specified
   */
  public boolean isShoulderAtPosition(double angle){
    return (Math.abs(countsToAngle(shoulderMotor_starboard.getSelectedSensorPosition(), C.Superstructure.Shoulder.gearRatio) - angle) <= C.Superstructure.Shoulder.tolerance);
  }

  /**
   * Calculates if an angle is at the input angle
   * @param angle
   * @return if shoulder motor is at the angle specified
   */
  public boolean isElbowAtPosition(double angle){
    return (Math.abs(countsToAngle(elbowMotor.getSelectedSensorPosition(), C.Superstructure.Elbow.gearRatio) - angle) <= C.Superstructure.Elbow.tolerance);
  }

  /**
   * Calculates if the current Joint Position is at inputs pair of angles. 
   * @param angles
   * @return If the joints are at the angle
   */
  public boolean isAtPosition(double[] angles){
    return isShoulderAtPosition(angles[0]) && isShoulderAtPosition(angles[1]);
  }

  /**
   * If the current draw on the motor exceed the threshold, then there is a game piece
   * It returns what game piece is in the intake based on toggle. Toggle will be updated by the driver. 
   * @return GamePiece.Cone or GamePiece.Cube
   */
  public GamePiece GetIntakeSensor(){
    if (IntakeMotor.getSupplyCurrent() < C.Superstructure.Intake.TalonPercentPower){
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

  public void setPiece(){
    if (toggle == GamePiece.Cone){
      toggle = GamePiece.Cube;
      return;
    }
    toggle = GamePiece.Cone;
  }

  public boolean getElbowIndexSensor(){
    return !ElbowIndexSensor.get();
  }
  public boolean getShoulderIndexSensor(){
    return !ShoulderIndexSensor.get();
  }

  @Override
  public void periodic() {
    if(getElbowIndexSensor()){
      //reset Elbow Motor Position to 0;
      shoulderMotor_starboard.setSelectedSensorPosition(0);
    }
    if(getShoulderIndexSensor()){
      //reset Shoulder Motor Position to 0;
      elbowMotor.setSelectedSensorPosition(0);
    }
    switch (armState) {
      case Manual -> {
        if (coDriveCommand != States.Manual){
          armState = States.Home;
        }
      }
      case Home -> {
        pickup = States.Home;
          /*
          if (coDriveCommand == armState) {
            setJointAngles(C.Superstructure.StateMachinePositions.Home);
          } */

        setJointAngles(C.Superstructure.StateMachinePositions.Home);

        if (isAtPosition(C.Superstructure.StateMachinePositions.Home)) {
          if (coDriveCommand == States.LowScore) {
            armState = States.LowScore;
            setTimeOut(5000);
          } else if (coDriveCommand == States.MidScore) {
            armState = States.MidScore;
            setTimeOut(5000);
          } else if (coDriveCommand == States.HighScore) {
            armState = States.HighScore;
            setTimeOut(5000);
          } else if (coDriveCommand == States.Ground) {
            armState = States.Ground;
          } else if (coDriveCommand == States.LowPortal) {
            armState = States.LowPortal;
          } else if (coDriveCommand == States.HighPortal) {
            armState = States.HighPortal;
          }
        }
      }
      case LowScore -> {
        setJointAngles(C.Superstructure.StateMachinePositions.LowScore);
        if (GetIntakeSensor() != GamePiece.Empty) {
          armState = coDriveCommand = States.Home;
        } else if (coDriveCommand != States.LowScore && isAtPosition(C.Superstructure.StateMachinePositions.LowScore)) {
          armState = States.Home;
        } else if (isTimedOut()) {
          armState = coDriveCommand = States.Home;
        }
      }
      case MidScore -> {
        setJointAngles(C.Superstructure.StateMachinePositions.MidScore);
        if (GetIntakeSensor() != GamePiece.Empty) {
          armState = coDriveCommand = States.Home;
        } else if (coDriveCommand != States.MidScore && isAtPosition(C.Superstructure.StateMachinePositions.MidScore)) {
          armState = States.Home;
        } else if (isTimedOut()) {
          armState = coDriveCommand = States.Home;
        }
      }
      case HighScore -> {
        setJointAngles(C.Superstructure.StateMachinePositions.HighScore);
        if (GetIntakeSensor() != GamePiece.Empty) {
          armState = coDriveCommand = States.Home;
        } else if (coDriveCommand != States.HighScore && isAtPosition(C.Superstructure.StateMachinePositions.HighScore)) {
          armState = States.Home;
        } else if (isTimedOut()) {
          armState = coDriveCommand = States.Home;
        }
      }
      case Ground -> {
        setJointAngles(C.Superstructure.StateMachinePositions.Ground);
        if (isAtPosition(C.Superstructure.StateMachinePositions.Ground)) {
          armState = States.Pickup;
          pickup = States.Ground;
          setTimeOut(10000);
        }
      }
      case HighPortal -> {
        setJointAngles(C.Superstructure.StateMachinePositions.HighPortal);
        if (isAtPosition(C.Superstructure.StateMachinePositions.HighPortal)) {
          armState = States.Pickup;
          pickup = States.HighPortal;
          setTimeOut(8000);
        }
      }
      case LowPortal -> {
        setJointAngles(C.Superstructure.StateMachinePositions.LowPortal);
        if (isAtPosition(C.Superstructure.StateMachinePositions.LowPortal)) {
          armState = States.Pickup;
          pickup = States.LowPortal;
          setTimeOut(8000);
        }
      }
      case Pickup -> {
        if (toggle == GamePiece.Cone) {
          IntakeMotor.set(ControlMode.PercentOutput, C.Superstructure.Intake.TalonPercentPower);
        } else if (toggle == GamePiece.Cube) {
          IntakeMotor.set(ControlMode.PercentOutput, -C.Superstructure.Intake.TalonPercentPower);
        }
        if (GetIntakeSensor() != GamePiece.Empty) {
          IntakeMotor.set(ControlMode.PercentOutput, 0);
          armState = coDriveCommand = States.Home;
        } else if (coDriveCommand != pickup) {
          armState = States.Home;
        } else if (isTimedOut()) {
          IntakeMotor.set(ControlMode.PercentOutput, 0);
          armState = coDriveCommand = States.Home;
        }
      }
      default -> armState = States.Home;
    }

    countDown();

    SmartDashboard.putNumber("Shouldmotor.DNG", shoulderMotor_starboard.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbowmotor.DNG", elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ShoulderMotorOutput", shoulderMotor_starboard.getMotorOutputPercent());
    SmartDashboard.putNumber("ElbowMotorOutput", elbowMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("shoulder Postion", countsToAngle(shoulderMotor_starboard.getSelectedSensorPosition(), C.Superstructure.Shoulder.gearRatio));
    SmartDashboard.putNumber("elbow Postion", countsToAngle(elbowMotor.getSelectedSensorPosition(), C.Superstructure.Elbow.gearRatio));
  }

  public void setTimeOut(double ms){
    TimerCounter = (int) ms/20; 
  }
  public void countDown(){
    if (TimerCounter != 0){
      TimerCounter -= 1;
      expired = false;
    }
    else{
      expired = true;
    }
  }

  public boolean isTimedOut(){
    return expired;
  }


/* funky manual code */
  public void moveShoulderSlowUp() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, 0.1);
    armState = States.Manual;
  }

  public void moveShoulderSlowDown() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, -0.1);
    armState = States.Manual;
  }

  public void stopShoulder() {
    shoulderMotor_starboard.set(ControlMode.PercentOutput, 0);
  }
    
  public void moveElbowSlowUp() {
    elbowMotor.set(ControlMode.PercentOutput, 0.1);
  }

  public void moveElbowSlowDown() {
    elbowMotor.set(ControlMode.PercentOutput, -0.1);
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

}


/*
 *  Pick from pickstation
 *  Pick up from Ground
 *  scrore low and mid
 *  storage inside robot
 */