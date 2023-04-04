// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import frc.lib.acutators.BioNeo;
import frc.lib.acutators.BioNeoConfigs;
import frc.robot.C;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


/*
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance; */

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.DriveStyle;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  BioNeoConfigs config = new BioNeoConfigs();

  private final BioNeo left1 = new BioNeo(C.CANid.driveLeft1, C.Drive.currentLimit);
  private final BioNeo left2 = new BioNeo(C.CANid.driveLeft2, C.Drive.currentLimit);
  private final BioNeo right1 = new BioNeo(C.CANid.driveRight1, C.Drive.currentLimit);
  private final BioNeo right2 = new BioNeo(C.CANid.driveRight2, C.Drive.currentLimit);

  public PIDController balance_pid = new PIDController(0, 0, 0);

  public RelativeEncoder leftEncoder = left1.getEncoder();
  public RelativeEncoder rightEncoder = right1.getEncoder();

  public SparkMaxPIDController PIDController_left;
  public SparkMaxPIDController PIDController_right;

  private double PIDdistance = 0;
  private boolean isDistancePIDenabled = false;

  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  Solenoid brake = new Solenoid(PneumaticsModuleType.REVPH, C.Drive.Brake);
  Solenoid brake_b = new Solenoid(PneumaticsModuleType.REVPH, C.Drive.Brake_b);

  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  public DriveStyle drivestyle = new DriveStyle();
  public Drivetrain() {

    left2.follow(left1);
    right2.follow(left1);

    PIDController_left = left1.getPIDController();
    PIDController_right = right1.getPIDController();

    // turnPID
    config.SLOT0(PIDController_left, C.Drive.turnGains);
    config.SLOT0(PIDController_right, C.Drive.turnGains);

    // distancePID
    config.SLOT1(PIDController_left, C.Drive.distanceGains);
    config.SLOT1(PIDController_right, C.Drive.distanceGains);

    leftEncoder = left1.getEncoder();
    rightEncoder = right1.getEncoder();
    phCompressor.enableAnalog(100, 115);


    
    brake.set(false);
    brake_b.set(true);
  }

  public void Drive(double wheel, double throttle) {
    setDrivetrainBrakeMode(false);
    drivestyle.cheesyDrive(wheel, throttle );
    //REV syntax
    left1.set(drivestyle.getLeftPower());
    
    right1.set(drivestyle.getRightPower());
    
  }

  public void setQuickTurn(){
      drivestyle.setQuickTurn();
  }

  public void resetQuickTurn(){
      drivestyle.resetQuickTurn();
  }
  
  public double GetRobotAngle(){
    return imu.getXComplementaryAngle(); 
  }

  public void setLeftRight(double _Leftpower, double _Rightpower){
    //REV syntax
    left1.set(-_Leftpower);
    right1.set(_Rightpower);

  } 

  public void setDrivetrainBrakeMode(boolean _BrakeMode){
    if (_BrakeMode){
      left1.setIdleMode(IdleMode.kBrake);
      left2.setIdleMode(IdleMode.kBrake);
      right1.setIdleMode(IdleMode.kBrake);
      right2.setIdleMode(IdleMode.kBrake);
    }
    else{
      left1.setIdleMode(IdleMode.kCoast);
      left2.setIdleMode(IdleMode.kCoast);
      right1.setIdleMode(IdleMode.kCoast);
      right2.setIdleMode(IdleMode.kCoast);
    }
  }

  public boolean isBrakeMode(){
    if (left1.getIdleMode() == IdleMode.kBrake){
      return true;
    }
    else{
      return false;
    }
  }

  public void resetIMU() {
    imu.reset();
  }

  public void toggleBrakes(){
    if (brake.get()) {
      brake.set(false);
      brake_b.set(true);
    } else{
      brake.set(true);
      brake_b.set(false);
    }
  }

  public void BrakeOn(){
    brake.set(true);
    brake_b.set(false);
  }

  public void BrakeOff(){
    brake.set(false);
    brake_b.set(true);
  }

  @Override
  public void periodic() {
    /*
    SmartDashboard.putNumber("ACCL_x", imu.getAccelX());///use this axis
    SmartDashboard.putNumber("ACCL_y", imu.getAccelY());
    SmartDashboard.putNumber("ACCL_z", imu.getAccelZ());/// use this axis
    SmartDashboard.putNumber("Robot Angle", GetRobotAngle());
    SmartDashboard.putNumber("Robot Yaw", getYaw());
    SmartDashboard.putNumber("Left Distance", right_encoder.getPosition() * C.Drive.gearRatio * (C.Drive.wheelDiameter * Math.PI));
    SmartDashboard.putNumber("Right Distance", left_encoder.getPosition() * C.Drive.gearRatio * (C.Drive.wheelDiameter * Math.PI));

    SmartDashboard.putNumber("PID Distance", PIDdistance);
    SmartDashboard.putNumber("output", left1.getAppliedOutput());

    SmartDashboard.putBoolean("brake mode", isBrakeMode());

    SmartDashboard.putNumber("left1", drivestyle.getLeftPower());
    SmartDashboard.putNumber("right1", drivestyle.getRightPower()); */
    SmartDashboard.putBoolean("brake", brake.get());


    if (isDistancePIDenabled){
      // reset the pid distances
      PIDController_left.setReference(PIDdistance, CANSparkMax.ControlType.kPosition);
      PIDController_right.setReference(-PIDdistance, CANSparkMax.ControlType.kPosition);

    }
  }

  public void setDistancePID(double _distance){
    PIDdistance = _distance/C.Drive.gearRatio/(C.Drive.wheelDiameter*Math.PI);
  }

  public void enableDistancePID(boolean _enable){
    isDistancePIDenabled = _enable;
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0); rightEncoder.setPosition(0);
  }

  public double getEncoder(){
    return Math.abs(leftEncoder.getPosition());
  }

  public double getPIDdistanceError(){
    return Math.abs(PIDdistance) - Math.abs(right_encoder.getPosition());
  }

  public double getYaw() {
    return -imu.getAngle();
  }


}

