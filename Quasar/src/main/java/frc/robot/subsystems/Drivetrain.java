// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


import frc.robot.C;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  
  //REV SparkMax
  private final CANSparkMax left1 = new CANSparkMax(C.CANid.driveLeft1, MotorType.kBrushless);
  private final CANSparkMax left2 = new CANSparkMax(C.CANid.driveLeft2, MotorType.kBrushless);
  private final CANSparkMax right1 = new CANSparkMax(C.CANid.driveRight1, MotorType.kBrushless);
  private final CANSparkMax right2 = new CANSparkMax(C.CANid.driveRight2, MotorType.kBrushless);

  public PIDController turnpid = new PIDController(C.Drive.turn_kd, C.Drive.turn_ki, C.Drive.turn_kp);
  public SparkMaxPIDController left_distance_pid = left1.getPIDController();
  public SparkMaxPIDController right_distance_pid = right1.getPIDController();

  public RelativeEncoder left_encoder = left1.getEncoder();
  public RelativeEncoder right_encoder = right1.getEncoder(); 
  private double PIDdistance = 0;
  private boolean isDistancePIDenabled = false;

  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  public DriveStyle drivestyle = new DriveStyle();
  public Drivetrain() {

    //REV Syntax
    left1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();

    //REV syntax
    //TODO: turn current limit into a Constant
    left1.setSmartCurrentLimit(60); //Current limit at number of amps 
    left2.setSmartCurrentLimit(60);
    right1.setSmartCurrentLimit(60);
    right2.setSmartCurrentLimit(60);

    //Set #2 controllers to follow #1 in both drives
    //Syntax is shared for REV/CTRE
    left2.follow(left1);
    right2.follow(right1);

    left_distance_pid.setP(C.Drive.distance_kp);
    left_distance_pid.setI(C.Drive.distance_ki);
    left_distance_pid.setD(C.Drive.distance_kd);
    left_distance_pid.setFF(C.Drive.distance_kff);

    right_distance_pid.setP(C.Drive.distance_kp);
    right_distance_pid.setI(C.Drive.distance_ki);
    right_distance_pid.setD(C.Drive.distance_kd);
    right_distance_pid.setFF(C.Drive.distance_kff);

    left_distance_pid.setIZone(C.Drive.distance_kIz / (C.Drive.gearRatio * (C.Drive.wheelDiameter * Math.PI)));
    right_distance_pid.setIZone(C.Drive.distance_kIz / (C.Drive.gearRatio * (C.Drive.wheelDiameter * Math.PI)));

    left_distance_pid.setOutputRange(-1,1);

  }

  public void Drive(Joystick stickLeft, Joystick stickRight) {
    setDrivetrainBrakeMode(false);
    drivestyle.cheesyDrive(stickLeft);
    //REV syntax
    left1.set(drivestyle.getLeftPower());
    SmartDashboard.putNumber("left1", drivestyle.getLeftPower());
    right1.set(drivestyle.getRightPower());
    SmartDashboard.putNumber("right1", drivestyle.getRightPower());
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

  @Override
  public void periodic() {
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

    if (isDistancePIDenabled){
      // reset the pid distances
      left_distance_pid.setReference(PIDdistance, CANSparkMax.ControlType.kPosition);
      right_distance_pid.setReference(-PIDdistance, CANSparkMax.ControlType.kPosition);

    }
  }

  public void setDistancePID(double _distance){
    PIDdistance = _distance/C.Drive.gearRatio/(C.Drive.wheelDiameter*Math.PI);
  }

  public void enableDistancePID(boolean _enable){
    isDistancePIDenabled = _enable;
  }

  public void resetEncoders(){
    left_encoder.setPosition(0); right_encoder.setPosition(0);
  }

  public double getPIDdistanceError(){
    return Math.abs(PIDdistance) - Math.abs(right_encoder.getPosition());
  }

  public double getYaw() {
    return -imu.getAngle();
  }
}
