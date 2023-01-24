// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


import frc.robot.C;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;

import frc.robot.DriveStyle;
public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final TalonFX left1 = new TalonFX(C.CANid.driveLeft1);
  private final TalonFX left2 = new TalonFX(C.CANid.driveLeft2);
  private final TalonFX right1 = new TalonFX(C.CANid.driveRight1);
  private final TalonFX right2 = new TalonFX(C.CANid.driveRight2);
  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  public DriveStyle drivestyle = new DriveStyle();
  public Drivetrain() {
    left1.configFactoryDefault();
      
      right1.configFactoryDefault();
 
      left2.configFactoryDefault();

      right2.configFactoryDefault();

      left1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
      left2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
      right1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
      right2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

      left1.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,255);
      left1.setStatusFramePeriod(StatusFrame.Status_1_General,40);
      left1.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      left1.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255); 
      left1.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,255);
      left1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
      left1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,255);
      left1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
      left1.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);

      left2.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,255);
      left2.setStatusFramePeriod(StatusFrame.Status_1_General,40);
      left2.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      left2.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255); 
      left2.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,255);
      left2.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
      left2.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,255);
      left2.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
      left2.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);

      right1.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,255);
      right1.setStatusFramePeriod(StatusFrame.Status_1_General,40);
      right1.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      right1.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255); 
      right1.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,255);
      right1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
      right1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,255);
      right1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
      right1.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);

      right2.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,255);
      right2.setStatusFramePeriod(StatusFrame.Status_1_General,40);
      right2.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      right2.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255); 
      right2.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,255);
      right2.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
      right2.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,255);
      right2.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
      right2.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);
  
      
      //Set #2 controllers to follow #1 in both drives
      left2.follow(left1);
      right2.follow(right1);
  }
  public void Drive(Joystick stickLeft, Joystick stickRight) {
    drivestyle.cheesyDrive(stickLeft);
    left1.set(ControlMode.PercentOutput,drivestyle.getLeftPower());
    right1.set(ControlMode.PercentOutput,drivestyle.getRightPower());
  }
    public void setQuickTurn(){
      drivestyle.setQuickTurn();
    }
    public void resetQuickTurn(){
      drivestyle.resetQuickTurn();
    }
  
  public double GetRobotAngle(){
    double xAccel = imu.getAccelX()/9.81;
    double zAccel = imu.getAccelZ()/9.81;

    double angleOne = Math.asin(zAccel);
    double angleTwo = Math.acos(xAccel);

    return ((angleOne+angleTwo)/2)*180/3.14159;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ACCL_x", imu.getAccelX());///use this axis
    SmartDashboard.putNumber("ACCL_y", imu.getAccelY());
    SmartDashboard.putNumber("ACCL_z", imu.getAccelZ());/// use this axis
    SmartDashboard.putNumber("Robot level", GetRobotAngle()-90);
    // SmartDashboard.putNumber("TEEHEE", imu.getAccelX());
    // This method will be called once per scheduler run

    //test
  }
}
