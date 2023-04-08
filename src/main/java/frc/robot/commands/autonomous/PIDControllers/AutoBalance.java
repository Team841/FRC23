// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.PIDControllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.C;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
  
  /** Creates a new AutoBalance. */
  private Drivetrain m_subsystem ;
  public AutoBalance(Drivetrain subsytem) {
    m_subsystem = subsytem ; 
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.balance_pid.reset();
    m_subsystem.balance_pid.setP(C.Drive.balance_kp);
    m_subsystem.balance_pid.setI(C.Drive.balance_ki);
    m_subsystem.balance_pid.setD(C.Drive.balance_kd);
    m_subsystem.balance_pid.setSetpoint(C.Drive.balance_point);
    m_subsystem.setDrivetrainBrakeMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_subsystem.balance_pid.calculate (m_subsystem.GetRobotAngle());
    output = MathUtil.clamp(output, -0.2,  0.2);
    if (Math.abs(m_subsystem.GetRobotAngle()) > 7.0){
      output = MathUtil.clamp(output, -0.2,  0.2);
    } 
    else
    {
      output = 0; 
    }
    
    m_subsystem.setLeftRight(output, output);

    if (Math.abs(m_subsystem.GetRobotAngle()) < 1.5){
      m_subsystem.BrakeOn();
    }
    else {
      m_subsystem.BrakeOff();
    }
  } 

    
/*
 Make the robot recognize that when it's angle reaches/ starts to move,  decrease its speed and wait.
 */
    /*double SensorData ; 
    m_subsystem.setDrivetrainBrakeMode(true);
    SensorData = m_subsystem.GetRobotAngle() ;
    double kp = 0.0110; 
    double ref = 0 ; 
    double error = ref - SensorData ; 
    m_subsystem.setLeftRight (error*kp,error*kp) ; */
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.BrakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
