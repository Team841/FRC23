// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.PIDControllers;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double SensorData ; 
    m_subsystem.setDrivetrainBrakeMode(true);
    SensorData = m_subsystem.GetRobotAngle() ;
    double kp = 0.0110; 
    double ref = 0 ; 
    double error = ref - SensorData ; 
    m_subsystem.setLeftRight (error*kp,error*kp) ; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
