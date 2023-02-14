// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.C;
import frc.robot.subsystems.Drivetrain;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */
    private Drivetrain m_subsystem ;
    private double m_intial_angle = 0;
    private double m_sub_goal_angle = 0;
    private double m_goal_angle = 0;
    public AutoTurn(Drivetrain subsytem, double goal_angle) {
      m_subsystem = subsytem; 
      m_sub_goal_angle = goal_angle % 360;
    
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_sub_goal_angle <= 180){
      if(m_sub_goal_angle <= -180){
        m_goal_angle = m_goal_angle - 360;
      }
      else{
        m_goal_angle = m_sub_goal_angle; // to the right, to the right, to the right
      }
    }
    else{
      m_goal_angle = m_sub_goal_angle - 360;  // to the left, to the left, to the left
    }
    m_subsystem.turnpid.reset(); // reset any pid variables from last time used
    m_intial_angle = m_subsystem.getYaw();  // get the current starting angle
    m_subsystem.turnpid.setP(C.Drive.turn_kp);
    m_subsystem.turnpid.setI(C.Drive.turn_ki);
    m_subsystem.turnpid.setD(C.Drive.turn_kd);
    m_subsystem.resetIMU();
    m_subsystem.turnpid.setSetpoint(m_intial_angle+m_goal_angle); // set the goal of the angle
    SmartDashboard.putNumber("goal angle", m_intial_angle+m_goal_angle);
    m_subsystem.turnpid.setTolerance(C.Drive.turn_tolerance); // set the goal tolerance to know when you're finished
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double c_angle = m_subsystem.getYaw();  // Get the current sample, changed becuase the robot has turned
    double output_PID = m_subsystem.turnpid.calculate(c_angle); // computes the motor power
    SmartDashboard.putNumber("pid output", output_PID);
    SmartDashboard.putNumber("error", m_subsystem.turnpid.getPositionError()); 
      m_subsystem.setLeftRight(-output_PID, output_PID); // to the right, to the right, to the right

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setLeftRight(0, 0); // Stops all motors just in case
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.turnpid.atSetpoint(); // Are we there yet???? if so end, or else run execute again!
   // return false;
  }
}
