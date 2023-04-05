// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.PIDControllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.C;
import frc.robot.subsystems.Drivetrain;

public class toDistance extends CommandBase {

  private Conversions _conversions = new Conversions();

  private Drivetrain subsystem;
  private double goal;

  public toDistance(Drivetrain drivetrain, double _goal) {
    this.subsystem = drivetrain;
    this.goal = _goal;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetEncoders();
    SmartDashboard.getNumber("goal conversion", _conversions.inchesToRotationsDrive(goal, C.Drive.gearRatio, C.Drive.wheelDiameter));
    subsystem.PIDController_left.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve, 1);
    subsystem.PIDController_right.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.                                                                             
  @Override
  public void execute() {
    subsystem.PIDController_left.setReference(-_conversions.inchesToRotationsDrive(goal, C.Drive.gearRatio, C.Drive.wheelDiameter), CANSparkMax.ControlType.kSmartMotion, 1);
    subsystem.PIDController_right.setReference(_conversions.inchesToRotationsDrive(goal, C.Drive.gearRatio, C.Drive.wheelDiameter), CANSparkMax.ControlType.kSmartMotion, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setLeftRight(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double position = subsystem.getEncoder();
    SmartDashboard.putNumber("toDistance position     ", position);
    return Math.abs(position - goal) <= C.Drive.distanceGains.getTolerance();
  }
}
