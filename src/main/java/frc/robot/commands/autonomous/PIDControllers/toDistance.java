// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.PIDControllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.C;
import frc.robot.subsystems.Drivetrain;

public class toDistance extends CommandBase {

  private Conversions _conversions = new Conversions();

  private Drivetrain subsystem;
  private SparkMaxPIDController left;
  private SparkMaxPIDController right;
  private double goal;

  public toDistance(Drivetrain drivetrain, double _goal, SparkMaxPIDController _left, SparkMaxPIDController _right) {
    this.left = _left;
    this.right = _right;
    this.subsystem = drivetrain;
    this.goal = _goal;
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetEncoders();
    left.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve, 1);
    right.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    left.setReference(_conversions.inchesToRotationsDrive(goal, C.Drive.gearRatio, C.Drive.wheelDiameter), CANSparkMax.ControlType.kSmartMotion, 1);
    right.setReference(_conversions.inchesToRotationsDrive(goal, C.Drive.gearRatio, C.Drive.wheelDiameter), CANSparkMax.ControlType.kSmartMotion, 1);
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
    return Math.abs(position - goal) <= C.Drive.distanceGains.getTolerance();
  }
}
