// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.C;
import frc.robot.subsystems.Superstructure;

public class SetJointsToHome extends CommandBase {
  private Superstructure c_Superstructure;
  /** Creates a new SetJointsToHighScoreCone. */
  public SetJointsToHome(Superstructure superstructure) {
    c_Superstructure = superstructure;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(superstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  c_Superstructure.buttonHome();
    //c_Superstructure.buttonTopScoreCone();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return c_Superstructure.isAtPosition(C.Superstructure.StateMachinePositions.Home);
    //return c_Superstructure.isAtPosition(C.Superstructure.StateMachinePositions.TopSCoreCone);
  }
}
