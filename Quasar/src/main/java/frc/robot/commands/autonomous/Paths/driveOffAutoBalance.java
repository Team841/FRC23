// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.Paths;

import frc.robot.commands.autonomous.PIDControllers.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class driveOffAutoBalance extends SequentialCommandGroup {
  /** Creates a new driveOffAutoBalance. */

  public driveOffAutoBalance(Drivetrain a_Drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveToDistance(a_Drivetrain, 60), 
      new AutoTurn(a_Drivetrain, 270), 
      new AutoDriveToDistance(a_Drivetrain, 48),
      new AutoTurn(a_Drivetrain, 270),
      new AutoDriveToDistance(a_Drivetrain, 72),
      new AutoBalance(a_Drivetrain)
    );
  }
}
