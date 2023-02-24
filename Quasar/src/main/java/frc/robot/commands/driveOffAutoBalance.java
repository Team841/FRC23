// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveToDistance;
import frc.robot.commands.AutoTurn;
import frc.robot.subsystems.Drivetrain;


public class driveOffAutoBalance extends SequentialCommandGroup {
  /** Creates a new driveOffAutoBalance. */

  private Drivetrain m_subsystem;
  public driveOffAutoBalance() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveToDistance(m_subsystem, 48), 
      new AutoTurn(m_subsystem, 270), 
      new AutoDriveToDistance(m_subsystem, 60),
      new AutoTurn(m_subsystem, 270),
      new AutoDriveToDistance(m_subsystem, 60),
      new AutoBalance(m_subsystem)
    );
  }
}
