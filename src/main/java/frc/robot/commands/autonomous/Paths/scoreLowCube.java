// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.coneOutTake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreLowCube extends SequentialCommandGroup {
  /** Creates a new driveoff. */
  public scoreLowCube(Drivetrain p_Drivetrain, Superstructure p_Superstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new coneOutTake(p_Superstructure).withTimeout(3)
      // new AutoDriveToDistance(p_Drivetrain, -10)
    );
  }
}
