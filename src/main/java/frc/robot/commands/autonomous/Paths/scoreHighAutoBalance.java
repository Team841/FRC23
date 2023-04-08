// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.Paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.C.Drive;
import frc.robot.C.*;
import frc.robot.commands.autonomous.PIDControllers.AutoBalance;
import frc.robot.commands.autonomous.PIDControllers.AutoDriveToDistance;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreHighAutoBalance extends SequentialCommandGroup {
  /** Creates a new scoreHighAutoBalance. */
  public scoreHighAutoBalance(Drivetrain aDrivetrain, Superstructure cSuperstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> cSuperstructure.IntakeCone(), cSuperstructure),
    
    //new InstantCommand(() -> cSuperstructure.stopMotor(), cSuperstructure),
    new SetJointsToHighScoreCone(cSuperstructure).withTimeout(2),
    new SpitOutCone(cSuperstructure).withTimeout(1.5),
    new SetJointsToHome(cSuperstructure),
    new AutoDriveToDistance(aDrivetrain, -15),
    new AutoDriveToDistance(aDrivetrain, -15),
    new AutoDriveToDistance(aDrivetrain, -20),
    new AutoDriveToDistance(aDrivetrain, -55),
    

    new AutoBalance(aDrivetrain)
    

      );
  }
}
