package frc.robot.commands.autonomous.Paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetJointsToHighScoreCone;
import frc.robot.commands.SetJointsToHome;
import frc.robot.commands.SetJointsToMidScoreCube;
import frc.robot.commands.SpitOutCone;
import frc.robot.commands.SpitOutCube;
import frc.robot.commands.autonomous.PIDControllers.AutoBalance;
import frc.robot.commands.autonomous.PIDControllers.AutoDriveToDistance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;


public class scoreMidCubeAutoBalance extends SequentialCommandGroup {
  /** Creates a new scoreMidCubeAutoBalance. */
  public scoreMidCubeAutoBalance(Drivetrain aDrivetrain, Superstructure cSuperstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> cSuperstructure.IntakeCube(), cSuperstructure),
    
    //new InstantCommand(() -> cSuperstructure.stopMotor(), cSuperstructure),
    new SetJointsToMidScoreCube(cSuperstructure).withTimeout(2),
    new SpitOutCube(cSuperstructure).withTimeout(1.5),
    new SetJointsToHome(cSuperstructure),
    new AutoDriveToDistance(aDrivetrain, -15),
    new AutoDriveToDistance(aDrivetrain, -15),
    new AutoDriveToDistance(aDrivetrain, -20),
    new AutoDriveToDistance(aDrivetrain, -55),

    new AutoBalance(aDrivetrain)
    );
  }
}
