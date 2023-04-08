package frc.robot.commands.autonomous.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.PIDControllers.AutoBalance;
import frc.robot.commands.autonomous.PIDControllers.AutoDriveToDistance;
import frc.robot.commands.autonomous.coneOutTake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;

public class LowCubeBalance extends SequentialCommandGroup {

    public LowCubeBalance(Superstructure p_Superstructure, Drivetrain aDrivetrain) {
        addCommands(
                new coneOutTake(p_Superstructure).withTimeout(2),
                new AutoDriveToDistance(aDrivetrain, -15),
                new AutoDriveToDistance(aDrivetrain, -15),
                new AutoDriveToDistance(aDrivetrain, -20),
                new AutoDriveToDistance(aDrivetrain, -55),


                new AutoBalance(aDrivetrain)

        );
    }
}
