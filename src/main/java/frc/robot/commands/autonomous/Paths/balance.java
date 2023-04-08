package frc.robot.commands.autonomous.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.PIDControllers.AutoBalance;
import frc.robot.commands.autonomous.PIDControllers.AutoDriveToDistance;
import frc.robot.subsystems.Drivetrain;

public class balance extends SequentialCommandGroup {

    public balance(Drivetrain aDrivetrain){
        addCommands(
                new AutoDriveToDistance(aDrivetrain, -15),
                new AutoDriveToDistance(aDrivetrain, -15),
                new AutoDriveToDistance(aDrivetrain, -20),
                new AutoDriveToDistance(aDrivetrain, -55),


                new AutoBalance(aDrivetrain)
        );
    }
}
