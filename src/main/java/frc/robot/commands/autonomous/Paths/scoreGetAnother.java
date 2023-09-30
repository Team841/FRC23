package frc.robot.commands.autonomous.Paths;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.PIDControllers.*;

public class scoreGetAnother extends SequentialCommandGroup{
    public scoreGetAnother(Superstructure aSuperstructure, Drivetrain aDrivetrain) {
        addCommands(
            new SetJointsToHighScoreCube(aSuperstructure).withTimeout(2),
            new SpitOutCube(aSuperstructure).withTimeout(1.5),
            new SetJointsToHome(aSuperstructure).withTimeout(2)
            /*new AutoDriveToDistance(aDrivetrain, -204),
            new AutoTurn(aDrivetrain, 180),
            new SetJointsToGroundPickup(aSuperstructure).withTimeout(2),
            new InstantCommand(() -> aSuperstructure.IntakeCube(), aSuperstructure),
            new AutoDriveToDistance(aDrivetrain, 21.6),
            new InstantCommand(() -> aSuperstructure.stopMotor(), aSuperstructure),
            new AutoTurn(aDrivetrain, 180),
            new AutoDriveToDistance(aDrivetrain, -222.6),
            new SetJointsToMidCube(aSuperstructure).withTimeout(2),
            new SpitOutCube(aSuperstructure).withTimeout(1.5)*/
        );
    }
}
