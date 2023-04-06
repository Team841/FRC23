package frc.robot.commands.autonomous.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetJointsToHighScoreCone;
import frc.robot.commands.SetJointsToHome;
import frc.robot.commands.SpitOutCone;
import frc.robot.subsystems.Superstructure;

public class scoreHighCone extends SequentialCommandGroup {

    //todo: test this code
    public scoreHighCone(Superstructure superstructure) {
        addCommands(
                new SetJointsToHighScoreCone(superstructure).withTimeout(2),
                new SpitOutCone(superstructure).withTimeout(1.5),
                new SetJointsToHome(superstructure)
        );
    }
}
