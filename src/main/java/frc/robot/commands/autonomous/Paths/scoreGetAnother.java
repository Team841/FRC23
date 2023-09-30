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
            //1. move arm high position
            //new SetJointsToHighScoreCube(aSuperstructure).withTimeout(2),
            //2. spit out
            //new SpitOutCube(aSuperstructure).withTimeout(1.5),
            //3. bring arm to home
            //new SetJointsToHome(aSuperstructure).withTimeout(2)
                       
            //4. drive back 11ft
            //new AutoDriveToDistance(aDrivetrain, -40),
            //5. turn 180
            //new AutoTurn(aDrivetrain, 180),
            //6. move arm to floor position
            //new SetJointsToGroundPickup(aSuperstructure).withTimeout(2),
            //7. turn on intake
            new InstantCommand(() -> aSuperstructure.IntakeCube(), aSuperstructure)
            //8. drive back 21.6 inches
           //new AutoDriveToDistance(aDrivetrain, 21.6),
            //9. stop intake motors
            //new InstantCommand(() -> aSuperstructure.stopMotor(), aSuperstructure),
            //10. turn 180
            //new AutoTurn(aDrivetrain, 180),
            //11. drive back
            //new AutoDriveToDistance(aDrivetrain, -222.6),
            //12. move arm to mid position
            //new SetJointsToMidCube(aSuperstructure).withTimeout(2),
            //13. spit out cube
            //new SpitOutCube(aSuperstructure).withTimeout(1.5)*/
        );
    }
}
