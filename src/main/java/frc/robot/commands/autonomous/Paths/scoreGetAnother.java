package frc.robot.commands.autonomous.Paths;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.PIDControllers.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;

public class scoreGetAnother extends SequentialCommandGroup {
  public scoreGetAnother(Superstructure aSuperstructure,
                         Drivetrain aDrivetrain) {
    addCommands(
        // 1. move arm high position
        new SetJointsToHighScoreCube(aSuperstructure).withTimeout(2),
        // 2. spit out
        new SpitOutCube(aSuperstructure).withTimeout(1.5),
        // 3. bring arm to home
        new SetJointsToHome(aSuperstructure).withTimeout(2),
        // 4. drive back 11ft to pick up cube

        new AutoDriveToDistance(aDrivetrain, -1),
        new AutoDriveToDistance(aDrivetrain, -5),
        new AutoDriveToDistance(aDrivetrain, -100),
        new AutoDriveToDistance(aDrivetrain, -50),
        new AutoDriveToDistance(aDrivetrain, -30),
        new AutoDriveToDistance(aDrivetrain, -10),
        // 5. turn 180
        new AutoTurn(aDrivetrain, 179).withTimeout(2),
        // 6. turn on intake
        new InstantCommand(() -> aSuperstructure.IntakeCube(), aSuperstructure),
        // 7. set to ground
        new SetJointsToGroundPickup(aSuperstructure).withTimeout(2),
        // stop intake motors
        new InstantCommand(() -> aSuperstructure.stopMotor(), aSuperstructure),
        // bring arm home
        new SetJointsToHome(aSuperstructure).withTimeout(2),
        // turn 180
        new AutoTurn(aDrivetrain, 180).withTimeout(2),
        // drive back
        new AutoDriveToDistance(aDrivetrain, 70),
        new AutoDriveToDistance(aDrivetrain, 50),
        new AutoDriveToDistance(aDrivetrain, 30),
        // spit out cube
        new SpitOutCube(aSuperstructure).withTimeout(1.5),
        new SetJointsToHome(aSuperstructure).withTimeout(2)

    );
  }
}
