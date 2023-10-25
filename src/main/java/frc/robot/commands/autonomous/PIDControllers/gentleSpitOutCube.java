package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.C;
import frc.robot.subsystems.Superstructure;

public class gentleSpitOutCube extends CommandBase {
  private Superstructure c_Superstructure;
  /** Creates a new SpitOutCone. */
  // this file was made to spit out a cube with less power for auto. NOT
  // COMPLETE
  public gentleSpitOutCube(Superstructure superstructure) {
    c_Superstructure = superstructure;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(superstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    c_Superstructure.gentleSpitOutCube();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_Superstructure.setIntakeMotor(0); // stop the intake when its done
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Stop the command on the w ait command
  }
}
