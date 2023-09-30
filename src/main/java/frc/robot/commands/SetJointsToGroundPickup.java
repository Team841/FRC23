package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.C;
import frc.robot.subsystems.Superstructure;

public class SetJointsToGroundPickup extends CommandBase {

    private Superstructure cSuperstructure;

    public SetJointsToGroundPickup(Superstructure c_Superstructure) {
        // Use addRequirements() here to declare subsystem dependencies.
        cSuperstructure = c_Superstructure;
        addRequirements(cSuperstructure);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cSuperstructure.buttonGround();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return cSuperstructure.isAtPosition(C.Superstructure.StateMachinePositions.Ground);
    }
}
