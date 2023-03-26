// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.C;
import frc.robot.subsystems.Superstructure;

public class coneOutTake extends CommandBase {
    /** Creates a new coneOutTake. */

    private Superstructure c_Superstructure;
    private double time = Timer.getFPGATimestamp();
    public coneOutTake(Superstructure superstructure) {
        c_Superstructure = superstructure;
        addRequirements(superstructure);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        c_Superstructure.setIntakeMotor(C.Superstructure.Intake.TalonPercentPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        c_Superstructure.setIntakeMotor(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() > (time+3.0));
    }
}
