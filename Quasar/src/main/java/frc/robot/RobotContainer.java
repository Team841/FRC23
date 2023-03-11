// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.autonomous.PIDControllers.*;
import frc.robot.commands.autonomous.Paths.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivetrain m_Drivetrain = new Drivetrain();

  private final Arm m_Arm = new Arm();
  
  private final Claw m_Claw = new Claw();
  
  private final Joystick m_driverCtrlLeft = new Joystick(C.OI.driverPortLeft);
  // private final PS4Controller m_driverCtrlLeft = new PS4Controller(C.OI.driverPortLeft);
  private final Joystick m_driverCtrlRight = new Joystick(C.OI.driverPortRight);

  private final Joystick m_codriverCtrl = new Joystick(C.OI.codriverPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_Drivetrain.setDefaultCommand(
      new RunCommand(() -> m_Drivetrain.Drive(m_driverCtrlLeft,m_driverCtrlRight),m_Drivetrain)
      );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     //Quick turn
    final JoystickButton qT = new JoystickButton(m_driverCtrlLeft, C.OI.kRB);
      qT.whenPressed(new InstantCommand(m_Drivetrain::setQuickTurn, m_Drivetrain));
      qT.whenReleased(new InstantCommand(m_Drivetrain::resetQuickTurn, m_Drivetrain));
    final JoystickButton AutoBalance = new JoystickButton(m_driverCtrlLeft, C.OI.kA);
      AutoBalance.whileHeld(new AutoBalance(m_Drivetrain));
    final JoystickButton AutoTurn = new JoystickButton(m_driverCtrlLeft, C.OI.kX);
      AutoTurn.whenPressed(new AutoTurn(m_Drivetrain, 270));
    final JoystickButton AutoDistance = new JoystickButton(m_driverCtrlLeft, C.OI.kB);
      AutoDistance.whenPressed(new AutoDriveToDistance(m_Drivetrain, 48));

    final JoystickButton upShoulder = new JoystickButton(m_codriverCtrl, C.OI.kRB);
      upShoulder.whileHeld(new InstantCommand(m_Arm::moveShoulderSlowUp, m_Arm));
      upShoulder.whenReleased(new InstantCommand(m_Arm::stopShoulder, m_Arm));
    final JoystickButton downShoulder = new JoystickButton(m_codriverCtrl, C.OI.kRT);
      downShoulder.whileHeld(new InstantCommand(m_Arm::moveShoulderSlowDown, m_Arm));
      downShoulder.whenReleased(new InstantCommand(m_Arm::stopShoulder, m_Arm));
    final JoystickButton upElbow = new JoystickButton(m_codriverCtrl, C.OI.kLB);
      upElbow.whileHeld(new InstantCommand(m_Arm::moveElbowSlowUp, m_Arm));
      upElbow.whenReleased(new InstantCommand(m_Arm::stopElbow, m_Arm));
    final JoystickButton downElbow = new JoystickButton(m_codriverCtrl, C.OI.kLT);
      downElbow.whileHeld(new InstantCommand(m_Arm::moveElbowSlowDown, m_Arm));
      downElbow.whenReleased(new InstantCommand(m_Arm::stopElbow, m_Arm));
    final JoystickButton stopArmMotors = new JoystickButton(m_codriverCtrl, C.OI.kA);
      stopArmMotors.whenPressed(new InstantCommand(m_Arm::stopAllMotors, m_Arm));

    final JoystickButton openClaw = new JoystickButton(m_codriverCtrl, C.OI.kX);
      openClaw.whenPressed(new InstantCommand(m_Claw::setClawOpen, m_Claw));
    final JoystickButton closeClaw = new JoystickButton(m_codriverCtrl, C.OI.kB);
      closeClaw.whenPressed(new InstantCommand(m_Claw::setClawClose, m_Claw));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous    
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new driveOffAutoBalance(m_Drivetrain);
  }

}
