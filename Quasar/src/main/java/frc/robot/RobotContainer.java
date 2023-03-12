// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.autonomous.PIDControllers.*;
import frc.robot.commands.autonomous.Paths.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Create subsystems */
  private final Drivetrain m_Drivetrain = new Drivetrain();
  private final Arm m_Arm = new Arm();
  private final Claw m_Claw = new Claw();
  
  private final CommandXboxController m_codriverCtrl = new CommandXboxController(C.OI.codriverPort);
  private final CommandPS4Controller m_driverCtrlLeft = new CommandPS4Controller(C.OI.driverPortLeft);
  private final CommandPS4Controller m_driverCtrlRight = new CommandPS4Controller(C.OI.driverPortRight);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_Drivetrain.setDefaultCommand(
      new RunCommand(() -> m_Drivetrain.Drive(m_driverCtrlLeft,m_driverCtrlRight),m_Drivetrain));
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Button.html */

     //Quick turn
    final Trigger qT = m_driverCtrlLeft.R1();
      qT.onTrue(new InstantCommand(m_Drivetrain::setQuickTurn, m_Drivetrain));
      qT.onFalse(new InstantCommand(m_Drivetrain::resetQuickTurn, m_Drivetrain));
    final Trigger AutoBalance = m_driverCtrlLeft.cross();
      AutoBalance.whileTrue(new AutoBalance(m_Drivetrain));
    final Trigger AutoTurn = m_driverCtrlLeft.square();
      AutoTurn.onTrue(new AutoTurn(m_Drivetrain, 270));
    final Trigger AutoDistance = m_driverCtrlLeft.circle();
      AutoDistance.onTrue(new AutoDriveToDistance(m_Drivetrain, 48));

    final Trigger upShoulder = m_codriverCtrl.rightBumper();
      upShoulder.whileTrue(new InstantCommand(m_Arm::moveShoulderSlowUp, m_Arm));
      upShoulder.onFalse(new InstantCommand(m_Arm::stopShoulder, m_Arm));
    final Trigger downShoulder = m_codriverCtrl.rightTrigger();
      downShoulder.whileTrue(new InstantCommand(m_Arm::moveShoulderSlowDown, m_Arm));
      downShoulder.onFalse(new InstantCommand(m_Arm::stopShoulder, m_Arm));
    final Trigger upElbow = m_codriverCtrl.leftBumper(); 
      upElbow.whileTrue(new InstantCommand(m_Arm::moveElbowSlowUp, m_Arm));
      upElbow.onFalse(new InstantCommand(m_Arm::stopElbow, m_Arm));
    final Trigger downElbow = m_codriverCtrl.leftTrigger();
      downElbow.whileTrue(new InstantCommand(m_Arm::moveElbowSlowDown, m_Arm));
      downElbow.onFalse(new InstantCommand(m_Arm::stopElbow, m_Arm));
    final Trigger stopArmMotors = m_codriverCtrl.a(); 
      stopArmMotors.onTrue(new InstantCommand(m_Arm::stopAllMotors, m_Arm));
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
