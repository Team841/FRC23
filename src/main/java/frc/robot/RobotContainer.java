// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final Superstructure m_Superstructure = new Superstructure();
  
  private final CommandXboxController m_codriverCtrl = new CommandXboxController(C.OI.codriverPort);
  private final CommandPS4Controller m_driverCtrl = new CommandPS4Controller(C.OI.driverPortLeft);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_chooser.setDefaultOption("Score low cube", new scoreLowCube(m_Drivetrain, m_Superstructure));
    m_chooser.addOption("Score high cone auto balance", new scoreHighAutoBalance(m_Drivetrain, m_Superstructure));
    m_chooser.addOption("Score high cone", new scoreHighCone(m_Superstructure));
    m_chooser.addOption("Balance", new balance(m_Drivetrain));
    m_chooser.addOption("Score low cube Balance", new LowCubeBalance(m_Superstructure, m_Drivetrain));
    m_chooser.addOption("score get another", new scoreGetAnother(m_Superstructure, m_Drivetrain));
    SmartDashboard.putData("Auto Chooser", m_chooser);


    // Configure the button bindings
    configureButtonBindings();
    m_Drivetrain.setDefaultCommand(
      new RunCommand(() -> m_Drivetrain.Drive(m_driverCtrl.getRightX(), m_driverCtrl.getLeftY()),m_Drivetrain));

    m_Superstructure.setDefaultCommand(
      new RunCommand(() -> m_Superstructure.updatejointoffsets(m_codriverCtrl.getRightY(),m_codriverCtrl.getLeftY()), m_Superstructure));
  }

  private void configureButtonBindings() {

    /* https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Button.html */

    /* Driver Commands */
    //Quick turn
    final Trigger qT = m_driverCtrl.R1();
      qT.onTrue(new InstantCommand(m_Drivetrain::setQuickTurn, m_Drivetrain));
      qT.onFalse(new InstantCommand(m_Drivetrain::resetQuickTurn, m_Drivetrain));
    /* final Trigger AutoBalance = m_driverCtrl.cross();
      AutoBalance.whileTrue(new AutoBalance(m_Drivetrain));
    final Trigger AutoTurn = m_driverCtrl.square();
      AutoTurn.onTrue(new AutoTurn(m_Drivetrain, 270));
    final Trigger AutoDistance = m_driverCtrl.circle();
      AutoDistance.onTrue(new AutoDriveToDistance(m_Drivetrain, 48)); */
    final Trigger BrakeOn = m_driverCtrl.cross();
    BrakeOn.onTrue(new InstantCommand(m_Drivetrain::BrakeOn, m_Drivetrain));
    final Trigger BrakeOff = m_driverCtrl.square();
    BrakeOff.onTrue(new InstantCommand(m_Drivetrain::BrakeOff, m_Drivetrain));

    /* Co driver commands */

    
    final Trigger e = m_codriverCtrl.b();
   e.onTrue(new InstantCommand(m_Superstructure::buttonGround, m_Superstructure));
    final Trigger s = m_codriverCtrl.x();
   s.whileTrue(new InstantCommand(m_Superstructure::buttonHome, m_Superstructure));
    final Trigger tcube = m_codriverCtrl.rightTrigger();
    tcube.whileTrue(new InstantCommand(m_Superstructure::buttonTopScoreCube, m_Superstructure));
    final Trigger MidScoreCube = m_codriverCtrl.rightBumper();
    MidScoreCube.whileTrue(new InstantCommand(m_Superstructure::buttonMidScoreCube, m_Superstructure));
    final Trigger tcone = m_codriverCtrl.leftTrigger();
    tcone.whileTrue(new InstantCommand(m_Superstructure::buttonTopScoreCone, m_Superstructure));
    final Trigger mcone = m_codriverCtrl.leftBumper();
    mcone.whileTrue(new InstantCommand(m_Superstructure::buttonMidScoreCone, m_Superstructure));
    final Trigger w = m_codriverCtrl.back();
    w.onTrue(new InstantCommand(m_Superstructure::setCube, m_Superstructure));
    final Trigger i = m_codriverCtrl.start();
    i.onTrue(new InstantCommand(m_Superstructure::setCone, m_Superstructure));


    /*final Trigger upShoulder = m_codriverCtrl.rightBumper();
    upShoulder.whileTrue(new InstantCommand(m_Superstructure::moveShoulderSlowUp, m_Superstructure));
    upShoulder.onFalse(new InstantCommand(m_Superstructure::stopShoulder, m_Superstructure));
    final Trigger downShoulder = m_codriverCtrl.rightTrigger();
    downShoulder.whileTrue(new InstantCommand(m_Superstructure::moveShoulderSlowDown, m_Superstructure));
    downShoulder.onFalse(new InstantCommand(m_Superstructure::stopShoulder, m_Superstructure));
    final Trigger upElbow = m_codriverCtrl.leftBumper();
    upElbow.whileTrue(new InstantCommand(m_Superstructure::moveElbowSlowUp, m_Superstructure));
    upElbow.onFalse(new InstantCommand(m_Superstructure::stopElbow, m_Superstructure));
    final Trigger downElbow = m_codriverCtrl.leftTrigger();
    downElbow.whileTrue(new InstantCommand(m_Superstructure::moveElbowSlowDown, m_Superstructure));
    downElbow.onFalse(new InstantCommand(m_Superstructure::stopElbow, m_Superstructure));
    final Trigger stopArmMotors = m_codriverCtrl.a();
    stopArmMotors.onTrue(new InstantCommand(m_Superstructure::stopJoints, m_Superstructure));*/

    final Trigger toggleIntake = m_codriverCtrl.y();
    toggleIntake.onTrue(new InstantCommand(m_Superstructure::toggleIntakeIn, m_Superstructure));
    final Trigger toggleIntakeOut = m_codriverCtrl.a();
    toggleIntakeOut.onTrue(new InstantCommand(m_Superstructure::toggleIntakeOut, m_Superstructure)); 
   
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous    
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new scoreDriveOff(m_Drivetrain);
    //return new driveoff(m_Drivetrain, m_Superstructure);
    // return new scoreHighAutoBalance(m_Drivetrain, m_Superstructure);
    return m_chooser.getSelected();
  }

}
