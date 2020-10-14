/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.OuttakeSlowly;
import frc.robot.commands.PrecisionDrive;
import frc.robot.commands.IndexIn;
import frc.robot.commands.IndexOut;
import frc.robot.commands.TurretTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShootingSystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final XboxController driveController = new XboxController(Constants.driveController);
  private final XboxController systemsController = new XboxController(Constants.systemsController);

  // Subsystems
  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final ShootingSystem m_ShootingSystem = new ShootingSystem();

  // Commands
  private final DefaultDrive m_driveCommand = new DefaultDrive(m_drive, () -> driveController.getX(Hand.kRight),
      () -> driveController.getY(Hand.kLeft));
  private final PrecisionDrive m_halfSpeedDrive = new PrecisionDrive(m_drive, () -> driveController.getX(Hand.kRight),
      () -> driveController.getY(Hand.kLeft), 0.5);
  private final PrecisionDrive m_quarterSpeedDrive = new PrecisionDrive(m_drive,
      () -> driveController.getX(Hand.kRight), () -> driveController.getY(Hand.kLeft), 0.3);
  private final IntakeBalls m_intakeCommand = new IntakeBalls(m_intake, Constants.intakeSpeed);
  private final OuttakeSlowly m_outtakeSlowlyCommand = new OuttakeSlowly(m_intake, Constants.outtakeSlowlySpeed);
  private final IndexIn m_indexInCommand = new IndexIn(m_ShootingSystem, Constants.indexSpeed);
  private final IndexOut m_indexOutCommand = new IndexOut(m_ShootingSystem, Constants.indexSpeed);
  private final TurretTurn m_turretTurnLeft = new TurretTurn(m_ShootingSystem, Constants.turretTurn);
  private final TurretTurn m_turretTurnRight = new TurretTurn(m_ShootingSystem, -Constants.turretTurn);

  // Triggers
  Trigger rightTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kRight) > 0.6);
  Trigger leftTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kLeft) > 0.6);
  JoystickButton leftBumper = new JoystickButton(driveController, Constants.leftBumper);
  JoystickButton startButton = new JoystickButton(systemsController, Constants.startButton);
  JoystickButton backButton = new JoystickButton(systemsController, Constants.backButton);
  JoystickButton aButton = new JoystickButton(driveController, Constants.aButton);
  Trigger rightTriggerSubsystems = new Trigger(() -> systemsController.getTriggerAxis(Hand.kRight) > 0.6);
  Trigger leftTriggerSubsystems = new Trigger(() -> systemsController.getTriggerAxis(Hand.kLeft) > 0.6);

  public RobotContainer() {
    m_drive.setDefaultCommand(m_driveCommand);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    rightTrigger.whenActive(m_halfSpeedDrive);
    leftTrigger.whenActive(m_quarterSpeedDrive);
    rightTriggerSubsystems.whenActive(m_turretTurnRight);
    leftTriggerSubsystems.whenActive(m_turretTurnLeft);
    startButton.whenHeld(m_indexInCommand);
    backButton.whenHeld(m_indexOutCommand);
    leftBumper.toggleWhenPressed(m_intakeCommand);
    aButton.whenHeld(m_outtakeSlowlyCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return null;
  }
}
