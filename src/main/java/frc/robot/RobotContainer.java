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
import frc.robot.commands.PrecisionDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final XboxController driveController = new XboxController(Constants.driveController);
  private final XboxController systemsController = new XboxController(Constants.systemsController);

  private final DefaultDrive m_driveCommand = new DefaultDrive(m_drive, () -> driveController.getX(Hand.kRight),
      () -> driveController.getY(Hand.kLeft));
  private final PrecisionDrive m_halfSpeedDrive = new PrecisionDrive(m_drive, () -> driveController.getX(Hand.kRight),
      () -> driveController.getY(Hand.kLeft), 0.5);
  private final PrecisionDrive m_quarterSpeedDrive = new PrecisionDrive(m_drive,
      () -> driveController.getX(Hand.kRight), () -> driveController.getY(Hand.kLeft), 0.3);

  private final IntakeBalls m_intakeCommand = new IntakeBalls(m_intake, Constants.intakeSpeed);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  // Triggers
  Trigger rightTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kRight) > 0.6);
  Trigger leftTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kLeft) > 0.6);
  JoystickButton leftBumper = new JoystickButton(driveController, Constants.leftBumper);

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

    leftBumper.toggleWhenPressed(m_intakeCommand);

    // if (driveController.getX(GenericHID.Hand.kRight) > 0.05 ||
    // driveController.getY(GenericHID.Hand.kLeft) > 0.05) {

    // }
    // new JoystickButton(exampleController, XBoxController.Button.kX.value)
    // .and(new JoystickButton(exampleController, XboxController.Button.kY.value))
    // .whenActive(new ExampleCommand());
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
