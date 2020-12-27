/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.drivePID;
import frc.robot.commands.AlignWithTarget;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DistanceAuton;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.OuttakeSlowly;
import frc.robot.commands.PrecisionDrive;
import frc.robot.commands.IndexIn;
import frc.robot.commands.IndexOut;
import frc.robot.commands.TurretTurn;
import frc.robot.commands.UnjamBall;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ShootingMacro;
import frc.robot.commands.BasicAuton;
import frc.robot.commands.TestMotor;
import frc.robot.commands.TurretAlignmentMacro;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TestingSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  // Controllers
  private final XboxController driveController = new XboxController(Constants.driveController);
  private final XboxController systemsController = new XboxController(Constants.systemsController);

  // Subsystems
  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final ShootingSystem m_ShootingSystem = new ShootingSystem();
  private final Index m_Index = new Index();
  private final Turret m_Turret = new Turret();
  private final TestingSystem m_motor = new TestingSystem();

  // Commands
  private final DefaultDrive m_driveCommand = new DefaultDrive(m_drive, () -> driveController.getY(Hand.kLeft),
      () -> driveController.getX(Hand.kRight));
  private final PrecisionDrive m_halfSpeedDrive = new PrecisionDrive(m_drive, () -> driveController.getY(Hand.kLeft),
      () -> driveController.getX(Hand.kRight), 0.5);
  private final PrecisionDrive m_quarterSpeedDrive = new PrecisionDrive(m_drive, () -> driveController.getY(Hand.kLeft),
      () -> driveController.getX(Hand.kRight), 0.3);
  private final DriveStraight m_driveStraight = new DriveStraight(m_drive, () -> driveController.getY(Hand.kLeft));
  private final IntakeBalls m_intakeCommand = new IntakeBalls(m_intake, Constants.intakeSpeed);
  private final OuttakeSlowly m_outtakeSlowlyCommand = new OuttakeSlowly(m_intake, Constants.outtakeSlowlySpeed);
  private final IndexIn m_indexInCommand = new IndexIn(m_Index, Constants.indexSpeed);
  private final IndexOut m_indexOutCommand = new IndexOut(m_Index, Constants.indexSpeed);
  private final TurretTurn m_turretTurnLeft = new TurretTurn(m_Turret,
      () -> systemsController.getTriggerAxis(Hand.kLeft) * 7 / 10);
  private final TurretTurn m_turretTurnRight = new TurretTurn(m_Turret,
      () -> -systemsController.getTriggerAxis(Hand.kRight) * 7 / 10);
  private final RunShooter m_runShooter = new RunShooter(m_ShootingSystem);
  private final UnjamBall m_unjamBalls = new UnjamBall(m_Index, m_ShootingSystem, Constants.unjamBalls.ind_power,
      Constants.unjamBalls.s_power, Constants.unjamBalls.f_power);
  // private final AlignWithTarget m_alignWithTarget = new
  // AlignWithTarget(m_Turret);
  private final TurretAlignmentMacro m_turretMacro = new TurretAlignmentMacro(m_drive, m_Turret);
  private final ShootingMacro m_shooterMacro = new ShootingMacro(m_drive, m_Turret, m_ShootingSystem);
  private final TestMotor m_testMotor = new TestMotor(m_motor, 0.3);

  // Autonomous Commands
  private final BasicAuton m_basicauton = new BasicAuton(m_drive);
  // private final DistanceAuton m_distanceauton = new DistanceAuton(m_drive);
  private final DriveForDistance m_distanceauton = new DriveForDistance(m_drive, 40);

  // Triggers
  Trigger rightTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kRight) > 0.6);
  Trigger leftTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kLeft) > 0.6);
  JoystickButton leftBumper = new JoystickButton(driveController, Constants.leftBumper);
  JoystickButton systemsStartButton = new JoystickButton(systemsController, Constants.startButton);
  JoystickButton systemsBackButton = new JoystickButton(systemsController, Constants.backButton);
  JoystickButton aButton = new JoystickButton(driveController, Constants.aButton);
  JoystickButton systemsYButton = new JoystickButton(systemsController, Constants.yButton);
  JoystickButton driverBackButton = new JoystickButton(driveController, Constants.backButton);
  JoystickButton driverYButton = new JoystickButton(driveController, Constants.yButton);
  JoystickButton driverStartButton = new JoystickButton(driveController, Constants.startButton);
  Trigger rightTriggerSubsystems = new Trigger(() -> systemsController.getTriggerAxis(Hand.kRight) > 0.2);
  Trigger leftTriggerSubsystems = new Trigger(() -> systemsController.getTriggerAxis(Hand.kLeft) > 0.2);
  Trigger joystickYOnly = new Trigger(
      () -> Math.abs(driveController.getX(Hand.kRight)) < 0.05 && Math.abs(driveController.getY(Hand.kLeft)) > 0.05
          && driveController.getTriggerAxis(Hand.kRight) < 0.6 && driveController.getTriggerAxis(Hand.kLeft) < 0.6);

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
    rightTrigger.whileActiveOnce(m_halfSpeedDrive);
    leftTrigger.whileActiveOnce(m_quarterSpeedDrive);
    rightTriggerSubsystems.whileActiveOnce(m_turretTurnRight);
    leftTriggerSubsystems.whileActiveOnce(m_turretTurnLeft);
    systemsStartButton.whenHeld(m_indexInCommand);
    systemsBackButton.whenHeld(m_indexOutCommand);
    leftBumper.toggleWhenPressed(m_intakeCommand);
    aButton.whenHeld(m_outtakeSlowlyCommand);
    systemsYButton.toggleWhenPressed(m_shooterMacro);
    driverBackButton.whenHeld(m_unjamBalls);
    joystickYOnly.whileActiveOnce(m_driveStraight, false);
    // joystickYOnly.whileActiveOnce(m_driveStraight);
    driverStartButton.whenHeld(m_testMotor);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (Constants.trajectoryMapping) {

      RamseteCommand ramseteCommand = new RamseteCommand(Robot.testTrajectory, m_drive::returnPose,
          new RamseteController(drivePID.kB, drivePID.kZeta),
          new SimpleMotorFeedforward(drivePID.kS, drivePID.kV, drivePID.kA), drivePID.kDriveKinematics,
          m_drive::returnWheelSpeeds, new PIDController(drivePID.kPDriveVel, 0, 0),
          new PIDController(drivePID.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drive::tankDrive, m_drive);

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_drive.tankDrive(0, 0));
    } else {
      return m_distanceauton;
    }
  }
}
