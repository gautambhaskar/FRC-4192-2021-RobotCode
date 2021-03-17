/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.visionPosition.blueA;
import frc.robot.Constants.visionPosition.blueB;
import frc.robot.Constants.visionPosition.redA;
import frc.robot.Constants.visionPosition.redB;

import frc.robot.commands.drive.DefaultDrive;

import frc.robot.commands.autonomous.RedSearchAutonA;
import frc.robot.commands.autonomous.RedSearchAutonB;
import frc.robot.commands.drive.DriveForDistance;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.OuttakeSlowly;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.drive.PrecisionDrive;
import frc.robot.commands.hood.AlignHood;
import frc.robot.commands.index.IndexIn;
import frc.robot.commands.index.IndexOut;
import frc.robot.commands.turret.TurretTurn;
import frc.robot.commands.macros.UnjamBall;
import frc.robot.commands.shootingSystem.BasicRunShooter;
import frc.robot.commands.shootingSystem.RunShooter;
import frc.robot.commands.macros.CloseRangeShootingMacro;
import frc.robot.commands.macros.ShootingMacro;
import frc.robot.commands.autonomous.BlueSearchAutonA;
import frc.robot.commands.autonomous.BlueSearchAutonB;
import frc.robot.commands.testingSystem.TestMotor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TestingSystem;
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

  private final DoubleSupplier centerX;
  ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  NetworkTableEntry pathChosen = mainTab.add("Path Chosen", "none").getEntry();

  // Controllers
  private final XboxController driveController = new XboxController(Constants.driveController);
  private final XboxController systemsController = new XboxController(Constants.systemsController);

  // Subsystems
  private final Drivetrain m_drive = new Drivetrain();

  private final Intake m_intake = new Intake();
  private final ShootingSystem m_shootingSystem = new ShootingSystem();
  private final Index m_index = new Index();
  private final Turret m_turret = new Turret();
  private final TestingSystem m_motor = new TestingSystem();
  private final Hood m_hood = new Hood();

  // Commands
  private final DefaultDrive m_driveCommand = new DefaultDrive(m_drive, () -> driveController.getY(Hand.kLeft),
      () -> driveController.getX(Hand.kRight));
  private final PrecisionDrive m_halfSpeedDrive = new PrecisionDrive(m_drive, () -> driveController.getY(Hand.kLeft),
      () -> driveController.getX(Hand.kRight), 0.5);
  private final PrecisionDrive m_quarterSpeedDrive = new PrecisionDrive(m_drive, () -> driveController.getY(Hand.kLeft),
      () -> driveController.getX(Hand.kRight), 0.3);
  private final DriveStraight m_driveStraight = new DriveStraight(m_drive, () -> driveController.getY(Hand.kLeft));
  private final IntakeBalls m_intakeCommand = new IntakeBalls(m_intake, Constants.intakeSpeed, true);
  private final OuttakeSlowly m_outtakeSlowlyCommand = new OuttakeSlowly(m_intake, Constants.outtakeSlowlySpeed);
  private final IndexIn m_indexIn = new IndexIn(m_index, Constants.indexSpeed);
  private final IndexOut m_indexOut = new IndexOut(m_index, Constants.indexSpeed);
  private final TurretTurn m_turretTurnLeft = new TurretTurn(m_turret,
      () -> systemsController.getTriggerAxis(Hand.kLeft) * 7 / 10);
  private final TurretTurn m_turretTurnRight = new TurretTurn(m_turret,
      () -> -systemsController.getTriggerAxis(Hand.kRight) * 7 / 10);
  private final UnjamBall m_unjamBalls = new UnjamBall(m_index, m_shootingSystem, Constants.unjamBalls.ind_power,
      Constants.unjamBalls.s_power, Constants.unjamBalls.f_power);
  // private final AlignWithTarget m_alignWithTarget = new
  // AlignWithTarget(m_turret);
  private final ShootingMacro m_shooterMacro = new ShootingMacro(m_drive, m_turret, m_shootingSystem, m_index, m_hood);
  private final CloseRangeShootingMacro m_closeRangeMacro = new CloseRangeShootingMacro(m_drive, m_turret, m_index,
      m_shootingSystem, m_hood);
  private final TestMotor m_testMotor = new TestMotor(m_motor, 0.3);
  private final AlignHood m_alignHood = new AlignHood(m_hood, true);
  private final AlignHood m_alignHoodReverse = new AlignHood(m_hood, false);
  private final RunShooter m_runShooter = new RunShooter(m_shootingSystem);
  private final BasicRunShooter m_basicRunShooter = new BasicRunShooter(m_shootingSystem, 0.5, 0.5);
  private final SetIntake m_raiseIntake = new SetIntake(m_intake, true);
  private final SetIntake m_lowerIntake = new SetIntake(m_intake, false);

  // Autonomous Commands
  private final BlueSearchAutonA autonBlueA = new BlueSearchAutonA(m_drive, m_intake);
  private final BlueSearchAutonB autonBlueB = new BlueSearchAutonB(m_drive, m_intake);
  private final RedSearchAutonA autonRedA = new RedSearchAutonA(m_drive, m_intake);
  private final RedSearchAutonB autonRedB = new RedSearchAutonB(m_drive, m_intake);
  private final DriveForDistance zeroDistance = new DriveForDistance(m_drive, 0);
  // private final DistanceAuton m_distanceauton = new DistanceAuton(m_drive);

  // Triggers
  Trigger driverRightTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kRight) > 0.6);
  Trigger driverLeftTrigger = new Trigger(() -> driveController.getTriggerAxis(Hand.kLeft) > 0.6);
  JoystickButton driverLeftBumper = new JoystickButton(driveController, Constants.leftBumper);
  JoystickButton driverRightBumper = new JoystickButton(driveController, Constants.rightBumper);
  JoystickButton driverAButton = new JoystickButton(driveController, Constants.aButton);
  JoystickButton driverBackButton = new JoystickButton(driveController, Constants.backButton);
  JoystickButton driverYButton = new JoystickButton(driveController, Constants.yButton);
  JoystickButton driverXButton = new JoystickButton(driveController, Constants.xButton);
  JoystickButton driverStartButton = new JoystickButton(driveController, Constants.startButton);
  JoystickButton systemsLeftBumper = new JoystickButton(systemsController, Constants.leftBumper);
  JoystickButton systemsRightBumper = new JoystickButton(systemsController, Constants.rightBumper);
  JoystickButton systemsStartButton = new JoystickButton(systemsController, Constants.startButton);
  JoystickButton systemsBackButton = new JoystickButton(systemsController, Constants.backButton);
  JoystickButton systemsXButton = new JoystickButton(systemsController, Constants.xButton);
  JoystickButton systemsAButton = new JoystickButton(systemsController, Constants.aButton);
  JoystickButton systemsBButton = new JoystickButton(systemsController, Constants.bButton);
  JoystickButton systemsYButton = new JoystickButton(systemsController, Constants.yButton);
  Trigger systemsRightTrigger = new Trigger(() -> systemsController.getTriggerAxis(Hand.kRight) > 0.2);
  Trigger systemsLeftTrigger = new Trigger(() -> systemsController.getTriggerAxis(Hand.kLeft) > 0.2);
  Trigger joystickYOnly = new Trigger(
      () -> Math.abs(driveController.getX(Hand.kRight)) < 0.05 && Math.abs(driveController.getY(Hand.kLeft)) > 0.05
          && driveController.getTriggerAxis(Hand.kRight) < 0.6 && driveController.getTriggerAxis(Hand.kLeft) < 0.6);

  public RobotContainer(DoubleSupplier maxCenterX) {
    m_drive.setDefaultCommand(m_driveCommand);
    centerX = maxCenterX;
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
    // Driver Controller
    driverRightTrigger.whileActiveOnce(m_halfSpeedDrive);
    driverLeftTrigger.whileActiveOnce(m_quarterSpeedDrive);
    driverLeftBumper.toggleWhenPressed(m_intakeCommand);
    driverRightBumper.toggleWhenPressed(m_shooterMacro);
    driverXButton.toggleWhenPressed(m_closeRangeMacro);
    driverBackButton.whenHeld(m_unjamBalls);
    driverStartButton.whenHeld(m_testMotor);
    driverAButton.whenHeld(m_outtakeSlowlyCommand);
    joystickYOnly.whileActiveOnce(m_driveStraight, false);

    // Systems Controller (Manual Control)
    systemsRightTrigger.whileActiveOnce(m_turretTurnRight);
    systemsLeftTrigger.whileActiveOnce(m_turretTurnLeft);
    systemsStartButton.whenHeld(m_indexIn);
    systemsAButton.whenPressed(m_alignHood);
    systemsBButton.whenPressed(m_alignHoodReverse);
    systemsXButton.toggleWhenPressed(m_runShooter);
    // systemsXButton.whenHeld(m_basicRunShooter);
    systemsBackButton.whenHeld(m_indexOut);
    systemsLeftBumper.toggleWhenPressed(m_raiseIntake);
    systemsRightBumper.toggleWhenPressed(m_lowerIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (centerX.getAsDouble() > blueA.left && centerX.getAsDouble() <
    // blueA.right) {
    // pathChosen.setString("Blue A");
    // return autonBlueA;
    // }

    // else if (centerX.getAsDouble() > blueB.left && centerX.getAsDouble() <
    // blueB.right) {
    // pathChosen.setString("Blue B");
    // return autonBlueB;
    // }

    // else if (centerX.getAsDouble() > redA.left && centerX.getAsDouble() <
    // redA.right) {
    // pathChosen.setString("Red A");
    // return autonRedA;
    // }

    // else if (centerX.getAsDouble() > redB.left && centerX.getAsDouble() <
    // redB.right) {
    // pathChosen.setString("Red B");
    // return autonRedB;
    // }

    // else {
    // pathChosen.setString("none");
    // return zeroDistance;
    // }
    //
    return autonRedB;
  }
}
