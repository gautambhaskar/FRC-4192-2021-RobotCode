/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drive.DefaultDrive;

import frc.robot.commands.autonomous.RedSearchAutonA;
import frc.robot.commands.autonomous.RedSearchAutonB;
import frc.robot.commands.autonomous.UILAuton;
import frc.robot.commands.drive.DriveForDistance;
import frc.robot.commands.drive.DriveSetDistance;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.OuttakeSlowly;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.drive.PrecisionDrive;
import frc.robot.commands.drive.ResetGyroAngle;
import frc.robot.commands.hood.AlignHood;
import frc.robot.commands.hood.HoodGoingDown;
import frc.robot.commands.hood.HoodGoingUp;
import frc.robot.commands.hood.SetHood;
import frc.robot.commands.index.IndexIn;
import frc.robot.commands.index.IndexOut;
import frc.robot.commands.turret.TurretTurn;
import frc.robot.commands.macros.UnjamBall;
import frc.robot.commands.macros.shootTheBall;
import frc.robot.commands.shootingSystem.BasicRunShooter;
import frc.robot.commands.shootingSystem.FlyWheelBasedShoot;
import frc.robot.commands.shootingSystem.ReverseFeeder;
import frc.robot.commands.shootingSystem.StopFlyWheel;
import frc.robot.commands.shootingSystem.TwoStagePID;
import frc.robot.commands.macros.CloseRangeShootingMacro;
import frc.robot.commands.macros.ShootingMacro;
import frc.robot.commands.macros.TurretAlignmentMacro;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj.buttons.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private DoubleSupplier centerX;
    ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private NetworkTableEntry autonEntry = mainTab.add("Auton Selection", 0).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 1)).getEntry();
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
            () -> driveController.getX(Hand.kRight), 0.7);
    private final PrecisionDrive m_quarterSpeedDrive = new PrecisionDrive(m_drive,
            () -> driveController.getY(Hand.kLeft), () -> driveController.getX(Hand.kRight), 0.5);
    private final DriveStraight m_driveStraight = new DriveStraight(m_drive, () -> driveController.getY(Hand.kLeft));
    private final IntakeBalls m_intakeCommand = new IntakeBalls(m_intake, Constants.intakeSpeed, false);
    private final OuttakeSlowly m_outtakeSlowlyCommand = new OuttakeSlowly(m_intake, Constants.outtakeSlowlySpeed);
    private final IndexIn m_indexIn = new IndexIn(m_index, Constants.indexSpeed);
    private final IndexOut m_indexOut = new IndexOut(m_index, Constants.indexSpeed);
    private final TurretTurn m_turretTurnLeft = new TurretTurn(m_turret,
            () -> systemsController.getTriggerAxis(Hand.kLeft) * 15 / 100);
    private final TurretTurn m_turretTurnRight = new TurretTurn(m_turret,
            () -> -systemsController.getTriggerAxis(Hand.kRight) * 15 / 100);
    private final UnjamBall m_unjamBalls = new UnjamBall(m_index, m_shootingSystem, Constants.unjamBalls.ind_power,
            Constants.unjamBalls.s_power, Constants.unjamBalls.f_power);
    private final HoodGoingUp hoodUp = new HoodGoingUp(m_hood);
    private final HoodGoingDown hoodDown = new HoodGoingDown(m_hood);
    private final ShootingMacro m_shooterMacro = new ShootingMacro(m_drive, m_turret, m_shootingSystem, m_index, m_hood,
            5, 0, 8.5, false, 5);
    private final shootTheBall shootingBallsOnly = new shootTheBall(m_shootingSystem, m_index, 5, 8.5, 5);
    private final ResetGyroAngle resetAngle = new ResetGyroAngle(m_drive);
    // private final CloseRangeShootingMacro m_closeRangeMacro = new
    // CloseRangeShootingMacro(m_drive, m_turret, m_index, m_shootingSystem, m_hood,
    // -1);
    private final TestMotor m_testMotor = new TestMotor(m_motor, 0.3);
    private final SetHood m_setHood = new SetHood(m_hood);
    private final BasicRunShooter m_basicRunShooter = new BasicRunShooter(m_shootingSystem, 11, 0);
    private final SetIntake m_setIntake = new SetIntake(m_intake, false);
    private final TurretAlignmentMacro m_turretAlignmentMacro = new TurretAlignmentMacro(m_drive, m_turret, m_hood, 0,
            true);
    //private final FlyWheelBasedShoot m_flywheel = new FlyWheelBasedShoot(m_shootingSystem, 2000);
    private final StopFlyWheel m_flywheelStop = new StopFlyWheel(m_shootingSystem);
    private final TwoStagePID m_flywheelShoot = new TwoStagePID(m_shootingSystem, 8.5);
    private final ReverseFeeder m_reverseFeed = new ReverseFeeder(m_shootingSystem);
    // Autonomous Commands

    private final DriveForDistance zeroDistance = new DriveForDistance(m_drive, 0);
    private final UILAuton uilAutonDSMid = new UILAuton(m_drive, m_turret, m_shootingSystem, m_index, m_hood, m_intake,
            0);
    private final UILAuton uilAutonDSRight = new UILAuton(m_drive, m_turret, m_shootingSystem, m_index, m_hood,
            m_intake, -1);
    private final UILAuton uilAutonDSRight2 = new UILAuton(m_drive, m_turret, m_shootingSystem, m_index, m_hood,
            m_intake, -2);
    private final UILAuton uilAutonDSLeft = new UILAuton(m_drive, m_turret, m_shootingSystem, m_index, m_hood, m_intake,
            1);
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
    Trigger joystickYOnly = new Trigger(() -> Math.abs(driveController.getX(Hand.kRight)) < 0.05
            && Math.abs(driveController.getY(Hand.kLeft)) > 0.05 && driveController.getTriggerAxis(Hand.kRight) < 0.6
            && driveController.getTriggerAxis(Hand.kLeft) < 0.6);
    POVButton downSystems = new POVButton(systemsController, 180);
    POVButton upSystems = new POVButton(systemsController, 0);
    public RobotContainer(DoubleSupplier maxCenterX) {
        m_drive.setDefaultCommand(m_driveCommand);
        centerX = maxCenterX;
        // Configurehe button bindings
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
        driverLeftBumper.toggleWhenPressed(m_setIntake);
        driverXButton.toggleWhenPressed(m_intakeCommand);
        //driverBackButton.whenHeld(m_unjamBalls);
        driverAButton.whenHeld(m_outtakeSlowlyCommand);
        joystickYOnly.whileActiveOnce(m_driveStraight, true);



        // Systems Controller (Manual Control)
        systemsRightTrigger.whileActiveOnce(m_turretTurnRight);
        systemsLeftTrigger.whileActiveOnce(m_turretTurnLeft);
        systemsStartButton.whenHeld(m_indexIn);
        systemsAButton.whenPressed(m_shooterMacro, true);
        systemsBButton.whenPressed(hoodDown);
        systemsRightBumper.whenHeld(m_indexOut);
        //systemsYButton.toggleWhenPressed(m_flywheelShoot);
        systemsYButton.whenPressed(hoodUp);
        systemsBackButton.whenPressed(shootingBallsOnly);
        //systemsAButton.whenPressed(m_basicRunShooter);
        systemsXButton.whenPressed(m_flywheelStop);
        systemsLeftBumper.toggleWhenPressed(m_setIntake);
        downSystems.whenHeld(m_reverseFeed);
        upSystems.whenHeld(m_unjamBalls);
        // systemsBackButton.toggleWhenPressed();
        // systemsXButton.toggleWhenPressed(m_intakeCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        String[] autons = {"DSLeft", "DSRight", "DSMid"};
        SmartDashboard.putStringArray("Auto List", autons);
        String selected = SmartDashboard.getString("Auto Selector", "DSLeft");
        if (selected.equals("DSLeft")) {
                Globals.selectedAuton = "Left";
                Globals.chosenAuton.setString("Left");
            return uilAutonDSLeft;
        } else if (selected.equals("DSRight")) {
                Globals.selectedAuton = "Right";
                Globals.chosenAuton.setString("Right");
            return uilAutonDSRight;
        } else {
                Globals.selectedAuton = "Middle";
                Globals.chosenAuton.setString("Middle");
            return uilAutonDSMid;
        }
    }
}
