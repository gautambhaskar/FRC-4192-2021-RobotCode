// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.drivePID;
import frc.robot.commands.drive.DriveSetDistance;
import frc.robot.commands.macros.CloseRangeShootingMacro;
import frc.robot.commands.macros.ShootingMacro;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IACAuton extends SequentialCommandGroup {
  /** Creates a new IACAuton. */
  public IACAuton(Drivetrain m_drive, Turret m_turret, Index m_index, ShootingSystem m_shooter, Hood m_hood,
      Intake m_intake, double shooterSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CloseRangeShootingMacro(m_drive, m_turret, m_index, m_shooter, m_hood, 3, shooterSpeed),
        new DriveAndIntake(m_drive, m_intake, 20 * drivePID.feetToRotations, false),
        new DriveSetDistance(m_drive, -15 * drivePID.feetToRotations),
        new ShootingMacro(m_drive, m_turret, m_shooter, m_index, m_hood, 3, shooterSpeed),
        new DriveAndIntake(m_drive, m_intake, 15 * drivePID.feetToRotations, false),
        new DriveSetDistance(m_drive, -10 * drivePID.feetToRotations),
        new ShootingMacro(m_drive, m_turret, m_shooter, m_index, m_hood, 3, shooterSpeed),
        new DriveAndIntake(m_drive, m_intake, 10 * drivePID.feetToRotations, false),
        new DriveSetDistance(m_drive, -5 * drivePID.feetToRotations),
        new ShootingMacro(m_drive, m_turret, m_shooter, m_index, m_hood, 3, shooterSpeed),
        new DriveAndIntake(m_drive, m_intake, 5 * drivePID.feetToRotations, false),
        new DriveSetDistance(m_drive, -5 * drivePID.feetToRotations),
        new ShootingMacro(m_drive, m_turret, m_shooter, m_index, m_hood, 3, shooterSpeed));
  }
}
