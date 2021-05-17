// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ShootingSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretPlusShoot extends ParallelRaceGroup {
  /** Creates a new TurretPlusShoot. */
  public TurretPlusShoot(ShootingSystem m_shooter, Drivetrain m_drive, Turret m_turret, Hood m_hood, double turretOffset, boolean runInfinite, double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TurretAlignmentMacro(m_drive, m_turret, m_hood, turretOffset, runInfinite), new BeginShoot(m_shooter, setpoint));
  }
}
