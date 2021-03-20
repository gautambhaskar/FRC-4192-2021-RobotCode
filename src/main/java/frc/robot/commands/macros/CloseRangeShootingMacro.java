// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hood.AlignHood;
import frc.robot.commands.shootingSystem.ReverseFeeder;
import frc.robot.commands.turret.FieldBasedTurretTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CloseRangeShootingMacro extends SequentialCommandGroup {
  /** Creates a new CloseRangeShootingMacro. */
  public CloseRangeShootingMacro(Drivetrain m_drive, Turret m_turret, Index m_index, ShootingSystem m_shooter,
      Hood m_hood, int numBalls) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AlignHood(m_hood, false), new FieldBasedTurretTurn(m_drive, m_turret), new ReverseFeeder(m_shooter),
        new FireInTheHole(m_shooter, m_index, numBalls));
  }
}
