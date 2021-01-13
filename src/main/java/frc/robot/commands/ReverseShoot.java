// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShootingSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReverseShoot extends SequentialCommandGroup {
  /** Creates a new ReverseShoot. */
  public ReverseShoot(ShootingSystem m_shooter, Index m_index) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ReverseFeeder(m_shooter), new RunShooterWithAutoIndex(m_index, m_shooter));
  }
}
