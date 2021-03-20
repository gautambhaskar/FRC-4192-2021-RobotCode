// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shootingSystem.RunShooter;
import frc.robot.commands.index.AutoIndex;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShootingSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireInTheHole extends ParallelCommandGroup {
  /** Creates a new RunShooterWithAutoIndex. */
  public FireInTheHole(ShootingSystem shooter, Index index, int numBalls) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(new RunShooter(shooter), new AutoIndex(index, numBalls));
  }
}
