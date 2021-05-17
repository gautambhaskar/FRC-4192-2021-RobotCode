// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.index.AutoIndex3;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.commands.shootingSystem.FlyWheelBasedShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireInTheHole extends ParallelRaceGroup {
  /** Creates a new RunShooterWithAutoIndex. */
  public FireInTheHole(ShootingSystem shooter, Index index, int numBalls, double setpoint, double runTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FlyWheelBasedShoot(shooter, setpoint), new AutoIndex3(index, numBalls));
  }
}
