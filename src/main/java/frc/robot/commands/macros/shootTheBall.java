// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Index;
import frc.robot.commands.shootingSystem.StopFlyWheel;
import frc.robot.commands.shootingSystem.ReverseFeeder;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootTheBall extends SequentialCommandGroup {
  /** Creates a new shootTheBall. */
  public shootTheBall(ShootingSystem shooter, Index index, int numBalls, double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ReverseFeeder(shooter), new FireInTheHole(shooter, index, numBalls, setpoint), new StopFlyWheel(shooter));
  }
}
