// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingSystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShootingSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoStagePID extends SequentialCommandGroup {
  /** Creates a new TwoStagePID. */
  public TwoStagePID(ShootingSystem m_shoot, double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FlyWheelBasedShoot(m_shoot, 0.04, 0, 0, setpoint, true), new FlyWheelBasedShoot(m_shoot, 0.02 /*test value */, 0, 0, setpoint, false));
  }
}
