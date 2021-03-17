// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSystem;

public class BasicRunShooter extends CommandBase {
  /** Creates a new BasicRunShooter. */
  private final ShootingSystem m_shooter;
  private final double s_power;
  private final double f_power;

  public BasicRunShooter(ShootingSystem shooter, double shooterPower, double feederPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    s_power = shooterPower;
    f_power = feederPower;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setPower(s_power, f_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
