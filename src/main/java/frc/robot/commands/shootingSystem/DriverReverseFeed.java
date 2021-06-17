// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSystem;

public class DriverReverseFeed extends CommandBase {
  /** Creates a new DriverReverseFeed. */
  private final ShootingSystem m_shooting;
  public DriverReverseFeed(ShootingSystem s_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooting = s_subsystem;
    addRequirements(m_shooting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooting.reverseFeedOnly(4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooting.reverseFeedOnly(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
