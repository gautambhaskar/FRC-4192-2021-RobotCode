// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopFlyWheel extends CommandBase {
  private ShootingSystem m_shooter;

  public StopFlyWheel(ShootingSystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setPower(0, 0);
  }
  @Override
  public void execute() {

  }
  @Override
  public void end(boolean interrupted){
    m_shooter.setPower(0, 0);
  }
  @Override
  public boolean isFinished() {
    m_shooter.setPower(0, 0);
    return true;
  }
}
