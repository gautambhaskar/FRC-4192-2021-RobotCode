// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingSystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSystem;

public class ReverseFeeder extends CommandBase {
  /** Creates a new ReverseFeeder. */
  private ShootingSystem x_shooter;
  private Timer timer = new Timer();

  public ReverseFeeder(ShootingSystem m_shooter) {
    x_shooter = m_shooter;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x_shooter.setPower(0, 3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    x_shooter.setPower(0, 0);
    DriverStation.reportError("reverseFeeder finished", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1;
  }
}
