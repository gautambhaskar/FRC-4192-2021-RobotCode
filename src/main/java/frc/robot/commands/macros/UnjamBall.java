/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShootingSystem;

public class UnjamBall extends CommandBase {
  private final Index m_index;
  private final ShootingSystem m_shooting;
  private final double m_indexPower;
  private final double m_shooterPower;
  private final double m_feederPower;

  /**
   * Creates a new IntakeBalls.
   */
  public UnjamBall(Index i_subsystem, ShootingSystem s_subsystem, double ind_power, double s_power, double f_power) {
    m_index = i_subsystem;
    m_shooting = s_subsystem;
    m_indexPower = ind_power;
    m_shooterPower = s_power;
    m_feederPower = f_power;
    addRequirements(m_index, m_shooting);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_index.run(-m_indexPower);
    m_shooting.setPower(-m_shooterPower, -m_feederPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.run(0);
    m_shooting.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
