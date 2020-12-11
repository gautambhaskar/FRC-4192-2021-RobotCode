/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.drivePID;
import frc.robot.subsystems.Drivetrain;

public class DriveStraightForTime extends CommandBase {
  /**
   * Creates a new DriveForTime.
   */
  private Drivetrain m_drive;
  private double m_time;
  private double m_fwd;
  private Timer m_timer;

  public DriveStraightForTime(Drivetrain drive, double forward, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_time = time;
    m_fwd = forward;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_drive.recalibrateAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_fwd, -drivePID.kP * m_drive.returnAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_time;
  }
}
