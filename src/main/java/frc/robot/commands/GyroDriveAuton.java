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

public class GyroDriveAuton extends CommandBase {
  /**
   * Creates a new GyroDriveAuton.
   */
  private final Drivetrain m_drive;
  private Timer timer;
  private double m_duration; 

  public GyroDriveAuton(Drivetrain subsystem, double duration) {
    m_drive = subsystem;
    timer = new Timer();
    m_duration = duration; 
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    m_drive.recalibrateAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0.3, -drivePID.kP * m_drive.returnAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > m_duration;
  }
}
