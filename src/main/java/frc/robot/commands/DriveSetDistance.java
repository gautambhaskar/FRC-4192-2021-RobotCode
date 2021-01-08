/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.drivePID;
import frc.robot.subsystems.Drivetrain;

public class DriveSetDistance extends CommandBase {
  /**
   * Creates a new DriveSetDistance.
   */
  Drivetrain m_drive;
  double m_distance;
  double m_speed;

  public DriveSetDistance(Drivetrain drive, double distance, double speed) {
    m_drive = drive;
    m_distance = distance;
    m_speed = speed;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.recalibrateAngle();
    m_drive.recalibratePosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, -drivePID.kP * m_drive.returnAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.returnDrivetrainPosition() - m_distance) < 10;
  }
}
