/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.drivePID;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveForDistance extends PIDCommand {
  /**
   * Creates a new DriveForDistance.
   */
  private Drivetrain drive;

  public DriveForDistance(Drivetrain m_drive, double m_distance) {
    super(
        // The controller that the command will use
        new PIDController(-0.075, 0, -0.012), // work with D to slow bot down.
        // This should return the measurement
        () -> m_drive.returnDrivetrainPosition(),
        // This should return the setpoint (can also be a constant)
        () -> m_distance,
        // This uses the output
        output -> {
          if ((output) > drivePID.autonMaxSpeed) {
            m_drive.arcadeDrive(drivePID.autonMaxSpeed, drivePID.kP * m_drive.returnAngle());
          } else if ((output) < -drivePID.autonMaxSpeed) {
            m_drive.arcadeDrive(-drivePID.autonMaxSpeed, drivePID.kP * m_drive.returnAngle());
          } else {
            m_drive.arcadeDrive((output), drivePID.kP * m_drive.returnAngle());
          }
        });
    drive = m_drive;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(drivePID.tolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint() && (Math.abs(drive.returnLeftEncoder().getVelocity()) < 200
        && (Math.abs(drive.returnRightEncoder().getVelocity()) < 200));
  }
}
