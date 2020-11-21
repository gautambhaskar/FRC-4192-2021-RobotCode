/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.drivePID;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveStraight extends PIDCommand {
  /**
   * Creates a new DriveStraight.
   */

  public DriveStraight(Drivetrain m_drive, double m_forward) {
    super(
        // The controller that the command will use
        new PIDController(drivePID.kP, drivePID.kI, drivePID.kD),
        // This should return the measurement
        () -> m_drive.returnAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.arcadeDrive(m_forward, output);
          SmartDashboard.putNumber("angle", output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    // Configure additional PID options by calling `getController` here.
  }
}