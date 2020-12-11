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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetRotate extends PIDCommand {
  /**
   * Creates a new SetRotate.
   */

  public SetRotate(Drivetrain m_drive, double rotation) {
    super(
        // The controller that the command will use
        new PIDController(drivePID.rotatekP, 0, 0.0005),
        // This should return the measurement
        () -> m_drive.returnAngle(),
        // This should return the setpoint (can also be a constant)
        () -> rotation,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.arcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(2);
    m_drive.recalibrateAngle();
    addRequirements(m_drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
