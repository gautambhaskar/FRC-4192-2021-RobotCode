/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.fieldBasedTurretPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FieldBasedTurretTurn extends PIDCommand {
  /**
   * Creates a new FieldBasedTurretTurn.
   */
  
  public FieldBasedTurretTurn(Drivetrain m_drive, Turret m_turret) {
    super(
        // The controller that the command will use
        new PIDController(fieldBasedTurretPID.kP, fieldBasedTurretPID.kI, fieldBasedTurretPID.kD),
        // This should return the measurement
        () -> m_turret.getNativePosition(),
        // This should return the setpoint (can also be a constant)
        () -> m_drive.returnNativeAngle(),
        // This uses the output
        output -> {
          if (output > fieldBasedTurretPID.maxSpeed) {
            m_turret.turn(fieldBasedTurretPID.maxSpeed);
          } else if (output < -fieldBasedTurretPID.maxSpeed) {
            m_turret.turn(-fieldBasedTurretPID.maxSpeed);
          } else {
            m_turret.turn(output);
          }
          // Use the output here
        });
        addRequirements(m_drive, m_turret);
        getController().setTolerance(5);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
