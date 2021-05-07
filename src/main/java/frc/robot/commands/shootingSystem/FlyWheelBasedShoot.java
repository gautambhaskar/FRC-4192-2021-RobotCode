// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingSystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.shooterPID;
import frc.robot.subsystems.ShootingSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlyWheelBasedShoot extends PIDCommand {
  /** Creates a new FlyWheelBasedShoot. */
  private Timer timer = new Timer();

  public FlyWheelBasedShoot(ShootingSystem m_shooter, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(0.04, 0, 0),
        // This should return the measurement
        () -> m_shooter.getFlywheelSpeed(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          m_shooter.setPower(output, -11);
          // Use the output here
        });
    timer.start();
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//timer.get() > shooterPID.maxRunTime;
  }
}