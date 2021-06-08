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
  private boolean endNearSetpoint;
  private double setpoint;
  private ShootingSystem shooter;
  public FlyWheelBasedShoot(ShootingSystem m_shooter, double kP, double kI, double kD, double setpoint, boolean endNearSetpoint) {
    super(
        // The controller that the command will use
        //0.04, 0, 0
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> m_shooter.getFlywheelSpeed(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          m_shooter.setPower(8.5, -11);
          // Use the output here
        });
    timer.start();
    this.endNearSetpoint = endNearSetpoint;
    this.setpoint = setpoint;
    shooter = m_shooter;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endNearSetpoint) {
      return Math.abs(shooter.getFlywheelSpeed() - setpoint) == 100;//timer.get() > shooterPID.maxRunTime;
    }
    return false;    
  }
}