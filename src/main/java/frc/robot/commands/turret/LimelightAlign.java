// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Globals;
import frc.robot.Constants.limelightPID;
import frc.robot.Globals.LimelightShuffleboard;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAlign extends PIDCommand {
  /** Creates a new LimelightAlign. */
  private boolean runInfinite;
  private Turret m_turret;
  private Timer timer = new Timer();
  private boolean timerStarted = false;

  public LimelightAlign(Turret m_turret, boolean runInfinite) {
    super(
        // The controller that the command will use
        new PIDController(0.02, 0.045,
            0),
        // This should return the measurement
        () -> m_turret.limelightOffset(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          m_turret.turn(output);
        });
    this.runInfinite = runInfinite;
    this.m_turret = m_turret;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (runInfinite) {
      return false;
    }
    if(m_turret.limelightOffset()<5&&!timerStarted){
      timerStarted=true;
      
      timer.start();
      timer.reset();
      timer.start();
    }
    if(Math.abs(m_turret.limelightOffset()) < limelightPID.tolerance || (timer.get()>1&&m_turret.limelightOffset()<5&&timerStarted)){
      timerStarted=false;
      timer.stop();
      timer.reset();
      timer.stop();
      return true;
    } else {
      return false;
    }
  }
}
