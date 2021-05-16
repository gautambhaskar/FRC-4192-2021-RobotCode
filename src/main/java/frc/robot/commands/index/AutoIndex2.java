// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.Constants.shooterPID;
import frc.robot.subsystems.Index;

public class AutoIndex2 extends CommandBase {
  /** Creates a new AutoIndex2. */
  private boolean inThreshold;
  private Index m_index;
  private Timer timer = new Timer();
  private double runTime;
  private boolean timerStarted;

  public AutoIndex2(Index m_index, double runTime) {
    inThreshold = false;
    this.m_index = m_index;
    this.runTime = runTime;
    addRequirements(m_index);
    timerStarted = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inThreshold = Globals.flyWheelSpeed > shooterPID.flyWheelSpeedMinimum
        && Globals.flyWheelSpeed < shooterPID.flyWheelSpeedMinimum + 100;
    if (inThreshold) {
      m_index.run(Constants.indexSpeed);
      timer.reset();
      timer.start();
      timerStarted = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(runTime)) {
      timer.reset();
      timer.stop();
      timerStarted = false;
      return true;
    }
    return false;
  }
}
