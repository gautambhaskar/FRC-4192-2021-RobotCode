
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

public class AutoIndex3 extends CommandBase {
  /** Creates a new AutoIndex. */
  private Index index;
  private boolean alreadyRun;
  private int numBalls;
  private int ballsShot;
  private boolean inThreshold;

  public AutoIndex3(Index m_index, int m_numBalls) {
    index = m_index;
    alreadyRun = false;
    numBalls = m_numBalls;
    ballsShot = 0;
    inThreshold = false;
    addRequirements(m_index);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballsShot=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Shooter is up to speed and hasn't shot a ball since it sped up, then run
    // index to fire a ball
    inThreshold = Globals.flyWheelSpeed > shooterPID.flyWheelSpeedMinimum
        && Globals.flyWheelSpeed < shooterPID.flyWheelSpeedMinimum + 100;
    if (inThreshold && Globals.feederSpeed > shooterPID.feederSpeedMinimum && alreadyRun==false) {
      index.run(Constants.indexSpeed);
      alreadyRun = true;
      // Once the index has run for long enough to fire a ball, stop running the index
    } else if (alreadyRun == true && Globals.feederSpeed < shooterPID.feederSpeedMinimum) {
      index.run(0);
      alreadyRun = false;
      ballsShot++;
      // Once the shooter has lost speed due to shooting the ball, set alreadyRun to
      // false
      // so that the next time it gets up to speed, a ball can be fired again
    } else {
      // by default, set index to 0 speed.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (numBalls == -1) {
      return false;
    } else {
      return ballsShot >= numBalls;
    }
  }
}