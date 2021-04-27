// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.Constants.shooterPID;
import frc.robot.subsystems.Index;

public class AutoIndex extends CommandBase {
  /** Creates a new AutoIndex. */
  private Index index;
  private boolean alreadyRun;
  private Timer timer = new Timer();
  private Timer timer2 = new Timer();
  private int numBalls;
  private int ballsShot;
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
  private NetworkTableEntry ballsFired;

  public AutoIndex(Index m_index, int m_numBalls) {
    index = m_index;
    alreadyRun = false;
    numBalls = m_numBalls;
    ballsShot = 0;
    addRequirements(m_index);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballsFired = tuningTab.add("Balls Firedd", 0).getEntry();
    timer.start();
    timer2.start();
    ballsShot = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Shooter is up to speed and hasn't shot a ball since it sped up, then run
    // index to fire a ball
    if (Globals.flyWheelSpeed > shooterPID.flyWheelSpeedMinimum && alreadyRun == false && timer2.get() > Constants.indexRunTime+1) {
      index.run(Constants.indexSpeed);
      timer.reset();
      timer2.reset();
      alreadyRun = true;
      ballsShot++;
      ballsFired.setNumber(ballsShot);
      // Once the index has run for long enough to fire a ball, stop running the index
    } else if (Globals.flyWheelSpeed > shooterPID.flyWheelSpeedMinimum && alreadyRun == true
        && timer.get() > Constants.indexRunTime) {
      index.run(0);
      
      alreadyRun = false;
      // Once the shooter has lost speed due to shooting the ball, set alreadyRun to
      // false
      // so that the next time it gets up to speed, a ball can be fired again
    } else if (Globals.flyWheelSpeed < shooterPID.flyWheelSpeedMinimum && timer.get() > Constants.indexRunTime) {
      alreadyRun = false;
      index.run(0);
      // by default, set index to 0 speed.
    } else {
      index.run(0);
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
