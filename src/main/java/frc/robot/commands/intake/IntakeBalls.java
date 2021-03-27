/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Intake;

public class IntakeBalls extends CommandBase {
  private final Intake m_intake;
  private final double m_intakeSpeed;
  private final boolean m_endRaise;

  /**
   * Creates a new IntakeBalls.
   */

  public IntakeBalls(Intake subsystem, double speed, boolean endRaise) {
    m_intake = subsystem;
    m_intakeSpeed = speed;
    m_endRaise = endRaise;
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.lower();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Globals.drivetrainDistanceReached) {
      m_intake.intake(m_intakeSpeed);
    } else {
      m_intake.intake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.intake(0);
    if (m_endRaise) {
      m_intake.raise();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
