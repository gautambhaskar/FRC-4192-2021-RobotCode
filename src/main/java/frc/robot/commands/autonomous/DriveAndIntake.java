// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveForDistance;
import frc.robot.commands.drive.DriveSetDistance;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndIntake extends ParallelRaceGroup {
  /** Creates a new DriveAndIntake. */
  public DriveAndIntake(Drivetrain m_drive, Intake m_intake, double m_distance, boolean endRaise) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(new DriveSetDistance(m_drive, m_distance), new IntakeBalls(m_intake, Constants.intakeSpeed, endRaise));
  }
}
