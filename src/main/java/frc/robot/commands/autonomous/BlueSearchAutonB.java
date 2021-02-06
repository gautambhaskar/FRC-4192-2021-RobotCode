// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.drive.DriveForDistance;
import frc.robot.commands.drive.RotateInPlace;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueSearchAutonB extends SequentialCommandGroup {
  /** Creates a new BlueSearchAuton. */
  public BlueSearchAutonB(Drivetrain m_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveForDistance(m_drive, 0), new RotateInPlace(m_drive, -45), new DriveForDistance(m_drive, 0), new RotateInPlace(m_drive, 90), new DriveForDistance(m_drive, 0), 
    new RotateInPlace(m_drive, -45), new DriveForDistance(m_drive, 0));
    // Change zeroes to correct distances
  }
}
