// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.drivePID;
import frc.robot.commands.drive.RotateInPlace;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedSearchAutonA extends SequentialCommandGroup {
  /** Creates a new RedSearchAutonA. */
  public RedSearchAutonA(Drivetrain m_drive, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveAndIntake(m_drive, m_intake, 5 * drivePID.feetToRotations), new RotateInPlace(m_drive, 26));
    //  new DriveAndIntake(m_drive, m_intake, 5.6 * drivePID.feetToRotations));
    //, new
    // RotateInPlace(m_drive, -98.13),
    // new DriveAndIntake(m_drive, m_intake, 7.9 * drivePID.feetToRotations), new
    // RotateInPlace(m_drive, 71.57),
    // new DriveAndIntake(m_drive, m_intake, 12.5 * drivePID.feetToRotations));
    // Replace 0 with correct distances
  }
}
