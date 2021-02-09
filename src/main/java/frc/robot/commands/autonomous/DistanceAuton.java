/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveForDistance;
import frc.robot.commands.drive.DriveSetDistance;
import frc.robot.commands.drive.RecalPosition;
import frc.robot.commands.drive.SetRotate;
import frc.robot.commands.macros.ShootingMacro;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DistanceAuton extends SequentialCommandGroup {
  /**
   * Creates a new DistanceAuton.
   */
  private Drivetrain m_drive;

  public DistanceAuton(Drivetrain drive, Turret turret, ShootingSystem shooter, Index index, Hood hood) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveSetDistance(drive, 15));
    m_drive = drive;
  }
}
