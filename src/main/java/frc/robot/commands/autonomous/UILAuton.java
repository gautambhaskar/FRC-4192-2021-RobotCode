// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.drivePID;
import frc.robot.commands.drive.DriveSetDistance;
import frc.robot.commands.macros.ShootingMacro;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Hood;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UILAuton extends SequentialCommandGroup {
  /** Creates a new UILAuton. */
  public UILAuton(Drivetrain drive, Turret turret, ShootingSystem shooter, Index index, Hood hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShootingMacro(drive, turret, shooter, index, hood, 3, 4200), new DriveSetDistance(drive, 3*drivePID.feetToRotations));
  }
}
