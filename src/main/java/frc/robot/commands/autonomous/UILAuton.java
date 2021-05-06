// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.drivePID;
import frc.robot.commands.drive.DriveSetDistance;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.macros.ShootingMacro;
import frc.robot.commands.autonomous.DriveAndIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootingSystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UILAuton extends SequentialCommandGroup {
  /** Creates a new UILAuton. */
  public UILAuton(Drivetrain drive, Turret turret, ShootingSystem shooter, Index index, Hood hood, Intake intake, int route) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    switch (route) {
    case 0: // turret side bumpers on line (DSMid)
      addCommands(new IntakeDown(intake), new ShootingMacro(drive, turret, shooter, index, hood, 3, 0, 2000), 
          new DriveSetDistance(drive, 3 * drivePID.feetToRotations));
      break;
    case -1: // the 1 on bumpers on line (DSLeft)
      addCommands(new IntakeDown(intake), new ShootingMacro(drive, turret, shooter, index, hood, 3, 20, 2000),
          new DriveAndIntake(drive, intake, 16 * drivePID.feetToRotations, false), new DriveSetDistance(drive, -7 * drivePID.feetToRotations), new ShootingMacro(drive, turret, shooter, index, hood, 3, 0, 2000));
      break;
    case 1: // the 1 on bumpers on line (DSRight)
      addCommands(new IntakeDown(intake), new ShootingMacro(drive, turret, shooter, index, hood, 3, -20, 2000),
          new DriveSetDistance(drive, 3 * drivePID.feetToRotations));
      break;
    }
  }
}
