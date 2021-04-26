/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hood.AlignHood;
import frc.robot.commands.turret.AlignWithTarget;
import frc.robot.commands.turret.FieldBasedTurretTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurretAlignmentMacro extends SequentialCommandGroup {
  /**
   * Creates a new TurretAlignmentMacro.
   */
  public TurretAlignmentMacro(Drivetrain m_drive, Turret m_turret, Hood m_hood) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AlignHood(m_hood, false), new FieldBasedTurretTurn(m_drive, m_turret), new AlignWithTarget(m_turret)); // new
    // new ResetHood(m_hood),
    // FieldBasedTurretTurn(m_drive,
    // m_turret),new AlignHood(m_turret, m_hood)
  }
};