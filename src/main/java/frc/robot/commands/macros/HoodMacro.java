// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hood.ResetHood;
import frc.robot.commands.hood.AlignHood;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoodMacro extends SequentialCommandGroup {
  /** Creates a new HoodMacro. */

  public HoodMacro(Turret m_turret, Hood m_hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetHood(m_hood), new AlignHood(m_turret, m_hood));
  }
}
